/**
 * @file arduino.ino
 * @brief Main firmware for Arduino Mega 2560 educational robotics platform
 * @version 0.8.0
 *
 * Educational robotics platform firmware for MAE 162 course.
 * Provides real-time motor control, sensor integration, and
 * communication with Raspberry Pi 5 via TLV protocol over UART.
 *
 * Two-tier scheduling architecture:
 *
 *   Hard real-time (ISR-driven — unaffected by loop() blocking):
 *     TIMER3_OVF_vect  10 kHz  Stepper pulse generation (StepperManager)
 *
 *   Soft real-time (millis-based, runs in loop() with interrupts enabled):
 *     taskUART          50 Hz  UART RX/TX — loop() keeps USART2_RX_vect alive;
 *                              RX/TX stay out of the ISR path entirely
 *     taskMotors       200 Hz  DC control compute + apply + feedback refresh
 *     taskSafety      100 Hz  button reads + heartbeat/battery safety checks
 *     taskSensors     100 Hz  IMU, lidar, ultrasonic, voltage updates (I2C/ADC)
 *     taskUserIO        20 Hz  LED animations, NeoPixel status
 *
 * Initialization Order (setup):
 *  1. Debug serial (Serial0)
 *  2. Scheduler (millis-based soft scheduler)
 *  3. MessageCenter (Serial2 + TLV codec)
 *  4. SensorManager (I2C, ADC)
 *  5. UserIO (GPIO, NeoPixel)
 *  6. ServoController (PCA9685 via I2C)
 *  7. StepperManager (Timer3 — also starts stepper ISR)
 *  8. DC Motors (PWM pins, encoder counters)
 *  9. Attach encoder ISRs
 * 10. Register soft-scheduler tasks
 * 11. ISRScheduler::init() — LAST: starts Timer1 ISR and Timer4 PWM only
 *
 * Main Loop:
 * - Scheduler::tick() executes highest-priority ready soft task
 */

#include <util/atomic.h>

// ============================================================================
// INCLUDES
// ============================================================================

// Core configuration
#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/ISRScheduler.h"
#include "src/SystemManager.h"

// Communication
#include "src/modules/MessageCenter.h"
#include "src/modules/DebugLog.h"
#include "src/modules/LoopMonitor.h"
#include "src/modules/SafetyManager.h"
#include "src/messages/TLV_Payloads.h"

// DC motor control
#include "src/modules/EncoderCounter.h"
#include "src/modules/VelocityEstimator.h"
#include "src/drivers/DCMotor.h"

// Stepper and servo control
#include "src/modules/StepperManager.h"
#include "src/drivers/StepperMotor.h"
#include "src/drivers/ServoController.h"

// Sensors and user I/O
#include "src/modules/SensorManager.h"
#include "src/modules/UserIO.h"
#include "src/drivers/IMUDriver.h"
#include "src/drivers/NeoPixelDriver.h"

// ============================================================================
// Compiler Guards
// ============================================================================
#ifndef SERIAL_RX_BUFFER_SIZE
#error "SERIAL_RX_BUFFER_SIZE not defined. Add -DSERIAL_RX_BUFFER_SIZE=256 to compiler flags."
#elif SERIAL_RX_BUFFER_SIZE < 256
#error "SERIAL_RX_BUFFER_SIZE < 256, which is recommanded for 250 kbps."
#endif

#ifndef SERIAL_TX_BUFFER_SIZE
#error "SERIAL_TX_BUFFER_SIZE not defined. Add -DSERIAL_TX_BUFFER_SIZE=256 to compiler flags."
#elif SERIAL_TX_BUFFER_SIZE < 256
#error "SERIAL_TX_BUFFER_SIZE < 256, which is recommanded for 250 kbps."
#endif

// ============================================================================
// GLOBAL OBJECTS
// ============================================================================

// Encoder instances (2x or 4x mode per config.h)
#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif

#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif

#if ENCODER_3_MODE == ENCODER_2X
EncoderCounter2x encoder3;
#else
EncoderCounter4x encoder3;
#endif

#if ENCODER_4_MODE == ENCODER_2X
EncoderCounter2x encoder4;
#else
EncoderCounter4x encoder4;
#endif

// Velocity estimators (edge-time algorithm)
EdgeTimeVelocityEstimator velocityEst1;
EdgeTimeVelocityEstimator velocityEst2;
EdgeTimeVelocityEstimator velocityEst3;
EdgeTimeVelocityEstimator velocityEst4;

// Motor controller arrays
DCMotor dcMotors[NUM_DC_MOTORS];

static uint32_t g_lastLoopUs = 0;
static uint16_t g_loopGapLastUs = 0;
static uint16_t g_loopGapPeakUs = 0;
static uint16_t g_rxBacklogPeak = 0;
static uint16_t g_txPendingPeak = 0;
static uint16_t g_statusTaskAvgUs = 0;
static uint16_t g_statusTaskPeakUs = 0;
static uint16_t g_statusTaskMaxUs = 0;
static uint16_t g_flushAvgUs = 0;
static uint16_t g_flushPeakUs = 0;
static uint16_t g_flushMaxUs = 0;
static volatile uint32_t g_uartDor2Count = 0;
static volatile uint32_t g_uartFe2Count = 0;
static volatile uint32_t g_motorRoundCount = 0;
static volatile uint32_t g_motorRequestedRound = 0;
static volatile uint32_t g_motorComputedRound = 0;
static volatile uint32_t g_motorAppliedRound = 0;
static volatile uint8_t g_motorCurrentSlot = 0;
static volatile uint8_t g_motorComputeSeq = 0;
static volatile uint8_t g_motorPreparedSeq = 0;
static volatile uint8_t g_motorAppliedSeq = 0;
static volatile uint32_t g_motorMissedRoundCount = 0;
static volatile uint32_t g_motorLateComputeCount = 0;
static volatile uint32_t g_motorReusedOutputCount = 0;
static volatile uint32_t g_motorCrossRoundComputeCount = 0;
static volatile bool g_motorRoundReady = false;
static volatile bool g_motorOutputsReady = false;
static volatile bool g_motorComputeBusy = false;

static void snapshotUart2FaultCounts(uint32_t &dor2, uint32_t &fe2) {
  noInterrupts();
  dor2 = g_uartDor2Count;
  fe2 = g_uartFe2Count;
  interrupts();
}

static void snapshotMotorControlSync(uint32_t &roundCount,
                                     uint32_t &requestedRound,
                                     uint32_t &computedRound,
                                     uint32_t &appliedRound,
                                     uint8_t &slot,
                                     uint8_t &computeSeq,
                                     uint8_t &appliedSeq,
                                     uint32_t &missedRoundCount,
                                     uint32_t &lateComputeCount,
                                     uint32_t &reusedOutputCount,
                                     uint32_t &crossRoundComputeCount,
                                     bool &computeBusy) {
  noInterrupts();
  roundCount = g_motorRoundCount;
  requestedRound = g_motorRequestedRound;
  computedRound = g_motorComputedRound;
  appliedRound = g_motorAppliedRound;
  slot = g_motorCurrentSlot;
  computeSeq = g_motorComputeSeq;
  appliedSeq = g_motorAppliedSeq;
  missedRoundCount = g_motorMissedRoundCount;
  lateComputeCount = g_motorLateComputeCount;
  reusedOutputCount = g_motorReusedOutputCount;
  crossRoundComputeCount = g_motorCrossRoundComputeCount;
  computeBusy = g_motorComputeBusy;
  interrupts();
}

static uint16_t clampElapsedUs(uint32_t elapsedUs) {
  return (elapsedUs > 0xFFFFUL) ? 0xFFFFU : (uint16_t)elapsedUs;
}

static uint16_t timerTicksToUs(uint16_t ticks) {
  return (uint16_t)((ticks + 1U) >> 1);
}

static void recordAuxTiming(uint16_t elapsedUs,
                            uint16_t &avgUs,
                            uint16_t &peakUs,
                            uint16_t &maxUs) {
  if (avgUs == 0U) {
    avgUs = elapsedUs;
  } else {
    avgUs = (uint16_t)(((uint32_t)avgUs * 7U + (uint32_t)elapsedUs) >> 3);
  }
  if (elapsedUs > peakUs) {
    peakUs = elapsedUs;
  }
  if (elapsedUs > maxUs) {
    maxUs = elapsedUs;
  }
}

struct StatusSnapshot {
  SystemState state;
  const char *linkState;
  const char *healthState;
  const char *servoCtrlState;
  const char *servoOutputState;
  const char *servoI2CState;
  const char *batteryState;
  const char *servoRailState;
  char liveBuf[24];
  char latchedBuf[24];
  char loopBuf[24];
  uint16_t vbatMv;
  uint16_t v5Mv;
  uint16_t vservoMv;
  uint32_t heartbeatAgeMs;
  uint16_t heartbeatTimeoutMs;
  uint32_t lastByteAgeMs;
  uint16_t servoEnabledMask;
  uint16_t freeRam;
  uint32_t txDroppedFrames;
  uint16_t debugDroppedBytes;
  uint16_t loopGapLastUs;
  uint16_t loopGapPeakUs;
  uint32_t motorRoundCount;
  uint32_t motorRequestedRound;
  uint32_t motorComputedRound;
  uint32_t motorAppliedRound;
  uint8_t motorSlot;
  uint8_t motorComputeSeq;
  uint8_t motorAppliedSeq;
  uint32_t missedRoundDelta;
  uint32_t lateComputeDelta;
  uint32_t reusedOutputDelta;
  uint32_t crossRoundDelta;
  bool motorComputeBusy;
  uint16_t pidAvgUs;
  uint16_t pidPeakUs;
  uint16_t pidMaxUs;
  uint16_t pidBudgetUs;
  uint16_t pidRoundAvgUs;
  uint16_t pidRoundPeakUs;
  uint16_t pidRoundMaxUs;
  uint16_t pidRoundBudgetUs;
  uint16_t stepAvgUs;
  uint16_t stepPeakUs;
  uint16_t stepMaxUs;
  uint16_t stepBudgetUs;
  uint16_t motorAvgUs;
  uint16_t motorPeakUs;
  uint16_t motorMaxUs;
  uint16_t motorBudgetUs;
  uint16_t sensorAvgUs;
  uint16_t sensorPeakUs;
  uint16_t sensorMaxUs;
  uint16_t sensorBudgetUs;
  uint16_t uartAvgUs;
  uint16_t uartPeakUs;
  uint16_t uartMaxUs;
  uint16_t uartBudgetUs;
  uint16_t ioAvgUs;
  uint16_t ioPeakUs;
  uint16_t ioMaxUs;
  uint16_t ioBudgetUs;
  uint16_t debugAvgUs;
  uint16_t debugPeakUs;
  uint16_t debugMaxUs;
  uint16_t flushAvgUs;
  uint16_t flushPeakUs;
  uint16_t flushMaxUs;
  uint16_t rxBytesWindow;
  uint16_t rxFramesWindow;
  uint16_t rxTlvsWindow;
  uint16_t rxHeartbeatsWindow;
  uint16_t txBytesWindow;
  uint16_t txFramesWindow;
  uint16_t uartRxErrors;
  uint32_t dor2Count;
  uint32_t fe2Count;
  uint16_t crcErrorCount;
  uint16_t frameLenErrorCount;
  uint16_t tlvErrorCount;
  uint16_t oversizeErrorCount;
  uint16_t rxAvailable;
  uint16_t rxPeak;
  uint16_t txPending;
  uint16_t txPeak;
};

static StatusSnapshot g_statusSnapshot = {};
static bool g_statusSnapshotPending = false;
static uint8_t g_statusSnapshotChunk = 0;

static const char *stateName(SystemState state) {
  switch (state) {
    case SystemState::SYS_STATE_INIT:    return "INIT";
    case SystemState::SYS_STATE_IDLE:    return "IDLE";
    case SystemState::SYS_STATE_RUNNING: return "RUNNING";
    case SystemState::SYS_STATE_ERROR:   return "ERROR";
    case SystemState::SYS_STATE_ESTOP:   return "ESTOP";
    default:                             return "?";
  }
}

static void appendToken(char *buffer, size_t size, bool &first, const char *token) {
  size_t used = strlen(buffer);
  if (used >= (size - 1U) || token == nullptr || token[0] == '\0') {
    return;
  }
  if (!first) {
    strlcat(buffer, ",", size);
  }
  strlcat(buffer, token, size);
  first = false;
}

static void formatErrorFlags(uint8_t flags, char *buffer, size_t size) {
  if (size == 0) {
    return;
  }
  buffer[0] = '\0';
  bool first = true;
  if (flags & ERR_UNDERVOLTAGE)  appendToken(buffer, size, first, "UV");
  if (flags & ERR_OVERVOLTAGE)   appendToken(buffer, size, first, "OV");
  if (flags & ERR_ENCODER_FAIL)  appendToken(buffer, size, first, "ENC");
  if (flags & ERR_I2C_ERROR)     appendToken(buffer, size, first, "I2C");
  if (flags & ERR_IMU_ERROR)     appendToken(buffer, size, first, "IMU");
  if (flags & ERR_LIVENESS_LOST) appendToken(buffer, size, first, "HB");
  if (flags & ERR_LOOP_OVERRUN)  appendToken(buffer, size, first, "LOOP");
  if (first) {
    strlcpy(buffer, "none", size);
  }
}

static void formatLoopFaults(uint8_t mask, char *buffer, size_t size) {
  if (size == 0) {
    return;
  }
  buffer[0] = '\0';
  bool first = true;
  if (mask & LOOP_FAULT_PID_ISR)     appendToken(buffer, size, first, "pid");
  if (mask & LOOP_FAULT_PID_ROUND)   appendToken(buffer, size, first, "pidr");
  if (mask & LOOP_FAULT_STEPPER_ISR) appendToken(buffer, size, first, "step");
  if (mask & LOOP_FAULT_MOTOR_TASK)  appendToken(buffer, size, first, "motor");
  if (mask & LOOP_FAULT_SENSOR_ISR)  appendToken(buffer, size, first, "sensor");
  if (mask & LOOP_FAULT_UART_TASK)   appendToken(buffer, size, first, "uart");
  if (mask & LOOP_FAULT_USERIO)      appendToken(buffer, size, first, "io");
  if (first) {
    strlcpy(buffer, "none", size);
  }
}

static const char *batteryStateName() {
  if (!SensorManager::isBatteryPresent()) {
    return "NO BATTERY";
  }
  if (SensorManager::isBatteryOvervoltage()) {
    return "HIGH";
  }
  if (SensorManager::isBatteryCritical()) {
    return "CRITICAL";
  }
  if (SensorManager::isBatteryLow()) {
    return "LOW";
  }
  return "OK";
}

static const char *servoRailStateName() {
  float servoV = SensorManager::getServoVoltage();
  if (servoV < 1.0f) {
    return "OFF";
  }
  if (servoV < VSERVO_MIN_PRESENT_V) {
    return "LOW";
  }
  if (servoV > 9.0f) {
    return "HIGH";
  }
  return "OK";
}

static const char *systemHealthName(SystemState state, uint8_t liveFlags) {
  if (state == SystemState::SYS_STATE_ESTOP || state == SystemState::SYS_STATE_ERROR) {
    return "FAULT";
  }
  if (liveFlags != ERR_NONE) {
    return "WARN";
  }
  return "OK";
}

static uint8_t computeLiveErrorFlags(SystemState state) {
  uint8_t flags = ERR_NONE;
  if (SensorManager::isBatteryLow())         flags |= ERR_UNDERVOLTAGE;
  if (SensorManager::isBatteryOvervoltage()) flags |= ERR_OVERVOLTAGE;
  if (!MessageCenter::isHeartbeatValid() && state == SystemState::SYS_STATE_RUNNING)
                                           flags |= ERR_LIVENESS_LOST;
  if (ServoController::hasI2CError())        flags |= ERR_I2C_ERROR;
#if IMU_ENABLED
  if (!SensorManager::isIMUAvailable())      flags |= ERR_IMU_ERROR;
#endif
  if (LoopMonitor::getFaultMask() != 0)      flags |= ERR_LOOP_OVERRUN;
  for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    if (dcMotors[i].isEncoderFailed()) {
      flags |= ERR_ENCODER_FAIL;
      break;
    }
  }
  return flags;
}

static void updateDebugWindowPeaks() {
  uint16_t rxBacklog = (uint16_t)RPI_SERIAL.available();
  if (rxBacklog > g_rxBacklogPeak) {
    g_rxBacklogPeak = rxBacklog;
  }

  uint16_t txPending = MessageCenter::getTxPendingBytes();
  if (txPending > g_txPendingPeak) {
    g_txPendingPeak = txPending;
  }
}

static void recordLoopGap() {
  uint32_t nowUs = micros();
  if (g_lastLoopUs != 0) {
    uint16_t gapUs = clampElapsedUs(nowUs - g_lastLoopUs);
    g_loopGapLastUs = gapUs;
    if (gapUs > g_loopGapPeakUs) {
      g_loopGapPeakUs = gapUs;
    }
  }
  g_lastLoopUs = nowUs;
}

/**
 * @brief Timer1 overflow ISR — short round-robin DC apply slot.
 *
 * One motor is serviced per 800 Hz tick, giving each DC motor a 200 Hz apply
 * cadence while keeping the per-slice ISR body comfortably inside the UART
 * safety budget. This uses a fixed one-round pipeline:
 * - slot 0 of round N publishes outputs computed for round N
 * - loop() computes outputs for round N+1 during round N
 */
ISR(TIMER1_OVF_vect) {
  static uint8_t motorSlot = 0;
  static uint32_t roundStartUs = 0;

  uint16_t t0 = TCNT1;
  bool running = (SystemManager::getState() == SYS_STATE_RUNNING);
  g_motorCurrentSlot = motorSlot;

  if (motorSlot == 0U) {
    if (running) {
      g_motorRoundCount++;
      uint32_t currentRound = g_motorRoundCount;
      roundStartUs = micros();
      if (g_motorOutputsReady && g_motorComputedRound == currentRound) {
        for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
          dcMotors[i].publishStagedOutputISR();
        }
        g_motorAppliedSeq = g_motorPreparedSeq;
        g_motorAppliedRound = currentRound;
        g_motorOutputsReady = false;
      } else {
        g_motorReusedOutputCount++;
      }
      if (g_motorRoundReady) {
        g_motorMissedRoundCount++;
      }
      g_motorRequestedRound = currentRound + 1U;
      g_motorRoundReady = true;
    } else {
      g_motorOutputsReady = false;
      g_motorRoundReady = false;
      g_motorRequestedRound = g_motorRoundCount;
      g_motorComputedRound = g_motorRoundCount;
      g_motorAppliedRound = g_motorRoundCount;
      g_motorAppliedSeq = g_motorPreparedSeq;
      roundStartUs = micros();
    }
  }

  dcMotors[motorSlot].latchFeedbackISR();
  dcMotors[motorSlot].update();

  if (motorSlot == (NUM_DC_MOTORS - 1U)) {
    if (running) {
      LoopMonitor::recordPidRoundSpan(clampElapsedUs(micros() - roundStartUs), true);
    } else {
      LoopMonitor::recordPidRoundSpan(0, false);
    }
  }

  motorSlot = (uint8_t)((motorSlot + 1U) % NUM_DC_MOTORS);
  g_motorCurrentSlot = motorSlot;
  LoopMonitor::record(SLOT_PID_ISR, timerTicksToUs((uint16_t)(TCNT1 - t0)));
}

/**
 * @brief Timer3 overflow ISR — stepper pulse generation only.
 *
 * The actual per-stepper work lives in StepperManager::timerISR(). Keeping the
 * vector in main firmware makes the active hard-RT paths explicit.
 */
ISR(TIMER3_OVF_vect) {
  static bool prevDor2 = false;
  static bool prevFe2 = false;

  uint16_t t0 = TCNT3;
  uint8_t ucsr2a = UCSR2A;
  bool curDor2 = (ucsr2a & _BV(DOR2)) != 0;
  bool curFe2 = (ucsr2a & _BV(FE2)) != 0;
  if (curDor2 && !prevDor2) {
    g_uartDor2Count++;
  }
  if (curFe2 && !prevFe2) {
    g_uartFe2Count++;
  }
  prevDor2 = curDor2;
  prevFe2 = curFe2;

  StepperManager::timerISR();
  LoopMonitor::record(SLOT_STEPPER_ISR, timerTicksToUs((uint16_t)(TCNT3 - t0)));
}

// ============================================================================
// SOFT TASK — taskUART (50 Hz, millis-based)
// ============================================================================

/**
 * @brief UART RX/TX task — runs in loop() so USART2_RX_vect stays enabled.
 *
 * Do NOT move into TIMER1_OVF_vect (see ISR comment above for explanation).
 * Registered at prior0 (highest) so it preempts all other soft tasks.
 */ 
void taskUART() {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_TASK, HIGH);
#endif

  uint32_t t0 = micros();

#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_RX, HIGH);
#endif
  MessageCenter::processIncoming();
  MessageCenter::processDeferred();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_RX, LOW);
  digitalWrite(DEBUG_PIN_UART_TX, HIGH);
#endif
  MessageCenter::sendTelemetry();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_UART_TX, LOW);
#endif

  uint32_t elapsed = micros() - t0;
  MessageCenter::recordLoopTime(elapsed);
  LoopMonitor::record(SLOT_UART_TASK, clampElapsedUs(elapsed));

#if DEBUG_PINS_ENABLED
  // A9 (UART_LATE): pulse when wall-clock > 20 ms. This fires due to ISR
  // preemption inflation, NOT actual UART slowness — the PID loop is unaffected.
  if (elapsed > UART_TASK_BUDGET_US) {
    digitalWrite(DEBUG_PIN_UART_LATE, HIGH);
    digitalWrite(DEBUG_PIN_UART_LATE, LOW);
  }
  digitalWrite(DEBUG_PIN_UART_TASK, LOW);
#endif
}

// ============================================================================
// SOFT TASK — taskSafety (100 Hz, millis-based)
// ============================================================================

/**
 * @brief Button polling + safety checks outside ISR context.
 *
 * This keeps the common Timer1 interrupt window well below the UART overrun
 * margin while preserving a 100 Hz safety cadence, which is ample versus the
 * 500 ms heartbeat timeout and battery fault response needs.
 */
void taskSafety() {
  UserIO::readButtons();
  SafetyManager::check();
}

// ============================================================================
// SOFT TASK — taskMotors (round-driven, loop-owned)
// ============================================================================

/**
 * @brief Run the full software DC compute round when a new ISR round completes.
 *
 * The compute round is triggered from slot 0 of the current round and prepares
 * the outputs that will be published at slot 0 of the next round.
 */
void taskMotors() {
  if (SystemManager::getState() != SYS_STATE_RUNNING) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
      g_motorComputeBusy = false;
      g_motorRoundReady = false;
      g_motorOutputsReady = false;
      g_motorRequestedRound = g_motorRoundCount;
      g_motorComputedRound = g_motorRoundCount;
    }
    return;
  }

  uint32_t requestedRound = 0;
  uint8_t slotSnapshot = 0;
  bool shouldRun = false;
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (g_motorRoundReady && !g_motorComputeBusy) {
      requestedRound = g_motorRequestedRound;
      slotSnapshot = g_motorCurrentSlot;
      g_motorRoundReady = false;
      g_motorComputeBusy = true;
      shouldRun = true;
    }
  }
  if (!shouldRun) {
    return;
  }

  // In the fixed one-round pipeline, compute for round N+1 is requested at
  // slot 0 of round N. Starting in slot 1/2/3 is still within the current
  // round and usually fine. Starting when the next slot is already 0 means
  // we are in the final 1.25 ms window before publish, which is the useful
  // "late" condition for this architecture.
  if (slotSnapshot == 0U) {
    g_motorLateComputeCount++;
  }

  uint32_t t0 = micros();
  for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
    dcMotors[i].service();
  }

  uint16_t elapsedUs = clampElapsedUs(micros() - t0);
  ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
    if (g_motorRoundCount >= requestedRound) {
      g_motorCrossRoundComputeCount++;
    }
    g_motorComputeSeq++;
    g_motorPreparedSeq = g_motorComputeSeq;
    g_motorComputedRound = requestedRound;
    g_motorOutputsReady = true;
    g_motorComputeBusy = false;
  }

  LoopMonitor::record(SLOT_MOTOR_TASK, elapsedUs);
}

// ============================================================================
// SOFT TASK — taskSensors (100 Hz, millis-based)
// ============================================================================

/**
 * @brief Sensor dispatch task — runs in loop() so I2C/ADC never execute in ISR context.
 *
 * This is the main bring-up change relative to the older architecture: Timer4
 * still provides motor PWM, but its overflow interrupt is disabled.
 */
void taskSensors() {
  uint32_t t0 = micros();
  SensorManager::tick();
  LoopMonitor::record(SLOT_SENSOR_ISR, clampElapsedUs(micros() - t0));
}

// ============================================================================
// SOFT TASK — taskUserIO (20 Hz, millis-based)
// ============================================================================

/**
 * @brief User I/O updates (20 Hz, soft scheduler)
 *
 * Runs LED animations (blink, breathe, RED battery warnings).
 * Drives NeoPixel state animations via UserIO::updateNeoPixelAnimation()
 * (INIT=yellow, IDLE=breathing emerald, RUNNING=rainbow, ERROR=flashing red).
 * Button reads are handled in taskSafety() at 100 Hz.
 */
void taskUserIO() {
  uint32_t t0 = micros();
  // LED animations + limit switches + NeoPixel state animation
  UserIO::update();

  // RED LED battery warnings (independent of NeoPixel system-state animation)
  if (SensorManager::isBatteryOvervoltage()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 250);   // Fast blink — wrong battery/charger
  } else if (SensorManager::isBatteryCritical()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 250);   // Fast blink — shutdown imminent
  } else if (SensorManager::isBatteryLow()) {
    UserIO::setLED(LED_RED, LED_BLINK, 255, 1000);  // Slow blink — low warning
  }
  LoopMonitor::record(SLOT_USERIO, clampElapsedUs(micros() - t0));
}


void systemInfo() {
  static uint32_t prevMissedRoundCount = 0;
  static uint32_t prevLateComputeCount = 0;
  static uint32_t prevReusedOutputCount = 0;
  static uint32_t prevCrossRoundComputeCount = 0;
  StatusSnapshot snapshot = {};
  snapshot.state = SystemManager::getState();
  uint8_t liveFlags = computeLiveErrorFlags(snapshot.state);
  uint8_t faultMask = LoopMonitor::getFaultMask();
  snapshot.vbatMv = (uint16_t)(SensorManager::getBatteryVoltage() * 1000.0f);
  snapshot.v5Mv = (uint16_t)(SensorManager::get5VRailVoltage() * 1000.0f);
  snapshot.vservoMv = (uint16_t)(SensorManager::getServoVoltage() * 1000.0f);
  snapshot.linkState = (snapshot.state == SystemState::SYS_STATE_RUNNING)
                         ? (MessageCenter::isHeartbeatValid() ? "OK" : "LOST")
                         : "IDLE";
  snapshot.healthState = systemHealthName(snapshot.state, liveFlags);
  snapshot.servoCtrlState = ServoController::isInitialized() ? "READY" : "OFFLINE";
  snapshot.servoOutputState = ServoController::isEnabled() ? "ENABLED" : "DISABLED";
  snapshot.servoI2CState = ServoController::hasI2CError() ? "ERROR" : "OK";
  snapshot.batteryState = batteryStateName();
  snapshot.servoRailState = servoRailStateName();
  formatErrorFlags(liveFlags, snapshot.liveBuf, sizeof(snapshot.liveBuf));
  formatErrorFlags(MessageCenter::getFaultLatchFlags(), snapshot.latchedBuf, sizeof(snapshot.latchedBuf));
  formatLoopFaults(faultMask, snapshot.loopBuf, sizeof(snapshot.loopBuf));
  snapshotUart2FaultCounts(snapshot.dor2Count, snapshot.fe2Count);
  MessageCenter::snapshotTrafficWindow(snapshot.rxBytesWindow,
                                       snapshot.rxFramesWindow,
                                       snapshot.rxTlvsWindow,
                                       snapshot.rxHeartbeatsWindow,
                                       snapshot.txBytesWindow,
                                       snapshot.txFramesWindow);
  snapshotMotorControlSync(snapshot.motorRoundCount,
                           snapshot.motorRequestedRound,
                           snapshot.motorComputedRound,
                           snapshot.motorAppliedRound,
                           snapshot.motorSlot,
                           snapshot.motorComputeSeq,
                           snapshot.motorAppliedSeq,
                           snapshot.missedRoundDelta,
                           snapshot.lateComputeDelta,
                           snapshot.reusedOutputDelta,
                           snapshot.crossRoundDelta,
                           snapshot.motorComputeBusy);
  snapshot.missedRoundDelta -= prevMissedRoundCount;
  snapshot.lateComputeDelta -= prevLateComputeCount;
  snapshot.reusedOutputDelta -= prevReusedOutputCount;
  snapshot.crossRoundDelta -= prevCrossRoundComputeCount;
  prevMissedRoundCount += snapshot.missedRoundDelta;
  prevLateComputeCount += snapshot.lateComputeDelta;
  prevReusedOutputCount += snapshot.reusedOutputDelta;
  prevCrossRoundComputeCount += snapshot.crossRoundDelta;
  snapshot.heartbeatAgeMs = MessageCenter::getTimeSinceHeartbeat();
  snapshot.heartbeatTimeoutMs = MessageCenter::getHeartbeatTimeoutMs();
  snapshot.lastByteAgeMs = MessageCenter::getTimeSinceRxByte();
  snapshot.servoEnabledMask = MessageCenter::getServoEnabledMask();
  snapshot.freeRam = MessageCenter::getFreeRAM();
  snapshot.txDroppedFrames = MessageCenter::getTxDroppedFrames();
  snapshot.debugDroppedBytes = DebugLog::getDroppedBytes();
  snapshot.loopGapLastUs = g_loopGapLastUs;
  snapshot.loopGapPeakUs = g_loopGapPeakUs;
  snapshot.pidAvgUs = LoopMonitor::getAvgUs(SLOT_PID_ISR);
  snapshot.pidPeakUs = LoopMonitor::getPeakUs(SLOT_PID_ISR);
  snapshot.pidMaxUs = LoopMonitor::getMaxUs(SLOT_PID_ISR);
  snapshot.pidBudgetUs = LoopMonitor::getBudgetUs(SLOT_PID_ISR);
  snapshot.pidRoundAvgUs = LoopMonitor::getPidRoundAvgUs();
  snapshot.pidRoundPeakUs = LoopMonitor::getPidRoundPeakUs();
  snapshot.pidRoundMaxUs = LoopMonitor::getPidRoundMaxUs();
  snapshot.pidRoundBudgetUs = LoopMonitor::getPidRoundBudgetUs();
  snapshot.stepAvgUs = LoopMonitor::getAvgUs(SLOT_STEPPER_ISR);
  snapshot.stepPeakUs = LoopMonitor::getPeakUs(SLOT_STEPPER_ISR);
  snapshot.stepMaxUs = LoopMonitor::getMaxUs(SLOT_STEPPER_ISR);
  snapshot.stepBudgetUs = LoopMonitor::getBudgetUs(SLOT_STEPPER_ISR);
  snapshot.motorAvgUs = LoopMonitor::getAvgUs(SLOT_MOTOR_TASK);
  snapshot.motorPeakUs = LoopMonitor::getPeakUs(SLOT_MOTOR_TASK);
  snapshot.motorMaxUs = LoopMonitor::getMaxUs(SLOT_MOTOR_TASK);
  snapshot.motorBudgetUs = LoopMonitor::getBudgetUs(SLOT_MOTOR_TASK);
  snapshot.sensorAvgUs = LoopMonitor::getAvgUs(SLOT_SENSOR_ISR);
  snapshot.sensorPeakUs = LoopMonitor::getPeakUs(SLOT_SENSOR_ISR);
  snapshot.sensorMaxUs = LoopMonitor::getMaxUs(SLOT_SENSOR_ISR);
  snapshot.sensorBudgetUs = LoopMonitor::getBudgetUs(SLOT_SENSOR_ISR);
  snapshot.uartAvgUs = LoopMonitor::getAvgUs(SLOT_UART_TASK);
  snapshot.uartPeakUs = LoopMonitor::getPeakUs(SLOT_UART_TASK);
  snapshot.uartMaxUs = LoopMonitor::getMaxUs(SLOT_UART_TASK);
  snapshot.uartBudgetUs = LoopMonitor::getBudgetUs(SLOT_UART_TASK);
  snapshot.ioAvgUs = LoopMonitor::getAvgUs(SLOT_USERIO);
  snapshot.ioPeakUs = LoopMonitor::getPeakUs(SLOT_USERIO);
  snapshot.ioMaxUs = LoopMonitor::getMaxUs(SLOT_USERIO);
  snapshot.ioBudgetUs = LoopMonitor::getBudgetUs(SLOT_USERIO);
  snapshot.debugAvgUs = g_statusTaskAvgUs;
  snapshot.debugPeakUs = g_statusTaskPeakUs;
  snapshot.debugMaxUs = g_statusTaskMaxUs;
  snapshot.flushAvgUs = g_flushAvgUs;
  snapshot.flushPeakUs = g_flushPeakUs;
  snapshot.flushMaxUs = g_flushMaxUs;
  snapshot.uartRxErrors = MessageCenter::getUartRxErrors();
  snapshot.crcErrorCount = MessageCenter::getCrcErrorCount();
  snapshot.frameLenErrorCount = MessageCenter::getFrameLenErrorCount();
  snapshot.tlvErrorCount = MessageCenter::getTlvErrorCount();
  snapshot.oversizeErrorCount = MessageCenter::getOversizeErrorCount();
  snapshot.rxAvailable = (uint16_t)RPI_SERIAL.available();
  snapshot.rxPeak = g_rxBacklogPeak;
  snapshot.txPending = MessageCenter::getTxPendingBytes();
  snapshot.txPeak = g_txPendingPeak;

  g_statusSnapshot = snapshot;
  g_statusSnapshotPending = true;
  g_statusSnapshotChunk = 0;

  LoopMonitor::clearPeaks();
  g_loopGapPeakUs = g_loopGapLastUs;
  g_rxBacklogPeak = snapshot.rxAvailable;
  g_txPendingPeak = snapshot.txPending;
  g_statusTaskPeakUs = 0;
  g_flushPeakUs = 0;
}

static void emitSystemInfoChunk() {
  if (!g_statusSnapshotPending) {
    return;
  }
  if (DebugLog::getQueuedBytes() > (DEBUG_LOG_BUFFER_SIZE / 2U)) {
    return;
  }

  uint32_t t0 = micros();
  const StatusSnapshot &s = g_statusSnapshot;

  switch (g_statusSnapshotChunk) {
    case 0:
      DebugLog::printf_P(PSTR("\n[SYSTEM]\n"));
      break;
    case 1:
      DebugLog::printf_P(PSTR("State: %s [%s]\n"), stateName(s.state), s.healthState);
      break;
    case 2:
      DebugLog::printf_P(PSTR("Link: %s | Heartbeat: %lums/%ums | Last Byte: %lums\n"),
        s.linkState, (unsigned long)s.heartbeatAgeMs, s.heartbeatTimeoutMs, (unsigned long)s.lastByteAgeMs);
      break;
    case 3:
      DebugLog::printf_P(PSTR("Battery: %u.%03uV [%s] | Servo: %u.%03uV [%s] | 5V: %u.%03uV\n"),
        (unsigned)(s.vbatMv / 1000U), (unsigned)(s.vbatMv % 1000U), s.batteryState,
        (unsigned)(s.vservoMv / 1000U), (unsigned)(s.vservoMv % 1000U), s.servoRailState,
        (unsigned)(s.v5Mv / 1000U), (unsigned)(s.v5Mv % 1000U));
      break;
    case 4:
      DebugLog::printf_P(PSTR("ServoCtrl: %s [%s] | EnabledMask: 0x%04X | I2C: %s\n"),
        s.servoCtrlState, s.servoOutputState, s.servoEnabledMask, s.servoI2CState);
      break;
    case 5:
      DebugLog::printf_P(PSTR("Faults: live=%s | latched=%s | overrun=%s\n"),
        s.liveBuf, s.latchedBuf, s.loopBuf);
      break;
    case 6:
      DebugLog::printf_P(PSTR("Control: round=%lu | req/cmp/app=%lu/%lu/%lu | slot=%u | prep/app=%u/%u | pipe=%u | busy=%s\n"),
        (unsigned long)s.motorRoundCount,
        (unsigned long)s.motorRequestedRound,
        (unsigned long)s.motorComputedRound,
        (unsigned long)s.motorAppliedRound,
        s.motorSlot,
        s.motorComputeSeq,
        s.motorAppliedSeq,
        (uint8_t)(s.motorComputeSeq - s.motorAppliedSeq),
        s.motorComputeBusy ? "yes" : "no");
      break;
    case 7:
      DebugLog::printf_P(PSTR("ControlWin: missed=%lu | late=%lu | reused=%lu | cross=%lu\n"),
        (unsigned long)s.missedRoundDelta,
        (unsigned long)s.lateComputeDelta,
        (unsigned long)s.reusedOutputDelta,
        (unsigned long)s.crossRoundDelta);
      break;
    case 8:
      DebugLog::printf_P(PSTR("Memory: RAM=%u B | TxDrop=%lu | DbgDrop=%u | LoopGap=%u/%uus\n"),
        s.freeRam, (unsigned long)s.txDroppedFrames, s.debugDroppedBytes, s.loopGapLastUs, s.loopGapPeakUs);
      break;
    case 9:
      DebugLog::printf_P(PSTR("\n[TIMING] (avg/peak/max)\n"));
      break;
    case 10:
      DebugLog::printf_P(PSTR("pid %u/%u/%u (%u) us | pidr %u/%u/%u (%u) us | step %u/%u/%u (%u) us\n"),
        s.pidAvgUs, s.pidPeakUs, s.pidMaxUs, s.pidBudgetUs,
        s.pidRoundAvgUs, s.pidRoundPeakUs, s.pidRoundMaxUs, s.pidRoundBudgetUs,
        s.stepAvgUs, s.stepPeakUs, s.stepMaxUs, s.stepBudgetUs);
      break;
    case 11:
      DebugLog::printf_P(PSTR("motor %u/%u/%u (%u) us | sensor %u/%u/%u (%u) us\n"),
        s.motorAvgUs, s.motorPeakUs, s.motorMaxUs, s.motorBudgetUs,
        s.sensorAvgUs, s.sensorPeakUs, s.sensorMaxUs, s.sensorBudgetUs);
      break;
    case 12:
      DebugLog::printf_P(PSTR("uart %u/%u/%u (%u) us | io %u/%u/%u (%u) us\n"),
        s.uartAvgUs, s.uartPeakUs, s.uartMaxUs, s.uartBudgetUs,
        s.ioAvgUs, s.ioPeakUs, s.ioMaxUs, s.ioBudgetUs);
      break;
    case 13:
      DebugLog::printf_P(PSTR("debug %u/%u/%u us | flush %u/%u/%u us\n"),
        s.debugAvgUs, s.debugPeakUs, s.debugMaxUs,
        s.flushAvgUs, s.flushPeakUs, s.flushMaxUs);
      break;
    case 14:
      DebugLog::printf_P(PSTR("\n[UART]\n"));
      break;
    case 15:
      DebugLog::printf_P(PSTR("activity rxB/frame/tlv/hb=%u/%u/%u/%u txB/frame=%u/%u\n"),
        s.rxBytesWindow, s.rxFramesWindow, s.rxTlvsWindow, s.rxHeartbeatsWindow,
        s.txBytesWindow, s.txFramesWindow);
      break;
    default:
      DebugLog::printf_P(PSTR("total_err=%u dor=%lu fe=%lu crc=%u frame=%u tlv=%u oversize=%u rxQ(available/peak)=%u/%u txQ(pending/peak)=%u/%u\n"),
        s.uartRxErrors,
        (unsigned long)s.dor2Count,
        (unsigned long)s.fe2Count,
        s.crcErrorCount,
        s.frameLenErrorCount,
        s.tlvErrorCount,
        s.oversizeErrorCount,
        s.rxAvailable,
        s.rxPeak,
        s.txPending,
        s.txPeak);
      g_statusSnapshotPending = false;
      g_statusSnapshotChunk = 0;
      recordAuxTiming(clampElapsedUs(micros() - t0), g_statusTaskAvgUs, g_statusTaskPeakUs, g_statusTaskMaxUs);
      return;
  }

  g_statusSnapshotChunk++;
  recordAuxTiming(clampElapsedUs(micros() - t0), g_statusTaskAvgUs, g_statusTaskPeakUs, g_statusTaskMaxUs);
}

// ============================================================================
// ENCODER ISR TRAMPOLINES
// ============================================================================

/**
 * @brief Encoder ISR wrappers
 *
 * These are minimal ISR wrappers that forward calls to encoder objects.
 * Encoder ISRs must be global functions (not class methods) to use with
 * attachInterrupt().
 */

void encoderISR_M1_A() {
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, HIGH);
#endif
  encoder1.onInterruptA();
#if DEBUG_PINS_ENABLED
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
#endif
}

void encoderISR_M1_B() {
  encoder1.onInterruptB();
}

void encoderISR_M2_A() {
  encoder2.onInterruptA();
}

void encoderISR_M2_B() {
  encoder2.onInterruptB();
}

// M3/M4 encoder pins (A14, A15, 11, 12) are on PCINT buses, not hardware
// INT pins. attachInterrupt() does not work for them on Mega, so the wrappers
// below are dispatched from the PCINT2/PCINT0 vectors.
void encoderISR_M3() {
  encoder3.onInterruptA();
}

void encoderISR_M4() {
  encoder4.onInterruptA();
}

ISR(PCINT2_vect) {
  encoderISR_M3();
}

ISR(PCINT0_vect) {
  encoderISR_M4();
}

// ============================================================================
// ARDUINO SETUP
// ============================================================================

void setup() {
  // ------------------------------------------------------------------------
  // Initialize SystemManager (sets INIT state before any module starts)
  // ------------------------------------------------------------------------
  SystemManager::init();

  // ------------------------------------------------------------------------
  // Initialize Debug Serial (USB)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
  while (!DEBUG_SERIAL && millis() < 2000) {
    ; // Wait for serial port to connect (up to 2 seconds)
  }
  DebugLog::init();
  LoopMonitor::init();

  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println(F("  MAE 162 Robot Firmware v0.8.0"));
  DEBUG_SERIAL.println(F("========================================"));
  DEBUG_SERIAL.println();

  // ------------------------------------------------------------------------
  // Initialize Soft Scheduler (millis-based)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing soft scheduler..."));
  Scheduler::init();

#if DEBUG_PINS_ENABLED
  // Configure debug pins for oscilloscope timing measurement
  pinMode(DEBUG_PIN_ENCODER_ISR, OUTPUT);  // A7
  pinMode(DEBUG_PIN_STEPPER_ISR, OUTPUT);  // A8
  pinMode(DEBUG_PIN_UART_LATE,   OUTPUT);  // A9
  pinMode(DEBUG_PIN_PID_LOOP,    OUTPUT);  // A10
  pinMode(DEBUG_PIN_UART_TASK,   OUTPUT);  // A11
  pinMode(DEBUG_PIN_UART_RX,     OUTPUT);  // A12
  pinMode(DEBUG_PIN_UART_TX,     OUTPUT);  // A13
  digitalWrite(DEBUG_PIN_ENCODER_ISR, LOW);
  digitalWrite(DEBUG_PIN_STEPPER_ISR, LOW);
  digitalWrite(DEBUG_PIN_UART_LATE,   LOW);
  digitalWrite(DEBUG_PIN_PID_LOOP,    LOW);
  digitalWrite(DEBUG_PIN_UART_TASK,   LOW);
  digitalWrite(DEBUG_PIN_UART_RX,     LOW);
  digitalWrite(DEBUG_PIN_UART_TX,     LOW);
  DEBUG_SERIAL.println(F("[Setup] Debug pins configured (A7-A13)"));
#endif

  // ------------------------------------------------------------------------
  // Initialize Communication (UART + TLV Protocol)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing UART communication..."));
  MessageCenter::init();

  // ------------------------------------------------------------------------
  // Initialize Sensors (I2C, ADC)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing sensors..."));
  SensorManager::init();

  // ------------------------------------------------------------------------
  // Initialize User I/O (LEDs, Buttons, NeoPixels)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing user I/O..."));
  UserIO::init();

  // ------------------------------------------------------------------------
  // Initialize Servo Controller (PCA9685)
  // ------------------------------------------------------------------------
#if SERVO_CONTROLLER_ENABLED
  DEBUG_SERIAL.println(F("[Setup] Initializing servo controller..."));
  ServoController::init();
  DEBUG_SERIAL.println(F("  - PCA9685 initialized (50Hz PWM)"));
#endif

  // ------------------------------------------------------------------------
  // Initialize Stepper Motors (Timer3 @ 10kHz)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing stepper motors..."));
  StepperManager::init();

  // ------------------------------------------------------------------------
  // Initialize DC Motors and Encoders
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Initializing DC motors and encoders..."));

  // Calculate counts per revolution based on encoder mode
  uint16_t countsPerRev = ENCODER_PPR * encoder1.getResolutionMultiplier();
  DEBUG_SERIAL.print(F("  - Encoder resolution: "));
  DEBUG_SERIAL.print(countsPerRev);
  DEBUG_SERIAL.println(F(" counts/rev"));

  // Motor 1
  encoder1.init(PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED);
  velocityEst1.init(countsPerRev);
  velocityEst1.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst1.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[0].init(0, &encoder1, &velocityEst1, DC_MOTOR_1_DIR_INVERTED);
  dcMotors[0].setPins(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2);
  dcMotors[0].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[0].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 1 initialized"));

  // Motor 2
  encoder2.init(PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED);
  velocityEst2.init(countsPerRev);
  velocityEst2.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst2.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[1].init(1, &encoder2, &velocityEst2, DC_MOTOR_2_DIR_INVERTED);
  dcMotors[1].setPins(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2);
  dcMotors[1].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[1].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 2 initialized"));

  // Motor 3
  encoder3.init(PIN_M3_ENC_A, PIN_M3_ENC_B, ENCODER_3_DIR_INVERTED);
  velocityEst3.init(countsPerRev);
  velocityEst3.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst3.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[2].init(2, &encoder3, &velocityEst3, DC_MOTOR_3_DIR_INVERTED);
  dcMotors[2].setPins(PIN_M3_EN, PIN_M3_IN1, PIN_M3_IN2);
  dcMotors[2].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[2].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 3 initialized"));

  // Motor 4
  encoder4.init(PIN_M4_ENC_A, PIN_M4_ENC_B, ENCODER_4_DIR_INVERTED);
  velocityEst4.init(countsPerRev);
  velocityEst4.setFilterSize(VELOCITY_FILTER_SIZE);
  velocityEst4.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);
  dcMotors[3].init(3, &encoder4, &velocityEst4, DC_MOTOR_4_DIR_INVERTED);
  dcMotors[3].setPins(PIN_M4_EN, PIN_M4_IN1, PIN_M4_IN2);
  dcMotors[3].setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
  dcMotors[3].setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);
  DEBUG_SERIAL.println(F("  - Motor 4 initialized"));

  // ------------------------------------------------------------------------
  // Attach Encoder Interrupts
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Attaching encoder interrupts..."));

  // M1: pins 2 (INT0) and 3 (INT1) — both on hardware INT, full 4x
  attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_B), encoderISR_M1_B, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 1 encoder ISRs attached (4x: A+B)"));

  // M2: pins 18 (INT5) and 19 (INT4) — both on hardware INT, full 4x
  attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2_A, CHANGE);
  attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_B), encoderISR_M2_B, CHANGE);
  DEBUG_SERIAL.println(F("  - Motor 2 encoder ISRs attached (4x: A+B)"));

  // M3: PCINT2 group (Port K) — A14=PCINT22, A15=PCINT23
  PCIFR |= _BV(PCIF2);
  PCMSK2 |= _BV(PCINT22);
#if ENCODER_3_MODE == ENCODER_4X
  PCMSK2 |= _BV(PCINT23);
#endif
  PCICR |= _BV(PCIE2);

  // M4: PCINT0 group (Port B) — pin11=PCINT5, pin12=PCINT6
  PCIFR |= _BV(PCIF0);
  PCMSK0 |= _BV(PCINT5);
#if ENCODER_4_MODE == ENCODER_4X
  PCMSK0 |= _BV(PCINT6);
#endif
  PCICR |= _BV(PCIE0);
  DEBUG_SERIAL.println(F("  - Motor 3/4 encoder ISRs attached (PCINT)"));

  // ------------------------------------------------------------------------
  // Register Soft Scheduler Tasks
  // Timer1 now keeps only the DC slot hard real-time. UART, safety, and sensor
  // work run in loop() so USART2 RX can preempt them safely.
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Registering soft scheduler tasks..."));

  // UART comms: priority 0 (highest) — must preempt all other soft tasks
  int8_t taskId = Scheduler::registerTask(taskUART, 1000 / UART_COMMS_FREQ_HZ, 0);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - UART: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / UART_COMMS_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (50Hz)"));
  }

  taskId = Scheduler::registerTask(taskSafety, 1000 / SENSOR_UPDATE_FREQ_HZ, 1);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Safety: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (100Hz)"));
  }

  DEBUG_SERIAL.println(F("  - Motors: round-driven in loop() from Timer1 slot-3 flag"));

  taskId = Scheduler::registerTask(taskSensors, 1000 / SENSOR_UPDATE_FREQ_HZ, 3);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - Sensors: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / SENSOR_UPDATE_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms (100Hz)"));
  }

  taskId = Scheduler::registerTask(taskUserIO, 1000 / USER_IO_FREQ_HZ, 4);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - User I/O: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / USER_IO_FREQ_HZ);
    DEBUG_SERIAL.println(F("ms"));
  }

  taskId = Scheduler::registerTask(systemInfo, 1000 / DEBUG_STATUS_STREAM_HZ, 5);
  if (taskId >= 0) {
    DEBUG_SERIAL.print(F("  - System Info: Task #"));
    DEBUG_SERIAL.print(taskId);
    DEBUG_SERIAL.print(F(" @ "));
    DEBUG_SERIAL.print(1000 / DEBUG_STATUS_STREAM_HZ);
    DEBUG_SERIAL.println(F("ms"));
  }
  // ------------------------------------------------------------------------
  // Transition to IDLE (all hardware initialized, ready to receive commands)
  // ------------------------------------------------------------------------
  SystemManager::requestTransition(SYS_STATE_IDLE);
  DEBUG_SERIAL.println(F("[Setup] System state → IDLE"));

  // ------------------------------------------------------------------------
  // Start Hard Real-Time ISRs (MUST be last — ISRs fire immediately)
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println(F("[Setup] Starting hard real-time ISRs (Timer1 + Timer3; Timer4 PWM only)..."));
  ISRScheduler::init();

  // ------------------------------------------------------------------------
  // Setup Complete
  // ------------------------------------------------------------------------
  DEBUG_SERIAL.println();
  DEBUG_SERIAL.println(F("[Setup] Initialization complete!"));
  DEBUG_SERIAL.println(F("  Hard RT: DC slot@800Hz + Stepper@10kHz"));
  DEBUG_SERIAL.print(F("  Soft RT: UART@50Hz, Safety@100Hz, MotorCompute@200Hz(round-flag), Sensors@100Hz, UserIO@20Hz, Debug@"));
  DEBUG_SERIAL.print(DEBUG_STATUS_STREAM_HZ);
  DEBUG_SERIAL.println(F("Hz"));
  DEBUG_SERIAL.println(F("[Setup] Entered main loop."));
  DEBUG_SERIAL.println();
}

// ============================================================================
// ARDUINO MAIN LOOP
// ============================================================================

void loop() {
  recordLoopGap();
  updateDebugWindowPeaks();
  MessageCenter::drainUart();
  MessageCenter::drainTx();
  taskMotors();
  emitSystemInfoChunk();
  uint32_t flushStartUs = micros();
  DebugLog::flush();
  recordAuxTiming(clampElapsedUs(micros() - flushStartUs),
                  g_flushAvgUs,
                  g_flushPeakUs,
                  g_flushMaxUs);

  // Execute highest-priority ready task
  Scheduler::tick();
  taskMotors();
  emitSystemInfoChunk();
  MessageCenter::drainTx();
  flushStartUs = micros();
  DebugLog::flush();
  recordAuxTiming(clampElapsedUs(micros() - flushStartUs),
                  g_flushAvgUs,
                  g_flushPeakUs,
                  g_flushMaxUs);
}
