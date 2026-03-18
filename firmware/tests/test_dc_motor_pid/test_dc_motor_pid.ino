/**
 * @file test_dc_motor_pid.ino
 * @brief Test sketch for DC motor PID control (Phase 3)
 *
 * This test verifies complete closed-loop motor control by:
 * 1. Initializing DCMotor instances with encoders and velocity estimators
 * 2. Testing velocity control mode
 * 3. Testing position control mode
 * 4. Verifying PID tracking performance
 *
 * Expected Behavior:
 * - Motor velocity tracks commanded setpoint
 * - Motor position reaches target with minimal overshoot
 * - PID loop runs at 200Hz (5ms period)
 * - Velocity estimation updates smoothly
 *
 * How to Test:
 * 1. Upload this sketch
 * 2. Open Serial Monitor @ 115200 baud
 * 3. Motors will test velocity control, then position control
 * 4. Observe setpoint tracking and PID output
 * 5. Tune PID gains if needed via serial commands
 *
 * Hardware Setup:
 * - Connect motors with encoders
 * - Connect motor power supply
 * - Ensure motor shafts are free to spin
 * - No mechanical load initially (tune PID first)
 *
 * Safety:
 * - PID limits PWM output to ±255
 * - Position control limits velocity setpoint
 * - Watch for integral windup on tracking errors
 *
 * Verification:
 * 1. Compile success
 * 2. Velocity control tracks setpoint within ±10%
 * 3. Position control reaches target within ±5 ticks
 * 4. No sustained oscillations
 * 5. Steady-state error < 5% for velocity
 */

#include "src/config.h"
#include "src/pins.h"
#include "src/Scheduler.h"
#include "src/ISRScheduler.h"
#include "src/modules/EncoderCounter.h"
#include "src/modules/LoopMonitor.h"
#include "src/modules/VelocityEstimator.h"
#include "src/drivers/DCMotor.h"
#include <util/atomic.h>

// ============================================================================
// ENCODER AND VELOCITY ESTIMATOR INSTANCES
// ============================================================================

// Motor 1
#if ENCODER_1_MODE == ENCODER_2X
EncoderCounter2x encoder1;
#else
EncoderCounter4x encoder1;
#endif
EdgeTimeVelocityEstimator velocityEst1;

// Motor 2
#if ENCODER_2_MODE == ENCODER_2X
EncoderCounter2x encoder2;
#else
EncoderCounter4x encoder2;
#endif
EdgeTimeVelocityEstimator velocityEst2;

// Motor 3
#if ENCODER_3_MODE == ENCODER_2X
EncoderCounter2x encoder3;
#else
EncoderCounter4x encoder3;
#endif
EdgeTimeVelocityEstimator velocityEst3;

// Motor 4
#if ENCODER_4_MODE == ENCODER_2X
EncoderCounter2x encoder4;
#else
EncoderCounter4x encoder4;
#endif
EdgeTimeVelocityEstimator velocityEst4;

// ============================================================================
// DC MOTOR INSTANCES
// ============================================================================

DCMotor motor1;
DCMotor motor2;
DCMotor motor3;
DCMotor motor4;

DCMotor* const g_motors[NUM_DC_MOTORS] = { &motor1, &motor2, &motor3, &motor4 };

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

static uint16_t clampElapsedUs(uint32_t elapsedUs) {
    return (elapsedUs > 0xFFFFUL) ? 0xFFFFU : (uint16_t)elapsedUs;
}

static uint16_t timerTicksToUs(uint16_t ticks) {
    return (uint16_t)((ticks + 1U) >> 1);
}

static bool controlActive() {
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        if (g_motors[i]->isEnabled()) {
            return true;
        }
    }
    return false;
}

static void snapshotMotorControl(uint32_t &roundCount,
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
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
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
    }
}

// ISRScheduler::init() enables TIMSK1 (Timer1 OVF) and TIMSK4 (Timer4 OVF).
//
// This test now mirrors the production mixed-control architecture:
// - TIMER1 @ 800 Hz latches one motor's encoder count and applies one cached
//   H-bridge output per tick
// - taskMotorPID() computes the next round during the current round
ISR(TIMER1_OVF_vect) {
    static uint8_t motorSlot = 0;
    static uint32_t roundStartUs = 0;

    uint16_t t0 = TCNT1;
    bool active = controlActive();
    g_motorCurrentSlot = motorSlot;

    if (motorSlot == 0U) {
        if (active) {
            g_motorRoundCount++;
            uint32_t currentRound = g_motorRoundCount;
            roundStartUs = micros();
            if (g_motorOutputsReady && g_motorComputedRound == currentRound) {
                for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
                    g_motors[i]->publishStagedOutputISR();
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

    g_motors[motorSlot]->latchFeedbackISR();
    g_motors[motorSlot]->update();

    if (motorSlot == (NUM_DC_MOTORS - 1U)) {
        if (active) {
            LoopMonitor::recordPidRoundSpan(clampElapsedUs(micros() - roundStartUs), true);
        } else {
            LoopMonitor::recordPidRoundSpan(0, false);
        }
    }

    motorSlot = (uint8_t)((motorSlot + 1U) % NUM_DC_MOTORS);
    g_motorCurrentSlot = motorSlot;
    LoopMonitor::record(SLOT_PID_ISR, timerTicksToUs((uint16_t)(TCNT1 - t0)));
}

// ============================================================================
// TEST STATE MACHINE
// ============================================================================

enum TestState {
    TEST_INIT,
    TEST_VELOCITY_POSITIVE,
    TEST_VELOCITY_NEGATIVE,
    TEST_VELOCITY_ZERO,
    TEST_POSITION_FORWARD,
    TEST_POSITION_REVERSE,
    TEST_DONE
};

TestState currentState = TEST_INIT;
uint32_t stateStartTime = 0;
const uint32_t STATE_DURATION_MS = 5000;  // 5 seconds per test

// Test setpoints
const float TEST_VELOCITY_POS = 500.0f;   // 500 ticks/sec forward
const float TEST_VELOCITY_NEG = -500.0f;  // 500 ticks/sec reverse
const int32_t TEST_POSITION_FWD = 720;    // 1 revolution forward (2x mode)
const int32_t TEST_POSITION_REV = -720;   // 1 revolution reverse

// ============================================================================
// INTERRUPT SERVICE ROUTINES
// ============================================================================

// M1 / M2 — hardware INT pins, use attachInterrupt()
void encoderISR_M1() { encoder1.onInterruptA(); }
void encoderISR_M2() { encoder2.onInterruptA(); }

// Phase B ISRs — required for 4x quadrature decoding.
#if ENCODER_1_MODE == ENCODER_4X
void encoderISR_M1_B() { encoder1.onInterruptB(); }
#endif
#if ENCODER_2_MODE == ENCODER_4X
void encoderISR_M2_B() { encoder2.onInterruptB(); }
#endif

// M3 — PCINT2 group (Port K): A14=PCINT22, A15=PCINT23
// attachInterrupt() silently fails for these pins on Mega, so we use PCINT.
// onInterruptA() calls processEdge() which reads both A and B, so one ISR
// handles both phases correctly in 4x mode.
ISR(PCINT2_vect) { encoder3.onInterruptA(); }

// M4 — PCINT0 group (Port B): pin11=PCINT5, pin12=PCINT6
ISR(PCINT0_vect) { encoder4.onInterruptA(); }

// ============================================================================
// SCHEDULER TASKS
// ============================================================================

/**
 * @brief Motor control compute task (one-round pipeline, soft)
 */
void taskMotorPID() {
    if (!controlActive()) {
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

        if (slotSnapshot == 0U) {
            g_motorLateComputeCount++;
        }

    uint32_t t0 = micros();
    motor1.service();
    motor2.service();
    motor3.service();
    motor4.service();

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

/**
 * @brief Status printing task (5Hz)
 */
void taskPrintStatus() {
    static uint32_t prevMissedRoundCount = 0;
    static uint32_t prevLateComputeCount = 0;
    static uint32_t prevReusedOutputCount = 0;
    static uint32_t prevCrossRoundComputeCount = 0;

    uint32_t roundCount = 0;
    uint32_t requestedRound = 0;
    uint32_t computedRound = 0;
    uint32_t appliedRound = 0;
    uint8_t slot = 0;
    uint8_t computeSeq = 0;
    uint8_t appliedSeq = 0;
    uint32_t missedRoundCount = 0;
    uint32_t lateComputeCount = 0;
    uint32_t reusedOutputCount = 0;
    uint32_t crossRoundComputeCount = 0;
    bool computeBusy = false;
    snapshotMotorControl(roundCount,
                         requestedRound,
                         computedRound,
                         appliedRound,
                         slot,
                         computeSeq,
                         appliedSeq,
                         missedRoundCount,
                         lateComputeCount,
                         reusedOutputCount,
                         crossRoundComputeCount,
                         computeBusy);
    uint32_t missedRoundDelta = missedRoundCount - prevMissedRoundCount;
    uint32_t lateComputeDelta = lateComputeCount - prevLateComputeCount;
    uint32_t reusedOutputDelta = reusedOutputCount - prevReusedOutputCount;
    uint32_t crossRoundDelta = crossRoundComputeCount - prevCrossRoundComputeCount;
    prevMissedRoundCount = missedRoundCount;
    prevLateComputeCount = lateComputeCount;
    prevReusedOutputCount = reusedOutputCount;
    prevCrossRoundComputeCount = crossRoundComputeCount;

    DEBUG_SERIAL.println(F("[CONTROL]"));
    DEBUG_SERIAL.print(F("round="));
    DEBUG_SERIAL.print(roundCount);
    DEBUG_SERIAL.print(F(" req/cmp/app="));
    DEBUG_SERIAL.print(requestedRound);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(computedRound);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(appliedRound);
    DEBUG_SERIAL.print(F(" slot="));
    DEBUG_SERIAL.print(slot);
    DEBUG_SERIAL.print(F(" seq="));
    DEBUG_SERIAL.print(computeSeq);
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(appliedSeq);
    DEBUG_SERIAL.print(F(" lag="));
    DEBUG_SERIAL.print((uint8_t)(computeSeq - appliedSeq));
    DEBUG_SERIAL.print(F(" busy="));
    DEBUG_SERIAL.println(computeBusy ? F("yes") : F("no"));
    DEBUG_SERIAL.print(F("window missed="));
    DEBUG_SERIAL.print(missedRoundDelta);
    DEBUG_SERIAL.print(F(" late="));
    DEBUG_SERIAL.print(lateComputeDelta);
    DEBUG_SERIAL.print(F(" reused="));
    DEBUG_SERIAL.print(reusedOutputDelta);
    DEBUG_SERIAL.print(F(" cross="));
    DEBUG_SERIAL.println(crossRoundDelta);

    DEBUG_SERIAL.print(F("M1 pos="));
    DEBUG_SERIAL.print(motor1.getPosition());
    DEBUG_SERIAL.print(F(" vel="));
    DEBUG_SERIAL.print((int)motor1.getVelocity());
    DEBUG_SERIAL.print(F(" pwm="));
    DEBUG_SERIAL.print(motor1.getPWMOutput());
    DEBUG_SERIAL.print(F(" | M2 pos="));
    DEBUG_SERIAL.print(motor2.getPosition());
    DEBUG_SERIAL.print(F(" vel="));
    DEBUG_SERIAL.print((int)motor2.getVelocity());
    DEBUG_SERIAL.print(F(" pwm="));
    DEBUG_SERIAL.println(motor2.getPWMOutput());

    DEBUG_SERIAL.print(F("M3 pos="));
    DEBUG_SERIAL.print(motor3.getPosition());
    DEBUG_SERIAL.print(F(" vel="));
    DEBUG_SERIAL.print((int)motor3.getVelocity());
    DEBUG_SERIAL.print(F(" pwm="));
    DEBUG_SERIAL.print(motor3.getPWMOutput());
    DEBUG_SERIAL.print(F(" | M4 pos="));
    DEBUG_SERIAL.print(motor4.getPosition());
    DEBUG_SERIAL.print(F(" vel="));
    DEBUG_SERIAL.print((int)motor4.getVelocity());
    DEBUG_SERIAL.print(F(" pwm="));
    DEBUG_SERIAL.println(motor4.getPWMOutput());

    DEBUG_SERIAL.println(F("[TIMING] (avg/peak/max)"));
    DEBUG_SERIAL.print(F("pid "));
    DEBUG_SERIAL.print(LoopMonitor::getAvgUs(SLOT_PID_ISR));
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(LoopMonitor::getPeakUs(SLOT_PID_ISR));
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(LoopMonitor::getMaxUs(SLOT_PID_ISR));
    DEBUG_SERIAL.print(F(" ("));
    DEBUG_SERIAL.print(LoopMonitor::getBudgetUs(SLOT_PID_ISR));
    DEBUG_SERIAL.print(F(") us | pidr "));
    DEBUG_SERIAL.print(LoopMonitor::getPidRoundAvgUs());
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(LoopMonitor::getPidRoundPeakUs());
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(LoopMonitor::getPidRoundMaxUs());
    DEBUG_SERIAL.print(F(" ("));
    DEBUG_SERIAL.print(LoopMonitor::getPidRoundBudgetUs());
    DEBUG_SERIAL.println(F(") us"));

    DEBUG_SERIAL.print(F("motor "));
    DEBUG_SERIAL.print(LoopMonitor::getAvgUs(SLOT_MOTOR_TASK));
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(LoopMonitor::getPeakUs(SLOT_MOTOR_TASK));
    DEBUG_SERIAL.print(F("/"));
    DEBUG_SERIAL.print(LoopMonitor::getMaxUs(SLOT_MOTOR_TASK));
    DEBUG_SERIAL.print(F(" ("));
    DEBUG_SERIAL.print(LoopMonitor::getBudgetUs(SLOT_MOTOR_TASK));
    DEBUG_SERIAL.println(F(") us"));
    DEBUG_SERIAL.println();

    LoopMonitor::clearPeaks();
}

/**
 * @brief Test state machine task (1Hz)
 */
void taskTestStateMachine() {
    uint32_t currentTime = millis();

    // Check if it's time to advance state
    if (currentTime - stateStartTime < STATE_DURATION_MS) {
        return;  // Still in current state
    }

    stateStartTime = currentTime;

    // Advance to next state
    switch (currentState) {
        case TEST_INIT:
            DEBUG_SERIAL.println(F("\n=== TEST: Velocity Control (Positive) ==="));
            motor1.enable(DC_MODE_VELOCITY);
            motor2.enable(DC_MODE_VELOCITY);
            motor3.enable(DC_MODE_VELOCITY);
            motor4.enable(DC_MODE_VELOCITY);
            motor1.setTargetVelocity(TEST_VELOCITY_POS);
            motor2.setTargetVelocity(TEST_VELOCITY_POS);
            motor3.setTargetVelocity(TEST_VELOCITY_POS);
            motor4.setTargetVelocity(TEST_VELOCITY_POS);
            currentState = TEST_VELOCITY_POSITIVE;
            break;

        case TEST_VELOCITY_POSITIVE:
            DEBUG_SERIAL.println(F("\n=== TEST: Velocity Control (Negative) ==="));
            motor1.setTargetVelocity(TEST_VELOCITY_NEG);
            motor2.setTargetVelocity(TEST_VELOCITY_NEG);
            motor3.setTargetVelocity(TEST_VELOCITY_NEG);
            motor4.setTargetVelocity(TEST_VELOCITY_NEG);
            currentState = TEST_VELOCITY_NEGATIVE;
            break;

        case TEST_VELOCITY_NEGATIVE:
            DEBUG_SERIAL.println(F("\n=== TEST: Velocity Control (Zero) ==="));
            motor1.setTargetVelocity(0);
            motor2.setTargetVelocity(0);
            motor3.setTargetVelocity(0);
            motor4.setTargetVelocity(0);
            currentState = TEST_VELOCITY_ZERO;
            break;

        case TEST_VELOCITY_ZERO:
            DEBUG_SERIAL.println(F("\n=== TEST: Position Control (Forward) ==="));
            motor1.enable(DC_MODE_POSITION);
            motor2.enable(DC_MODE_POSITION);
            motor3.enable(DC_MODE_POSITION);
            motor4.enable(DC_MODE_POSITION);
            encoder1.resetCount();
            encoder2.resetCount();
            encoder3.resetCount();
            encoder4.resetCount();
            motor1.setTargetPosition(TEST_POSITION_FWD);
            motor2.setTargetPosition(TEST_POSITION_FWD);
            motor3.setTargetPosition(TEST_POSITION_FWD);
            motor4.setTargetPosition(TEST_POSITION_FWD);
            currentState = TEST_POSITION_FORWARD;
            break;

        case TEST_POSITION_FORWARD:
            DEBUG_SERIAL.println(F("\n=== TEST: Position Control (Reverse) ==="));
            // Re-enable resets both PIDs (clears velocity integral windup that
            // accumulated while holding position against friction at +720 ticks).
            // Without this, the saturated velocity integral fights the proportional
            // term and the motors may not start moving in the reverse direction.
            motor1.enable(DC_MODE_POSITION);
            motor2.enable(DC_MODE_POSITION);
            motor3.enable(DC_MODE_POSITION);
            motor4.enable(DC_MODE_POSITION);
            motor1.setTargetPosition(TEST_POSITION_REV);
            motor2.setTargetPosition(TEST_POSITION_REV);
            motor3.setTargetPosition(TEST_POSITION_REV);
            motor4.setTargetPosition(TEST_POSITION_REV);
            currentState = TEST_POSITION_REVERSE;
            break;

        case TEST_POSITION_REVERSE:
            DEBUG_SERIAL.println(F("\n=== TEST COMPLETE ==="));
            motor1.disable();
            motor2.disable();
            motor3.disable();
            motor4.disable();
            currentState = TEST_DONE;
            break;

        case TEST_DONE:
            // Stay in done state
            break;
    }
}

// ============================================================================
// SETUP
// ============================================================================

void setup() {
    // Initialize Debug Serial
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000); // Wait for connection

    DEBUG_SERIAL.println();
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println(F("  DC Motor PID Control Test - Phase 3"));
    DEBUG_SERIAL.println(F("========================================"));

    // Initialize Scheduler
    Scheduler::init();
    LoopMonitor::init();
    DEBUG_SERIAL.println(F("[Setup] Scheduler initialized"));

    // Initialize encoders (with direction flags from config.h)
    encoder1.init(PIN_M1_ENC_A, PIN_M1_ENC_B, ENCODER_1_DIR_INVERTED);
    encoder2.init(PIN_M2_ENC_A, PIN_M2_ENC_B, ENCODER_2_DIR_INVERTED);
    encoder3.init(PIN_M3_ENC_A, PIN_M3_ENC_B, ENCODER_3_DIR_INVERTED);
    encoder4.init(PIN_M4_ENC_A, PIN_M4_ENC_B, ENCODER_4_DIR_INVERTED);
    DEBUG_SERIAL.println(F("[Setup] Encoders initialized"));

    // M1/M2: hardware INT pins — use attachInterrupt()
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_A), encoderISR_M1, CHANGE);
#if ENCODER_1_MODE == ENCODER_4X
    attachInterrupt(digitalPinToInterrupt(PIN_M1_ENC_B), encoderISR_M1_B, CHANGE);
#endif
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_A), encoderISR_M2, CHANGE);
#if ENCODER_2_MODE == ENCODER_4X
    attachInterrupt(digitalPinToInterrupt(PIN_M2_ENC_B), encoderISR_M2_B, CHANGE);
#endif

    // M3: PCINT2 group (Port K) — A14=PCINT22=PK6, A15=PCINT23=PK7
    // Enable phase A; also enable phase B in 4x mode (processEdge reads both pins).
    PCMSK2 |= (1 << PCINT22);
#if ENCODER_3_MODE == ENCODER_4X
    PCMSK2 |= (1 << PCINT23);
#endif
    PCICR  |= (1 << PCIE2);

    // M4: PCINT0 group (Port B) — pin11=PCINT5=PB5, pin12=PCINT6=PB6
    PCMSK0 |= (1 << PCINT5);
#if ENCODER_4_MODE == ENCODER_4X
    PCMSK0 |= (1 << PCINT6);
#endif
    PCICR  |= (1 << PCIE0);

    DEBUG_SERIAL.println(F("[Setup] Encoder interrupts attached (M1/M2: INT, M3/M4: PCINT)"));

    // Initialize velocity estimators
    uint16_t countsPerRev1 = ENCODER_PPR * encoder1.getResolutionMultiplier();
    uint16_t countsPerRev2 = ENCODER_PPR * encoder2.getResolutionMultiplier();
    uint16_t countsPerRev3 = ENCODER_PPR * encoder3.getResolutionMultiplier();
    uint16_t countsPerRev4 = ENCODER_PPR * encoder4.getResolutionMultiplier();

    velocityEst1.init(countsPerRev1);
    velocityEst1.setFilterSize(VELOCITY_FILTER_SIZE);
    velocityEst1.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

    velocityEst2.init(countsPerRev2);
    velocityEst2.setFilterSize(VELOCITY_FILTER_SIZE);
    velocityEst2.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

    velocityEst3.init(countsPerRev3);
    velocityEst3.setFilterSize(VELOCITY_FILTER_SIZE);
    velocityEst3.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

    velocityEst4.init(countsPerRev4);
    velocityEst4.setFilterSize(VELOCITY_FILTER_SIZE);
    velocityEst4.setZeroTimeout(VELOCITY_ZERO_TIMEOUT);

    DEBUG_SERIAL.println(F("[Setup] Velocity estimators initialized"));

    // Initialize motors (with direction flags from config.h)
    motor1.init(0, &encoder1, &velocityEst1, DC_MOTOR_1_DIR_INVERTED);
    motor1.setPins(PIN_M1_EN, PIN_M1_IN1, PIN_M1_IN2);
    motor1.setCurrentPin(PIN_M1_CT, CURRENT_SENSE_MA_PER_VOLT);
    motor1.setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
    motor1.setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);

    motor2.init(1, &encoder2, &velocityEst2, DC_MOTOR_2_DIR_INVERTED);
    motor2.setPins(PIN_M2_EN, PIN_M2_IN1, PIN_M2_IN2);
    motor2.setCurrentPin(PIN_M2_CT, CURRENT_SENSE_MA_PER_VOLT);
    motor2.setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
    motor2.setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);

    motor3.init(2, &encoder3, &velocityEst3, DC_MOTOR_3_DIR_INVERTED);
    motor3.setPins(PIN_M3_EN, PIN_M3_IN1, PIN_M3_IN2);
    motor3.setCurrentPin(PIN_M3_CT, CURRENT_SENSE_MA_PER_VOLT);
    motor3.setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
    motor3.setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);

    motor4.init(3, &encoder4, &velocityEst4, DC_MOTOR_4_DIR_INVERTED);
    motor4.setPins(PIN_M4_EN, PIN_M4_IN1, PIN_M4_IN2);
    motor4.setCurrentPin(PIN_M4_CT, CURRENT_SENSE_MA_PER_VOLT);
    motor4.setPositionPID(DEFAULT_POS_KP, DEFAULT_POS_KI, DEFAULT_POS_KD);
    motor4.setVelocityPID(DEFAULT_VEL_KP, DEFAULT_VEL_KI, DEFAULT_VEL_KD);

    DEBUG_SERIAL.println(F("[Setup] All 4 motors initialized"));

    // Register scheduler tasks
    Scheduler::registerTask(taskPrintStatus, 200, 1);       // 200ms (5Hz)
    Scheduler::registerTask(taskTestStateMachine, 2000, 2); // 2000ms state timer

    DEBUG_SERIAL.println(F("[Setup] Tasks registered"));
    DEBUG_SERIAL.println(F("[Setup] Motor compute is round-driven from Timer1 slot-3 flag"));
    DEBUG_SERIAL.println();
    DEBUG_SERIAL.print(F("PID Vel: Kp="));  DEBUG_SERIAL.print(DEFAULT_VEL_KP);
    DEBUG_SERIAL.print(F(" Ki="));          DEBUG_SERIAL.print(DEFAULT_VEL_KI);
    DEBUG_SERIAL.print(F(" Kd="));          DEBUG_SERIAL.println(DEFAULT_VEL_KD);
    DEBUG_SERIAL.print(F("PID Pos: Kp="));  DEBUG_SERIAL.print(DEFAULT_POS_KP);
    DEBUG_SERIAL.print(F(" Ki="));          DEBUG_SERIAL.print(DEFAULT_POS_KI);
    DEBUG_SERIAL.print(F(" Kd="));          DEBUG_SERIAL.println(DEFAULT_POS_KD);
    DEBUG_SERIAL.println(F("Control: mixed 800Hz ISR latch/apply + 200Hz soft compute"));
    DEBUG_SERIAL.println(F("Test sequence (5s each): +vel / -vel / 0 / +pos / -pos"));
    DEBUG_SERIAL.println(F("========================================"));
    DEBUG_SERIAL.println();

    // Configure Timer1 and Timer4 for Fast PWM (ICR-based TOP).
    // MUST be called last — enables OC4A/OC4B hardware PWM on pins 6/7
    // (M1_EN, M2_EN) and sets ICR4=199, without which OCR4A/B writes
    // always produce 0 and motors never move.
    ISRScheduler::init();
    DEBUG_SERIAL.println(F("[Setup] ISRScheduler initialized (Timer1/4 Fast PWM)"));

    stateStartTime = millis();
}

// ============================================================================
// LOOP
// ============================================================================

void loop() {
    taskMotorPID();
    Scheduler::tick();
    taskMotorPID();
}
