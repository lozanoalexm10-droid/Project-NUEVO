/**
 * @file MessageCenter.h
 * @brief Central message routing and TLV communication handler (v2.0)
 *
 * This module manages all UART communication with the Raspberry Pi:
 * - Receives and parses incoming TLV messages (v2.0 protocol)
 * - Routes commands to appropriate subsystems (motors, servos, LEDs, IMU)
 * - Generates and sends outgoing telemetry at appropriate rates
 * - Monitors heartbeat and enforces liveness timeout
 *
 * Message Flow:
 *   RPi → UART → MessageCenter → Parse TLV → Route to module
 *   Module → Generate TLV → MessageCenter → UART → RPi
 *
 * Batched telemetry:
 *   sendTelemetry() opens a single frame, conditionally appends each TLV
 *   based on millis-gated intervals, then queues the completed frame for
 *   non-blocking UART drain from loop(). This cuts per-frame overhead
 *   (header + CRC) from N copies down to one.
 *
 * Safety:
 * - Liveness timeout: heartbeatValid_ goes false; SafetyManager responds on next 100 Hz check
 * - State machine now owned by SystemManager; MessageCenter routes transitions via requestTransition()
 *
 * Usage:
 *   MessageCenter::init();
 *
 *   // In scheduler task @ 50Hz:
 *   MessageCenter::processIncoming();
 *   MessageCenter::sendTelemetry();
 */

#ifndef MESSAGECENTER_H
#define MESSAGECENTER_H

#include <Arduino.h>
#include <stdint.h>
#include "../lib/tlvcodec.h"
#include "../messages/TLV_TypeDefs.h"
#include "../messages/TLV_Payloads.h"
#include "../config.h"
#include "RobotKinematics.h"

// Maximum TLV payload size we will accept from the RPi
#define MAX_TLV_PAYLOAD_SIZE 256
// RX frame storage only needs to hold the largest accepted inbound frame:
//   28-byte FrameHeader + 8-byte TLV header + 256-byte payload + padding.
#define RX_BUFFER_SIZE 384
// The bridge currently sends exactly one command TLV per frame. The largest
// supported inbound command is SERVO_SET bulk: 28-byte frame header + 8-byte
// TLV header + 34-byte payload = 70 bytes, padded to 72. Keep some margin, but
// reject implausibly large inbound frames so a corrupted length field cannot
// hold the decoder in WaitFullFrame long enough to trip liveness.
#define RX_MAX_FRAME_ACCEPT_SIZE 96
// Keep RUNNING telemetry frames small enough that one frame does not occupy the
// UART for an entire 20 ms task period. Large multi-TLV bursts were driving the
// queued TX length near 500 bytes and correlating with heartbeat loss.
#define TX_BUFFER_SIZE 256

// ============================================================================
// MESSAGE CENTER CLASS (Static)
// ============================================================================

/**
 * @brief Central TLV message routing and communication handler
 *
 * Static class providing:
 * - TLV v2.0 message parsing and routing
 * - Firmware state machine management
 * - Batched telemetry generation and transmission
 * - Liveness monitoring and safety timeout
 */
class MessageCenter
{
public:
    /**
     * @brief Initialize message center
     *
     * Opens Serial2 at RPI_BAUD_RATE and initialises the TLV codec.
     * Sets firmware state to SYS_STATE_IDLE.
     * Must be called once in setup().
     */
    static void init();

    // ========================================================================
    // MESSAGE PROCESSING
    // ========================================================================

    /**
     * @brief Drain the hardware UART ring buffer directly into the TLV decoder.
     *
     * Call from loop() on EVERY iteration, BEFORE Scheduler::tick().
     * Feeds bytes straight into decodePacket() at loop() rate, avoiding a
     * second 1 KB staging buffer on top of the TLV decoder's own frame buffer.
     */
    static void drainUart();

    /**
     * @brief Drain queued TX bytes to the Raspberry Pi UART without blocking.
     *
     * Called from loop() on every iteration. Uses direct UDR2 polling so TX
     * does not rely on per-byte USART TX interrupts.
     */
    static void drainTx();

    /**
     * @brief Process incoming messages from UART
     *
     * drainUart() already feeds bytes into the decoder continuously. This task
     * keeps the heartbeat timeout logic on a predictable 50 Hz cadence.
     * Should be called from the scheduler at 50 Hz.
     */
    static void processIncoming();

    /**
     * @brief Apply deferred slow command side effects outside decode context.
     *
     * Servo PCA9685 writes and magnetometer calibration EEPROM/apply actions
     * are queued by the decode handlers and executed later from the soft task.
     */
    static void processDeferred();

    /**
     * @brief Send telemetry data to RPi
     *
     * Opens a single TLV frame, conditionally appends each message type
     * based on its millis interval, then queues the finished frame for
     * non-blocking UART drain from loop().
     *
     * Rates:
     * - DC_STATUS_ALL      ( 25 Hz, RUNNING)
     * - STEP_STATUS_ALL    (  5 Hz, RUNNING)
     * - SERVO_STATUS_ALL   (  5 Hz, RUNNING)
     * - SENSOR_IMU         ( 25 Hz, RUNNING, if IMU attached)
     * - SENSOR_KINEMATICS  ( 25 Hz, RUNNING)
     * - IO_STATUS          ( 25 Hz, RUNNING)
     * - SENSOR_RANGE lidar ( 10 Hz, RUNNING, per configured lidar slot)
     * - SENSOR_RANGE sonic (  5 Hz, RUNNING, per configured ultrasonic slot)
     *   (status=3 / not-installed is reported for slots that did not respond on init)
     * - SENSOR_VOLTAGE     (  5 Hz, RUNNING or ERROR)
     * - SYS_STATUS         (  1 Hz IDLE/ESTOP, 10 Hz RUNNING/ERROR)
     * - SENSOR_MAG_CAL_STATUS ( 5 Hz while sampling; immediately on cmd response)
     *
     * Should be called from scheduler at 50Hz.
     */
    static void sendTelemetry();

    // ========================================================================
    // LIVENESS
    // ========================================================================

    /**
     * @brief Check if liveness is valid
     *
     * Returns false if no TLV received within timeout period.
     *
     * @return True if liveness is valid
     */
    static bool isHeartbeatValid();

    /**
     * @brief Get time since last received TLV
     *
     * @return Milliseconds since last TLV received from RPi
     */
    static uint32_t getTimeSinceHeartbeat();

    /**
     * @brief Immediately disable all DC motors, steppers, and servos
     *
     * Called by SafetyManager on hard fault, and by handleSysCmd() on STOP/RESET/ESTOP.
     * Safe to call from ISR context.
     */
    static void disableAllActuators();

    /**
     * @brief Latch the error flags that caused the current ERROR state.
     *
     * Called by SafetyManager BEFORE forceState(ERROR) so the cause is not lost
     * when the state transitions away from RUNNING/IDLE.  The latched flags are
     * OR'd into every subsequent SYS_STATUS errorFlags field until CMD_RESET clears
     * them.  Safe to call from ISR context (volatile write, no heap alloc).
     *
     * @param flags  SystemErrorFlags bitmask describing the triggering fault(s)
     */
    static void latchFaultFlags(uint8_t flags);

    /**
     * @brief Record UART task wall-clock time for diagnostic display
     *
     * Maintains exponential moving average and per-window max.
     * Values are reported in SYS_STATUS (loopTimeAvgUs / loopTimeMaxUs).
     *
     * NOTE: elapsedUs is measured with micros() in loop() while interrupts are
     * enabled — it includes ISR preemption time (Timer1 PID, Timer3 stepper).
     * This is NOT a control-loop overrun measurement; the PID loop runs in
     * Timer1 ISR and is completely unaffected by anything that happens here.
     *
     * @param elapsedUs  Wall-clock microseconds for this taskUART() call
     */
    static void recordLoopTime(uint32_t elapsedUs);

    // ---- Diagnostic accessors (for debug serial / SYS_STATUS) ----
    static uint16_t getLoopTimeAvgUs()  { return loopTimeAvgUs_; }
    static uint16_t getLoopTimeMaxUs()  { return loopTimeMaxUs_; }
    static uint16_t getUartRxErrors()   { return uartRxErrors_; }
    static uint16_t getCrcErrorCount()  { return crcErrorCount_; }
    static uint16_t getFrameLenErrorCount() { return frameLenErrorCount_; }
    static uint16_t getTlvErrorCount()  { return tlvErrorCount_; }
    static uint16_t getOversizeErrorCount() { return oversizeErrorCount_; }
    static uint32_t getTxDroppedFrames(){ return txDroppedFrames_; }
    static uint16_t getTxPendingBytes() { return (txPendingOffset_ < txPendingLen_) ? (uint16_t)(txPendingLen_ - txPendingOffset_) : 0; }
    static uint32_t getTimeSinceRxByte() { return millis() - lastRxByteMs_; }
    static uint8_t getFaultLatchFlags() { return faultLatchFlags_; }
    static uint16_t getHeartbeatTimeoutMs() { return heartbeatTimeoutMs_; }
    static uint16_t getServoEnabledMask() { return servoEnabledMask_; }
    static uint16_t getFreeRAM();
    static void snapshotTrafficWindow(uint16_t &rxBytes,
                                      uint16_t &rxFrames,
                                      uint16_t &rxTlvs,
                                      uint16_t &rxHeartbeats,
                                      uint16_t &txBytes,
                                      uint16_t &txFrames);

private:
    // ---- TLV codec (owned directly, no UARTDriver wrapper) ----
    static struct TlvEncodeDescriptor encodeDesc_;
    static struct TlvDecodeDescriptor decodeDesc_;
    static uint8_t txStorage_[TX_BUFFER_SIZE];
    static uint8_t rxStorage_[RX_BUFFER_SIZE];
    static struct TlvHeader decodeHeaders_[TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(RX_BUFFER_SIZE)];
    static uint8_t *decodeData_[TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(RX_BUFFER_SIZE)];
    static uint8_t *txBuffer_;                // Alias of txStorage_ for drainTx()
    static uint16_t txPendingLen_;            // Bytes queued in txBuffer_
    static uint16_t txPendingOffset_;         // Next queued TX byte to send
    static uint32_t txDroppedFrames_;         // Frames skipped because prior TX still draining

    // ---- Liveness tracking ----
    static uint32_t lastHeartbeatMs_;    // millis() of last received TLV
    static uint32_t lastCmdMs_;          // millis() of last non-heartbeat command
    static uint32_t lastRxByteMs_;       // millis() of last raw UART byte received
    static bool heartbeatValid_;         // False if liveness timeout
    static uint16_t heartbeatTimeoutMs_; // Configurable timeout (ms)

    // ---- Fault latch (cleared only by CMD_RESET) ----
    // Captures the error flags that triggered the current ERROR state so they
    // remain visible in SYS_STATUS even after the state has already changed to ERROR
    // (at which point live flag conditions may no longer evaluate true).
    static volatile uint8_t faultLatchFlags_;

    // ---- Configuration (from SYS_CONFIG) ----
    static uint8_t motorDirMask_;  // Direction inversion bitmask
    static uint8_t neoPixelCount_; // Configured NeoPixel count

    // ---- Servo enable tracking (bit N = channel N enabled) ----
    static uint16_t servoEnabledMask_;

    // ---- Loop timing statistics (updated externally) ----
    static uint16_t loopTimeAvgUs_;
    static uint16_t loopTimeMaxUs_;

    // ---- UART error counter ----
    static uint16_t uartRxErrors_;
    static uint16_t crcErrorCount_;
    static uint16_t frameLenErrorCount_;
    static uint16_t tlvErrorCount_;
    static uint16_t oversizeErrorCount_;
    static uint16_t rxBytesWindow_;
    static uint16_t rxFramesWindow_;
    static uint16_t rxTlvsWindow_;
    static uint16_t rxHeartbeatsWindow_;
    static uint16_t txBytesWindow_;
    static uint16_t txFramesWindow_;

    // ---- Telemetry timing ----
    static uint32_t lastDCStatusSendMs_;
    static uint32_t lastStepStatusSendMs_;
    static uint32_t lastServoStatusSendMs_;
    static uint32_t lastIMUSendMs_;
    static uint32_t lastKinematicsSendMs_;
    static uint32_t lastVoltageSendMs_;
    static uint32_t lastIOStatusSendMs_;
    static uint32_t lastStatusSendMs_;
    static uint32_t lastMagCalSendMs_;
    static uint32_t lastLidarSendMs_;
    static uint32_t lastUltrasonicSendMs_;

    // ---- Queued async response ----
    // Set by handleMagCalCmd on STOP/SAVE/APPLY/CLEAR so the response is
    // included in the very next sendTelemetry() frame rather than sent as a
    // standalone frame from within the command handler.
    static bool pendingMagCal_;
    static bool pendingServoStatus_;
    static bool pendingDCStatus_;

    // ---- Deferred servo side effects ----
    static bool servoHardwareDirty_;
    static uint16_t servoPulseDirtyMask_;
    static uint16_t servoOffDirtyMask_;
    static uint16_t servoPendingPulseUs_[NUM_SERVO_CHANNELS];
    static bool servoUnavailableLogged_;
    static bool servoI2CFaultLogged_;

    enum DeferredMagCalAction : uint8_t {
        DEFER_MAG_NONE = 0,
        DEFER_MAG_START,
        DEFER_MAG_STOP,
        DEFER_MAG_SAVE,
        DEFER_MAG_APPLY,
        DEFER_MAG_CLEAR,
    };

    static DeferredMagCalAction deferredMagCalAction_;
    static float deferredMagCalOffsetX_;
    static float deferredMagCalOffsetY_;
    static float deferredMagCalOffsetZ_;

    // ---- Initialization flag ----
    static bool initialized_;

    // ========================================================================
    // FRAME HELPERS
    // ========================================================================

    /**
     * @brief Reset the TX encoder for a new outgoing frame
     */
    static void beginFrame();
    static bool appendTlv(uint32_t tlvType, uint16_t tlvLen, const void *dataAddr);

    /**
     * @brief Finalise and transmit the accumulated frame over RPI_SERIAL
     *
     * No-ops if no TLVs have been appended since the last beginFrame().
     */
    static void sendFrame();

    /**
     * @brief TLV decode callback — called synchronously by decode() for each complete frame
     *
     * Loops over every TLV in the frame and routes each to routeMessage().
     * Incoming multi-TLV frames are therefore handled correctly.
     */
    static void decodeCallback(enum DecodeErrorCode *error,
                               const struct FrameHeader *frameHeader,
                               struct TlvHeader *tlvHeaders,
                               uint8_t **tlvData);

    // ========================================================================
    // MESSAGE ROUTING
    // ========================================================================

    /**
     * @brief Route received message to appropriate handler
     *
     * Updates liveness timer for any received TLV.
     * Enforces state-based command gating.
     *
     * @param type Message type (from TLV_TypeDefs.h)
     * @param payload Pointer to message payload
     * @param length Payload length in bytes
     */
    static void routeMessage(uint32_t type, const uint8_t *payload, uint32_t length);

    /**
     * @brief Check liveness timeout and disable actuators if expired
     */
    static void checkHeartbeatTimeout();

    // ---- System message handlers ----
    static void handleHeartbeat(const PayloadHeartbeat *payload);
    static void handleSysCmd(const PayloadSysCmd *payload);
    static void handleSysConfig(const PayloadSysConfig *payload);
    static void handleSetPID(const PayloadSetPID *payload);

    // ---- DC motor message handlers ----
    static void handleDCEnable(const PayloadDCEnable *payload);
    static void handleDCSetPosition(const PayloadDCSetPosition *payload);
    static void handleDCSetVelocity(const PayloadDCSetVelocity *payload);
    static void handleDCSetPWM(const PayloadDCSetPWM *payload);

    // ---- Stepper motor message handlers ----
    static void handleStepEnable(const PayloadStepEnable *payload);
    static void handleStepSetParams(const PayloadStepSetParams *payload);
    static void handleStepMove(const PayloadStepMove *payload);
    static void handleStepHome(const PayloadStepHome *payload);

    // ---- Servo message handlers ----
    static void handleServoEnable(const PayloadServoEnable *payload);
    static void handleServoSet(const PayloadServoSetSingle *payload);
    static void handleServoSetBulk(const PayloadServoSetBulk *payload);

    // ---- User I/O message handlers ----
    static void handleSetLED(const PayloadSetLED *payload);
    static void handleSetNeoPixel(const PayloadSetNeoPixel *payload);

    // ---- Magnetometer calibration handler ----
    static void handleMagCalCmd(const PayloadMagCalCmd *payload);

    // ---- Deferred command workers ----
    static void processDeferredServo();
    static void processDeferredMagCal();

    // ========================================================================
    // TELEMETRY APPENDERS
    // ========================================================================
    // Each method appends one TLV to the current frame buffer.
    // Must be called between beginFrame() and sendFrame().

    /** @brief Append all DC motor status (184 bytes payload) */
    static void sendDCStatusAll();

    /** @brief Append all stepper motor status (96 bytes payload) */
    static void sendStepStatusAll();

    /** @brief Append servo status for all 16 channels (36 bytes payload) */
    static void sendServoStatusAll();

    /** @brief Append IMU quaternion and raw sensor data (52 bytes payload) */
    static void sendSensorIMU();

    /** @brief Append wheel odometry kinematics (28 bytes payload) */
    static void sendSensorKinematics();

    /** @brief Append battery and rail voltages (8 bytes payload) */
    static void sendVoltageData();

    /** @brief Append button/limit states and NeoPixel RGB (variable payload) */
    static void sendIOStatus();

    /** @brief Append system state, version, and diagnostics (40 bytes payload) */
    static void sendSystemStatus();

    /** @brief Append magnetometer calibration progress (44 bytes payload) */
    static void sendMagCalStatus();

    /**
     * @brief Append SENSOR_RANGE packets for all configured range sensors.
     *
     * For sensors that responded during init: sends the latest distance reading
     * (status = 0, valid).
     * For sensors configured in config.h but absent on the I2C bus:
     * sends status = 3 (not installed) so the RPi can notify the user.
     *
     * @param lidarOnly  true = only lidar sensors (10 Hz path);
     *                   false = only ultrasonic sensors (5 Hz path)
     */
    static void sendSensorRange(bool lidarOnly);

    // ========================================================================
    // HELPERS
    // ========================================================================
};

#endif // MESSAGECENTER_H
