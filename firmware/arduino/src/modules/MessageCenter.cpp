/**
 * @file MessageCenter.cpp
 * @brief Implementation of central message routing (TLV protocol v2.0)
 *
 * Telemetry batching:
 *   sendTelemetry() calls beginFrame() once, appends each due TLV with
 *   appendTlv(), then calls sendFrame() to queue a single multi-TLV frame
 *   per scheduler tick instead of N separate frames.
 *
 * Incoming:
 *   processIncoming() feeds raw bytes into the TLV decoder; decodeCallback()
 *   is invoked synchronously for each complete frame and routes every TLV in
 *   that frame to routeMessage().
 */

#include "MessageCenter.h"
#include "DebugLog.h"
#include "LoopMonitor.h"
#include "RobotKinematics.h"
#include "SensorManager.h"
#include "UserIO.h"
#include "../SystemManager.h"
#include "../drivers/DCMotor.h"
#include "../drivers/StepperMotor.h"
#include "../drivers/ServoController.h"
#include "StepperManager.h"
#include <string.h>
#include <math.h>

// External references to motor arrays (defined in arduino.ino)
extern DCMotor dcMotors[NUM_DC_MOTORS];

#ifdef DEBUG_UART_RX_BYTES
static void logRawRxBurst(const uint8_t *bytes, uint8_t count)
{
    if (bytes == nullptr || count == 0) {
        return;
    }

    DEBUG_LOG.print(F("[RXB]"));
    for (uint8_t i = 0; i < count; ++i) {
        DEBUG_LOG.write(' ');
        if (bytes[i] < 0x10U) {
            DEBUG_LOG.write('0');
        }
        DEBUG_LOG.print((unsigned int)bytes[i], HEX);
    }
    DEBUG_LOG.write('\n');
}
#endif

// ============================================================================
// STATIC MEMBER INITIALIZATION
// ============================================================================

struct TlvEncodeDescriptor MessageCenter::encodeDesc_;
struct TlvDecodeDescriptor MessageCenter::decodeDesc_;

uint8_t MessageCenter::txStorage_[TX_BUFFER_SIZE];
uint8_t MessageCenter::rxStorage_[RX_BUFFER_SIZE];
struct TlvHeader MessageCenter::decodeHeaders_[TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(RX_BUFFER_SIZE)];
uint8_t *MessageCenter::decodeData_[TLVCODEC_TLV_SLOTS_FOR_FRAME_BYTES(RX_BUFFER_SIZE)];
uint8_t *MessageCenter::txBuffer_;
uint16_t MessageCenter::txPendingLen_ = 0;
uint16_t MessageCenter::txPendingOffset_ = 0;
uint32_t MessageCenter::txDroppedFrames_ = 0;

uint32_t MessageCenter::lastHeartbeatMs_ = 0;
uint32_t MessageCenter::lastCmdMs_ = 0;
uint32_t MessageCenter::lastRxByteMs_ = 0;
bool MessageCenter::heartbeatValid_ = true;   // true → 2s grace period on boot before timeout fires
uint16_t MessageCenter::heartbeatTimeoutMs_ = HEARTBEAT_TIMEOUT_MS;

uint8_t MessageCenter::motorDirMask_ = 0;
uint8_t MessageCenter::neoPixelCount_ = NEOPIXEL_COUNT;

uint16_t MessageCenter::servoEnabledMask_ = 0;
uint16_t MessageCenter::loopTimeAvgUs_ = 0;
uint16_t MessageCenter::loopTimeMaxUs_ = 0;
uint16_t MessageCenter::uartRxErrors_ = 0;
uint16_t MessageCenter::crcErrorCount_ = 0;
uint16_t MessageCenter::frameLenErrorCount_ = 0;
uint16_t MessageCenter::tlvErrorCount_ = 0;
uint16_t MessageCenter::oversizeErrorCount_ = 0;
uint16_t MessageCenter::rxBytesWindow_ = 0;
uint16_t MessageCenter::rxFramesWindow_ = 0;
uint16_t MessageCenter::rxTlvsWindow_ = 0;
uint16_t MessageCenter::rxHeartbeatsWindow_ = 0;
uint16_t MessageCenter::txBytesWindow_ = 0;
uint16_t MessageCenter::txFramesWindow_ = 0;

uint32_t MessageCenter::lastDCStatusSendMs_ = 0;
uint32_t MessageCenter::lastStepStatusSendMs_ = 0;
uint32_t MessageCenter::lastServoStatusSendMs_ = 0;
uint32_t MessageCenter::lastIMUSendMs_ = 0;
uint32_t MessageCenter::lastKinematicsSendMs_ = 0;
uint32_t MessageCenter::lastVoltageSendMs_ = 0;
uint32_t MessageCenter::lastIOStatusSendMs_ = 0;
uint32_t MessageCenter::lastStatusSendMs_ = 0;
uint32_t MessageCenter::lastMagCalSendMs_ = 0;
uint32_t MessageCenter::lastLidarSendMs_ = 0;
uint32_t MessageCenter::lastUltrasonicSendMs_ = 0;

bool MessageCenter::pendingMagCal_ = false;
bool MessageCenter::pendingServoStatus_ = false;
bool MessageCenter::pendingDCStatus_ = false;
bool MessageCenter::servoHardwareDirty_ = false;
uint16_t MessageCenter::servoPulseDirtyMask_ = 0;
uint16_t MessageCenter::servoOffDirtyMask_ = 0;
uint16_t MessageCenter::servoPendingPulseUs_[NUM_SERVO_CHANNELS] = {0};
bool MessageCenter::servoUnavailableLogged_ = false;
bool MessageCenter::servoI2CFaultLogged_ = false;
MessageCenter::DeferredMagCalAction MessageCenter::deferredMagCalAction_ = MessageCenter::DEFER_MAG_NONE;
float MessageCenter::deferredMagCalOffsetX_ = 0.0f;
float MessageCenter::deferredMagCalOffsetY_ = 0.0f;
float MessageCenter::deferredMagCalOffsetZ_ = 0.0f;
bool MessageCenter::initialized_ = false;
volatile uint8_t MessageCenter::faultLatchFlags_ = 0;

// ============================================================================
// INITIALIZATION
// ============================================================================

void MessageCenter::init()
{
    if (initialized_)
        return;

    // Open Serial2 for RPi communication
    RPI_SERIAL.begin(RPI_BAUD_RATE);

    // Initialise TX encoder using static storage to avoid AVR heap pressure.
    memset(&encodeDesc_, 0, sizeof(encodeDesc_));
    encodeDesc_.buffer = txStorage_;
    encodeDesc_.bufferSize = TX_BUFFER_SIZE;
    encodeDesc_.bufferIndex = sizeof(struct FrameHeader);
    encodeDesc_.crc = ENABLE_CRC_CHECK;
    memcpy(encodeDesc_.frameHeader.magicNum, FRAME_HEADER_MAGIC_NUM, sizeof(FRAME_HEADER_MAGIC_NUM));
    encodeDesc_.frameHeader.deviceId = DEVICE_ID;
    txBuffer_ = encodeDesc_.buffer;

    // Initialise RX decoder using static storage to avoid malloc() in setup().
    memset(&decodeDesc_, 0, sizeof(decodeDesc_));
    decodeDesc_.buffer = rxStorage_;
    decodeDesc_.bufferSize = RX_MAX_FRAME_ACCEPT_SIZE;
    decodeDesc_.crc = ENABLE_CRC_CHECK;
    decodeDesc_.decodeState = Init;
    decodeDesc_.errorCode = NoError;
    decodeDesc_.callback = decodeCallback;
    decodeDesc_.tlvHeaders = decodeHeaders_;
    decodeDesc_.tlvData = decodeData_;

    // Stagger the bring-up telemetry timers so the 25 Hz traffic alternates
    // across adjacent 20 ms UART task slots.
    uint32_t now = millis();
    lastDCStatusSendMs_    = now;       // first 25 Hz DC frame at now+40 ms
    lastStepStatusSendMs_  = now - 180; // first 5 Hz stepper frame at now+20 ms
    lastServoStatusSendMs_ = now - 80;  // first 5 Hz servo frame at now+120 ms
    lastKinematicsSendMs_  = now - 20;  // first 25 Hz kinematics frame at now+20 ms
    lastIMUSendMs_         = now - 20;  // first 25 Hz IMU frame at now+20 ms
    lastIOStatusSendMs_    = now;       // first 25 Hz I/O frame at now+40 ms
    lastVoltageSendMs_     = now;
    lastStatusSendMs_      = now;
    lastMagCalSendMs_      = now;
    lastLidarSendMs_       = now - 80;  // first 10 Hz lidar frame at now+20 ms
    lastUltrasonicSendMs_  = now - 180; // first 5 Hz ultrasonic frame at now+20 ms

    lastHeartbeatMs_ = now;
    lastCmdMs_ = now;
    lastRxByteMs_ = now;
    // Start valid so SafetyManager doesn't fire during the boot grace period.
    // checkHeartbeatTimeout() will clear this after HEARTBEAT_TIMEOUT_MS if no
    // TLV arrives. routeMessage() re-sets it on every received packet.
    heartbeatValid_ = true;
    pendingMagCal_ = false;
    pendingServoStatus_ = false;
    pendingDCStatus_ = false;
    rxBytesWindow_ = 0;
    rxFramesWindow_ = 0;
    rxTlvsWindow_ = 0;
    rxHeartbeatsWindow_ = 0;
    txBytesWindow_ = 0;
    txFramesWindow_ = 0;
    servoHardwareDirty_ = false;
    servoPulseDirtyMask_ = 0;
    servoOffDirtyMask_ = 0;
    memset(servoPendingPulseUs_, 0, sizeof(servoPendingPulseUs_));
    servoUnavailableLogged_ = false;
    servoI2CFaultLogged_ = false;
    deferredMagCalAction_ = DEFER_MAG_NONE;
    deferredMagCalOffsetX_ = 0.0f;
    deferredMagCalOffsetY_ = 0.0f;
    deferredMagCalOffsetZ_ = 0.0f;
    RobotKinematics::reset(0, 0);
    SystemManager::requestTransition(SYS_STATE_IDLE);

    initialized_ = true;

#ifdef DEBUG_TLV_PACKETS
    DEBUG_LOG.println(F("[MessageCenter] Initialized (v2.0, batched TX)"));
#endif
}

// ============================================================================
// FRAME HELPERS
// ============================================================================

void MessageCenter::beginFrame()
{
    resetDescriptor(&encodeDesc_);
}

bool MessageCenter::appendTlv(uint32_t tlvType, uint16_t tlvLen, const void *dataAddr)
{
    const size_t required = sizeof(struct TlvHeader) + tlvLen + (MSG_BUFFER_SEGMENT_LEN - 1U);
    if (encodeDesc_.bufferIndex + required > encodeDesc_.bufferSize) {
        return false;
    }

    addTlvPacket(&encodeDesc_, tlvType, tlvLen, dataAddr);
    return true;
}

void MessageCenter::sendFrame()
{
    // Only send if at least one TLV was appended
    if (encodeDesc_.frameHeader.numTlvs == 0)
        return;

    if (txPendingOffset_ < txPendingLen_) {
        txDroppedFrames_++;
        return;
    }

    int n = wrapupBuffer(&encodeDesc_);
    if (n > 0)
    {
        txPendingLen_ = (uint16_t)n;
        txPendingOffset_ = 0;
        txFramesWindow_++;
        uint32_t txBytesSum = (uint32_t)txBytesWindow_ + (uint32_t)n;
        txBytesWindow_ = (txBytesSum > 0xFFFFUL) ? 0xFFFFU : (uint16_t)txBytesSum;
        drainTx();
    }

#ifdef DEBUG_TLV_PACKETS
    DEBUG_LOG.print(F("[TX] Frame: "));
    DEBUG_LOG.print(encodeDesc_.frameHeader.numTlvs);
    DEBUG_LOG.print(F(" TLVs, "));
    DEBUG_LOG.print(n);
    DEBUG_LOG.println(F(" bytes"));
#endif
}

void MessageCenter::drainTx()
{
    while (txPendingOffset_ < txPendingLen_ && bit_is_set(UCSR2A, UDRE2)) {
        UDR2 = txBuffer_[txPendingOffset_++];
    }

    if (txPendingOffset_ >= txPendingLen_) {
        txPendingLen_ = 0;
        txPendingOffset_ = 0;
    }
}

// ============================================================================
// LOOP TIMING
// ============================================================================

void MessageCenter::recordLoopTime(uint32_t elapsedUs) {
    // Exponential moving average (alpha = 1/8) for display in SYS_STATUS.
    // NOTE: elapsedUs includes ISR preemption time — see header for details.
    uint32_t clamped = (elapsedUs > 0xFFFF) ? 0xFFFF : elapsedUs;
    if (loopTimeAvgUs_ == 0)
        loopTimeAvgUs_ = (uint16_t)clamped;
    else
        loopTimeAvgUs_ = (uint16_t)(((uint32_t)loopTimeAvgUs_ * 7 + clamped) >> 3);

    // Per-window max: reset every ~2 s of taskUART activity.
    if (elapsedUs > loopTimeMaxUs_)
        loopTimeMaxUs_ = (uint16_t)clamped;

    static uint8_t windowTick = 0;
    if (++windowTick >= (uint8_t)(UART_COMMS_FREQ_HZ * 2U)) {
        windowTick    = 0;
        loopTimeMaxUs_ = 0;
    }
}

// ============================================================================
// MESSAGE PROCESSING
// ============================================================================

// ----------------------------------------------------------------------------
// drainUart — called from loop() on EVERY iteration
// ----------------------------------------------------------------------------
void MessageCenter::drainUart()
{
    // Feed incoming bytes straight into the TLV decoder. This avoids carrying
    // a second software FIFO in SRAM in addition to the decoder frame buffer.
    int b;
#ifdef DEBUG_UART_RX_BYTES
    uint8_t rxBurst[16];
    uint8_t rxBurstCount = 0;
#endif
    while ((b = RPI_SERIAL.read()) >= 0) {
        uint8_t byte = (uint8_t)b;
        lastRxByteMs_ = millis();
        if (rxBytesWindow_ != 0xFFFFU) {
            rxBytesWindow_++;
        }
#ifdef DEBUG_UART_RX_BYTES
        rxBurst[rxBurstCount++] = byte;
        if (rxBurstCount >= sizeof(rxBurst)) {
            logRawRxBurst(rxBurst, rxBurstCount);
            rxBurstCount = 0;
        }
#endif
        decodePacket(&decodeDesc_, &byte);
    }
#ifdef DEBUG_UART_RX_BYTES
    if (rxBurstCount > 0) {
        logRawRxBurst(rxBurst, rxBurstCount);
    }
#endif
}

// ----------------------------------------------------------------------------
// processIncoming — called from taskUART at 50 Hz
// ----------------------------------------------------------------------------
void MessageCenter::processIncoming()
{
    checkHeartbeatTimeout();
}

void MessageCenter::processDeferred()
{
    processDeferredMagCal();
    processDeferredServo();
}

void MessageCenter::processDeferredServo()
{
    if (!servoHardwareDirty_ && servoPulseDirtyMask_ == 0 && servoOffDirtyMask_ == 0) {
        return;
    }

    if (!ServoController::isInitialized()) {
        if (!servoUnavailableLogged_) {
            DEBUG_LOG.println(F("[I2C] Servo controller unavailable."));
            servoUnavailableLogged_ = true;
        }
        servoHardwareDirty_ = false;
        servoPulseDirtyMask_ = 0;
        servoOffDirtyMask_ = 0;
        pendingServoStatus_ = true;
        return;
    }

    servoUnavailableLogged_ = false;

    servoHardwareDirty_ = false;

    if (servoEnabledMask_ == 0) {
        if (ServoController::isEnabled()) {
            ServoController::disable();
        }
    } else if (!ServoController::isEnabled()) {
        ServoController::enable();
    }

    uint16_t offMask = servoOffDirtyMask_;
    servoOffDirtyMask_ = 0;
    for (uint8_t channel = 0; channel < NUM_SERVO_CHANNELS; ++channel) {
        if ((offMask & (uint16_t)(1u << channel)) != 0) {
            ServoController::setChannelOff(channel);
        }
    }

    uint16_t dirtyMask = servoPulseDirtyMask_;
    servoPulseDirtyMask_ = 0;
    uint8_t channel = 0;
    while (channel < NUM_SERVO_CHANNELS) {
        uint16_t bit = (uint16_t)(1u << channel);
        if ((dirtyMask & bit) == 0 || (servoEnabledMask_ & bit) == 0) {
            channel++;
            continue;
        }

        uint8_t start = channel;
        uint8_t count = 0;
        while (channel < NUM_SERVO_CHANNELS && count < NUM_SERVO_CHANNELS) {
            uint16_t runBit = (uint16_t)(1u << channel);
            if ((dirtyMask & runBit) == 0 || (servoEnabledMask_ & runBit) == 0) {
                break;
            }
            count++;
            channel++;
        }

        if (count == 1U) {
            ServoController::setPositionUs(start, servoPendingPulseUs_[start]);
        } else {
            ServoController::setMultiplePositionsUs(start, count, &servoPendingPulseUs_[start]);
        }
    }

    if (ServoController::hasI2CError()) {
        if (!servoI2CFaultLogged_) {
            DEBUG_LOG.println(F("[I2C] Servo controller communication error."));
            servoI2CFaultLogged_ = true;
        }
    } else {
        servoI2CFaultLogged_ = false;
    }

    pendingServoStatus_ = true;
}

void MessageCenter::processDeferredMagCal()
{
    DeferredMagCalAction action = deferredMagCalAction_;
    if (action == DEFER_MAG_NONE) {
        return;
    }

    deferredMagCalAction_ = DEFER_MAG_NONE;

    switch (action)
    {
    case DEFER_MAG_START:
        SensorManager::startMagCalibration();
        lastMagCalSendMs_ = 0;
        break;
    case DEFER_MAG_STOP:
        SensorManager::cancelMagCalibration();
        pendingMagCal_ = true;
        break;
    case DEFER_MAG_SAVE:
        SensorManager::saveMagCalibration();
        pendingMagCal_ = true;
        break;
    case DEFER_MAG_APPLY:
        SensorManager::applyMagCalibration(
            deferredMagCalOffsetX_, deferredMagCalOffsetY_, deferredMagCalOffsetZ_);
        pendingMagCal_ = true;
        break;
    case DEFER_MAG_CLEAR:
        SensorManager::clearMagCalibration();
        pendingMagCal_ = true;
        break;
    default:
        break;
    }
}

void MessageCenter::snapshotTrafficWindow(uint16_t &rxBytes,
                                          uint16_t &rxFrames,
                                          uint16_t &rxTlvs,
                                          uint16_t &rxHeartbeats,
                                          uint16_t &txBytes,
                                          uint16_t &txFrames)
{
    rxBytes = rxBytesWindow_;
    rxFrames = rxFramesWindow_;
    rxTlvs = rxTlvsWindow_;
    rxHeartbeats = rxHeartbeatsWindow_;
    txBytes = txBytesWindow_;
    txFrames = txFramesWindow_;

    rxBytesWindow_ = 0;
    rxFramesWindow_ = 0;
    rxTlvsWindow_ = 0;
    rxHeartbeatsWindow_ = 0;
    txBytesWindow_ = 0;
    txFramesWindow_ = 0;
}

void MessageCenter::sendTelemetry()
{
    if (txPendingOffset_ < txPendingLen_) {
        txDroppedFrames_++;
        return;
    }

    uint32_t currentMs = millis();
    const uint8_t phase = (uint8_t)((currentMs / 20U) % 10U);

    SystemState state = SystemManager::getState();
    bool running = (state == SYS_STATE_RUNNING);
    bool runningOrError = (state == SYS_STATE_RUNNING ||
                           state == SYS_STATE_ERROR);

    // Open a new frame; all send*() calls below append TLVs to it.
    beginFrame();

    // ---- System status first so the UI keeps receiving link/error state even
    // when the bring-up TX budget forces lower-priority telemetry to drop. ----
    uint32_t statusInterval = runningOrError ? TELEMETRY_SYS_STATUS_RUN_MS
                                             : TELEMETRY_SYS_STATUS_IDLE_MS;
    if (currentMs - lastStatusSendMs_ >= statusInterval && phase == 7U)
    {
        lastStatusSendMs_ = currentMs;
        sendSystemStatus();
    }

    // ---- Power telemetry early so UI battery / servo rails do not get
    // squeezed out by lower-priority RUNNING telemetry in the same frame. ----
    if (currentMs - lastVoltageSendMs_ >= TELEMETRY_VOLTAGE_MS && phase == 9U)
    {
        lastVoltageSendMs_ = currentMs;
        sendVoltageData();
    }

    // ---- Bring-up telemetry profile (RUNNING only) ----
    if (running)
    {
        if (pendingDCStatus_ ||
            (currentMs - lastDCStatusSendMs_ >= TELEMETRY_DC_STATUS_MS && phase == 0U))
        {
            lastDCStatusSendMs_ = currentMs;
            pendingDCStatus_ = false;
            sendDCStatusAll();
        }

        if (currentMs - lastStepStatusSendMs_ >= TELEMETRY_STEP_STATUS_MS && phase == 8U)
        {
            lastStepStatusSendMs_ = currentMs;
            sendStepStatusAll();
        }

        if (currentMs - lastKinematicsSendMs_ >= TELEMETRY_KINEMATICS_MS && phase == 2U)
        {
            lastKinematicsSendMs_ = currentMs;
            sendSensorKinematics();
        }

        if (currentMs - lastIOStatusSendMs_ >= TELEMETRY_IO_STATUS_MS && phase == 4U)
        {
            lastIOStatusSendMs_ = currentMs;
            sendIOStatus();
        }

        if (SensorManager::isIMUAvailable() &&
            currentMs - lastIMUSendMs_ >= TELEMETRY_IMU_MS &&
            phase == 6U)
        {
            lastIMUSendMs_ = currentMs;
            sendSensorIMU();
        }

        // Lidar range at 10 Hz (each configured slot, found or not)
        if (SensorManager::getLidarConfiguredCount() > 0 &&
            currentMs - lastLidarSendMs_ >= TELEMETRY_LIDAR_MS &&
            phase == 1U)
        {
            lastLidarSendMs_ = currentMs;
            sendSensorRange(true);
        }

    }

    // Servo status is needed in IDLE as well because the UI can enable and
    // position servos outside RUNNING. Also push an immediate status refresh
    // after servo commands so optimistic UI state is confirmed quickly.
    if (state != SYS_STATE_INIT &&
        (pendingServoStatus_ ||
         (currentMs - lastServoStatusSendMs_ >= TELEMETRY_SERVO_STATUS_MS &&
          phase == 5U)))
    {
        lastServoStatusSendMs_ = currentMs;
        pendingServoStatus_ = false;
        sendServoStatusAll();
    }

    // Ultrasonic range at 5 Hz (each configured slot, found or not)
    if (running && SensorManager::getUltrasonicConfiguredCount() > 0 &&
        currentMs - lastUltrasonicSendMs_ >= TELEMETRY_ULTRASONIC_MS &&
        phase == 3U)
    {
        lastUltrasonicSendMs_ = currentMs;
        sendSensorRange(false);
    }

    // ---- Mag cal status at 5 Hz while sampling ----
    if (SensorManager::getMagCalData().state == MAG_CAL_SAMPLING)
    {
        if (currentMs - lastMagCalSendMs_ >= 200 && phase == 1U)
        {
            lastMagCalSendMs_ = currentMs;
            sendMagCalStatus();
        }
    }

    // ---- Queued immediate mag cal response (STOP / SAVE / APPLY / CLEAR) ----
    if (pendingMagCal_)
    {
        pendingMagCal_ = false;
        sendMagCalStatus();
    }

    // Transmit the completed frame (no-ops if nothing was appended)
    sendFrame();
}

// ============================================================================
// LIVENESS MONITORING
// ============================================================================

bool MessageCenter::isHeartbeatValid()
{
    return heartbeatValid_;
}

uint32_t MessageCenter::getTimeSinceHeartbeat()
{
    return millis() - lastHeartbeatMs_;
}

void MessageCenter::checkHeartbeatTimeout()
{
    if (getTimeSinceHeartbeat() > (uint32_t)heartbeatTimeoutMs_)
    {
        if (heartbeatValid_) {
            DebugLog::printf_P(PSTR("[hb] timeout age=%lu ms state=%u\n"),
                               (unsigned long)getTimeSinceHeartbeat(),
                               (unsigned)SystemManager::getState());
        }
        heartbeatValid_ = false;
    }
}

// ============================================================================
// DECODE CALLBACK
// ============================================================================

void MessageCenter::decodeCallback(enum DecodeErrorCode *error,
                                   const struct FrameHeader *frameHeader,
                                   struct TlvHeader *tlvHeaders,
                                   uint8_t **tlvData)
{
    if (*error != NoError)
    {
        uartRxErrors_++;
        switch (*error)
        {
        case CrcError:
            crcErrorCount_++;
            break;
        case TotalPacketLenError:
        case BufferOutOfIndex:
        case UnpackFrameHeaderError:
            frameLenErrorCount_++;
            break;
        case TlvError:
        case TlvLenError:
            tlvErrorCount_++;
            break;
        default:
            break;
        }
#ifdef DEBUG_TLV_PACKETS
        DEBUG_LOG.print(F("[RX] Decode error: "));
        DEBUG_LOG.println((unsigned)*error);
#endif
        return;
    }

    // Route every TLV in the frame — supports multi-TLV incoming frames
    if (rxFramesWindow_ != 0xFFFFU) {
        rxFramesWindow_++;
    }
    uint32_t rxTlvsSum = (uint32_t)rxTlvsWindow_ + frameHeader->numTlvs;
    rxTlvsWindow_ = (rxTlvsSum > 0xFFFFUL) ? 0xFFFFU : (uint16_t)rxTlvsSum;
    for (uint32_t i = 0; i < frameHeader->numTlvs; i++)
    {
        uint32_t length = tlvHeaders[i].tlvLen;
        if (length > MAX_TLV_PAYLOAD_SIZE)
        {
            uartRxErrors_++;
            oversizeErrorCount_++;
#ifdef DEBUG_TLV_PACKETS
            DEBUG_LOG.print(F("[RX] Oversized payload, type="));
            DEBUG_LOG.println(tlvHeaders[i].tlvType);
#endif
            continue;
        }

        routeMessage(tlvHeaders[i].tlvType, tlvData[i], length);

#ifdef DEBUG_TLV_PACKETS
        DEBUG_LOG.print(F("[RX] Type: "));
        DEBUG_LOG.print(tlvHeaders[i].tlvType);
        DEBUG_LOG.print(F(", Len: "));
        DEBUG_LOG.println(length);
#endif
    }
}

// ============================================================================
// MESSAGE ROUTING
// ============================================================================

void MessageCenter::routeMessage(uint32_t type, const uint8_t *payload, uint32_t length)
{
    // Any received TLV resets the liveness timer
    bool wasHeartbeatValid = heartbeatValid_;
    uint32_t previousAgeMs = getTimeSinceHeartbeat();
    uint32_t nowMs = millis();
    lastHeartbeatMs_ = nowMs;
    heartbeatValid_ = true;
    if (!wasHeartbeatValid) {
        DebugLog::printf_P(PSTR("[hb] restored age=%lu ms\n"),
                           (unsigned long)previousAgeMs);
    }
    // Non-heartbeat commands update the command timer
    if (type != SYS_HEARTBEAT)
    {
        lastCmdMs_ = nowMs;
    }

    // ---- State-based command gating ----
    SystemState state = SystemManager::getState();
    // Motor commands only accepted in RUNNING state
    bool allowMotorCmds = (state == SYS_STATE_RUNNING);
    // Config only accepted in IDLE state
    bool allowConfig = (state == SYS_STATE_IDLE);
    // SYS_CMD and SYS_HEARTBEAT accepted in any state
    // SYS_SET_PID accepted in IDLE or RUNNING
    bool allowPID = (state == SYS_STATE_IDLE || state == SYS_STATE_RUNNING);

    switch (type)
    {
    // ---- System messages (always accepted) ----
    case SYS_HEARTBEAT:
        if (rxHeartbeatsWindow_ != 0xFFFFU) {
            rxHeartbeatsWindow_++;
        }
        if (length == sizeof(PayloadHeartbeat))
            handleHeartbeat((const PayloadHeartbeat *)payload);
        break;

    case SYS_CMD:
        if (length == sizeof(PayloadSysCmd))
            handleSysCmd((const PayloadSysCmd *)payload);
        break;

    case SYS_CONFIG:
        if (allowConfig && length == sizeof(PayloadSysConfig))
            handleSysConfig((const PayloadSysConfig *)payload);
        break;

    case SYS_SET_PID:
        if (allowPID && length == sizeof(PayloadSetPID))
            handleSetPID((const PayloadSetPID *)payload);
        break;

    // ---- DC motor commands (RUNNING only) ----
    case DC_ENABLE:
        if (length == sizeof(PayloadDCEnable))
            handleDCEnable((const PayloadDCEnable *)payload);
        break;

    case DC_SET_POSITION:
        if (allowMotorCmds && length == sizeof(PayloadDCSetPosition))
            handleDCSetPosition((const PayloadDCSetPosition *)payload);
        break;

    case DC_SET_VELOCITY:
        if (allowMotorCmds && length == sizeof(PayloadDCSetVelocity))
            handleDCSetVelocity((const PayloadDCSetVelocity *)payload);
        break;

    case DC_SET_PWM:
        if (allowMotorCmds && length == sizeof(PayloadDCSetPWM))
            handleDCSetPWM((const PayloadDCSetPWM *)payload);
        break;

    // ---- Stepper commands (RUNNING only) ----
    case STEP_ENABLE:
        if (length == sizeof(PayloadStepEnable))
            handleStepEnable((const PayloadStepEnable *)payload);
        break;

    case STEP_SET_PARAMS:
        if (allowMotorCmds && length == sizeof(PayloadStepSetParams))
            handleStepSetParams((const PayloadStepSetParams *)payload);
        break;

    case STEP_MOVE:
        if (allowMotorCmds && length == sizeof(PayloadStepMove))
            handleStepMove((const PayloadStepMove *)payload);
        break;

    case STEP_HOME:
        if (allowMotorCmds && length == sizeof(PayloadStepHome))
            handleStepHome((const PayloadStepHome *)payload);
        break;

    // ---- Servo commands ----
    case SERVO_ENABLE:
        if (length == sizeof(PayloadServoEnable))
            handleServoEnable((const PayloadServoEnable *)payload);
        break;

    case SERVO_SET:
        if (length == sizeof(PayloadServoSetSingle))
        {
            handleServoSet((const PayloadServoSetSingle *)payload);
        }
        else if (length == sizeof(PayloadServoSetBulk))
        {
            handleServoSetBulk((const PayloadServoSetBulk *)payload);
        }
        break;

    // ---- User I/O commands ----
    case IO_SET_LED:
        if (length == sizeof(PayloadSetLED))
            handleSetLED((const PayloadSetLED *)payload);
        break;

    case IO_SET_NEOPIXEL:
        if (length == sizeof(PayloadSetNeoPixel))
            handleSetNeoPixel((const PayloadSetNeoPixel *)payload);
        break;

    // ---- Magnetometer calibration (IDLE only) ----
    case SENSOR_MAG_CAL_CMD:
        if (allowConfig && length == sizeof(PayloadMagCalCmd))
            handleMagCalCmd((const PayloadMagCalCmd *)payload);
        break;

    default:
#ifdef DEBUG_TLV_PACKETS
        DEBUG_LOG.print(F("[RX] Unknown type: "));
        DEBUG_LOG.println(type);
#endif
        break;
    }
}

// ============================================================================
// SYSTEM MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleHeartbeat(const PayloadHeartbeat *payload)
{
    // Liveness timer already updated in routeMessage()
    (void)payload;
#ifdef DEBUG_TLV_PACKETS
    DEBUG_LOG.print(F("[RX] Heartbeat ts="));
    DEBUG_LOG.println(payload->timestamp);
#endif
}

void MessageCenter::handleSysCmd(const PayloadSysCmd *payload)
{
    switch ((SysCmdType)payload->command)
    {
    case SYS_CMD_START:
        if (SystemManager::requestTransition(SYS_STATE_RUNNING))
        {
            DebugLog::writeFlashLine(F("[SYS] CMD_START -> RUNNING OK"));
        }
        else
        {
            DebugLog::printf_P(PSTR("[SYS] CMD_START rejected state=%u\n"),
                               (unsigned)SystemManager::getState());
        }
        break;

    case SYS_CMD_STOP:
        if (SystemManager::requestTransition(SYS_STATE_IDLE))
        {
            disableAllActuators();
            deferredMagCalAction_ = DEFER_MAG_NONE;
            pendingMagCal_ = false;
            DebugLog::writeFlashLine(F("[SYS] CMD_STOP -> IDLE"));
        }
        break;

    case SYS_CMD_RESET:
        if (SystemManager::requestTransition(SYS_STATE_IDLE))
        {
            disableAllActuators();
            deferredMagCalAction_ = DEFER_MAG_NONE;
            pendingMagCal_ = false;
            faultLatchFlags_ = 0;   // clear after successful reset so UI shows clean state
            LoopMonitor::clearFaults();
            DebugLog::writeFlashLine(F("[SYS] CMD_RESET -> IDLE"));
        }
        else
        {
            DebugLog::printf_P(PSTR("[SYS] CMD_RESET rejected state=%u\n"),
                               (unsigned)SystemManager::getState());
        }
        break;

    case SYS_CMD_ESTOP:
        disableAllActuators();
        deferredMagCalAction_ = DEFER_MAG_NONE;
        pendingMagCal_ = false;
        SystemManager::requestTransition(SYS_STATE_ESTOP);
        DebugLog::writeFlashLine(F("[SYS] CMD_ESTOP -> ESTOP"));
        break;

    default:
        DebugLog::printf_P(PSTR("[SYS] unknown command=%u\n"),
                           (unsigned)payload->command);
        break;
    }
}

void MessageCenter::handleSysConfig(const PayloadSysConfig *payload)
{
    // IDLE state only — enforced by routeMessage()
    if (payload->neoPixelCount != 0)
        neoPixelCount_ = payload->neoPixelCount;

    if (payload->heartbeatTimeoutMs != 0)
        heartbeatTimeoutMs_ = payload->heartbeatTimeoutMs;

    // Apply direction mask changes (store; motors read this in sendSystemStatus)
    if (payload->motorDirChangeMask != 0)
    {
        motorDirMask_ = (motorDirMask_ & ~payload->motorDirChangeMask) |
                        (payload->motorDirMask & payload->motorDirChangeMask);
    }

    if (payload->resetOdometry)
    {
        RobotKinematics::reset(
            dcMotors[ODOM_LEFT_MOTOR].getPosition(),
            dcMotors[ODOM_RIGHT_MOTOR].getPosition());
    }
}

void MessageCenter::handleSetPID(const PayloadSetPID *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;

    DCMotor &motor = dcMotors[payload->motorId];

    if (payload->loopType == 0)
    {
        motor.setPositionPID(payload->kp, payload->ki, payload->kd);
    }
    else if (payload->loopType == 1)
    {
        motor.setVelocityPID(payload->kp, payload->ki, payload->kd);
    }
}

// ============================================================================
// DC MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleDCEnable(const PayloadDCEnable *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;

    DCMotor &motor = dcMotors[payload->motorId];

    if (payload->mode == DC_MODE_DISABLED)
    {
        motor.disable();   // disable always allowed regardless of state
        pendingDCStatus_ = true;
    }
    else
    {
        // Enable only permitted in IDLE or RUNNING — block in ERROR / ESTOP / INIT
        SystemState s = SystemManager::getState();
        if (s != SYS_STATE_IDLE && s != SYS_STATE_RUNNING) {
            DEBUG_LOG.println(F("[DC] Enable rejected: invalid state."));
            return;
        }
        // Silently block when no battery present — see VBAT_MIN_PRESENT_V in config.h
        if (!SensorManager::isBatteryPresent()) {
            DEBUG_LOG.println(F("[DC] Enable rejected: battery not present."));
            return;
        }
        motor.enable((DCMotorMode)payload->mode);
        pendingDCStatus_ = true;
    }
}

void MessageCenter::handleDCSetPosition(const PayloadDCSetPosition *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;
    if (!SensorManager::isBatteryPresent()) return;

    DCMotor &motor = dcMotors[payload->motorId];
    // Auto-switch to position mode. Only call enable() on mode change so
    // the PID is not reset unnecessarily when updating an in-flight target.
    if (motor.getMode() != DC_MODE_POSITION)
        motor.enable(DC_MODE_POSITION);

    motor.setTargetPosition(payload->targetTicks);
    pendingDCStatus_ = true;
}

void MessageCenter::handleDCSetVelocity(const PayloadDCSetVelocity *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;
    if (!SensorManager::isBatteryPresent()) return;

    DCMotor &motor = dcMotors[payload->motorId];
    if (motor.getMode() != DC_MODE_VELOCITY)
        motor.enable(DC_MODE_VELOCITY);

    motor.setTargetVelocity((float)payload->targetTicks);
    pendingDCStatus_ = true;
}

void MessageCenter::handleDCSetPWM(const PayloadDCSetPWM *payload)
{
    if (payload->motorId >= NUM_DC_MOTORS)
        return;
    if (!SensorManager::isBatteryPresent()) return;

    DCMotor &motor = dcMotors[payload->motorId];
    if (motor.getMode() != DC_MODE_PWM)
        motor.enable(DC_MODE_PWM);

    motor.setDirectPWM(payload->pwm);
    pendingDCStatus_ = true;
}

// ============================================================================
// STEPPER MOTOR MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleStepEnable(const PayloadStepEnable *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    if (payload->enable)
    {
        // Enable only permitted in IDLE or RUNNING
        SystemState st = SystemManager::getState();
        if (st != SYS_STATE_IDLE && st != SYS_STATE_RUNNING) return;
        if (!SensorManager::isBatteryPresent()) return;
        s->enable();
    }
    else
    {
        s->disable();   // disable always allowed
    }
}

void MessageCenter::handleStepSetParams(const PayloadStepSetParams *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    s->setMaxVelocity((uint16_t)payload->maxVelocity);
    s->setAcceleration((uint16_t)payload->acceleration);
}

void MessageCenter::handleStepMove(const PayloadStepMove *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    if (payload->moveType == STEP_MOVE_ABSOLUTE)
        s->moveToPosition(payload->target);
    else if (payload->moveType == STEP_MOVE_RELATIVE)
        s->moveSteps(payload->target);
}

void MessageCenter::handleStepHome(const PayloadStepHome *payload)
{
    if (payload->stepperId >= NUM_STEPPERS)
        return;

    StepperMotor *s = StepperManager::getStepper(payload->stepperId);
    if (!s) return;

    if (payload->homeVelocity > 0)
        s->setMaxVelocity((uint16_t)payload->homeVelocity);

    s->home(payload->direction);
}

// ============================================================================
// SERVO MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleServoEnable(const PayloadServoEnable *payload)
{
    // Enable only permitted in RUNNING; disable always allowed
    SystemState st = SystemManager::getState();
    bool stateOk = (st == SYS_STATE_RUNNING);
    bool powerOk = SensorManager::isBatteryPresent() || SensorManager::isServoRailPresent();
    bool canEnable = stateOk && powerOk;

    if (payload->channel == 0xFF)
    {
        // All channels
        if (payload->enable)
        {
            if (!canEnable) {
                DEBUG_LOG.println(F("[SERVO] Enable rejected: no actuator power or invalid state."));
                return;
            }
            servoEnabledMask_ = 0xFFFF;
            servoHardwareDirty_ = true;
            pendingServoStatus_ = true;
        }
        else
        {
            servoEnabledMask_ = 0;
            servoHardwareDirty_ = true;
            servoOffDirtyMask_ = 0xFFFF;
            pendingServoStatus_ = true;
        }
    }
    else if (payload->channel < NUM_SERVO_CHANNELS)
    {
        uint16_t bit = (uint16_t)(1u << payload->channel);
        if (payload->enable)
        {
            if (!canEnable) {
                DEBUG_LOG.println(F("[SERVO] Enable rejected: no actuator power or invalid state."));
                return;
            }
            servoEnabledMask_ |= bit;
            servoHardwareDirty_ = true;
            pendingServoStatus_ = true;
        }
        else
        {
            servoEnabledMask_ &= (uint16_t)~bit;
            servoHardwareDirty_ = true;
            servoOffDirtyMask_ |= bit;
            servoPulseDirtyMask_ &= (uint16_t)~bit;
            pendingServoStatus_ = true;
        }
    }
}

void MessageCenter::handleServoSet(const PayloadServoSetSingle *payload)
{
    if (payload->channel >= NUM_SERVO_CHANNELS)
        return;

    uint16_t bit = (uint16_t)(1u << payload->channel);
    servoPendingPulseUs_[payload->channel] = payload->pulseUs;
    servoPulseDirtyMask_ |= bit;
    pendingServoStatus_ = true;
}

void MessageCenter::handleServoSetBulk(const PayloadServoSetBulk *payload)
{
    if (payload->startChannel >= NUM_SERVO_CHANNELS || payload->count == 0) {
        return;
    }

    uint8_t count = payload->count;
    if ((uint16_t)payload->startChannel + count > NUM_SERVO_CHANNELS) {
        count = (uint8_t)(NUM_SERVO_CHANNELS - payload->startChannel);
    }

    for (uint8_t i = 0; i < count; ++i) {
        uint8_t channel = (uint8_t)(payload->startChannel + i);
        servoPendingPulseUs_[channel] = payload->pulseUs[i];
        servoPulseDirtyMask_ |= (uint16_t)(1u << channel);
    }
    pendingServoStatus_ = true;
}

// ============================================================================
// USER I/O MESSAGE HANDLERS
// ============================================================================

void MessageCenter::handleSetLED(const PayloadSetLED *payload)
{
    if (payload->ledId >= LED_COUNT)
        return;

    UserIO::setLED((LEDId)payload->ledId,
                   (LEDMode)payload->mode,
                   payload->brightness,
                   payload->periodMs);
}

void MessageCenter::handleSetNeoPixel(const PayloadSetNeoPixel *payload)
{
    if (payload->index == 0xFF)
    {
        // Set all pixels
        UserIO::setNeoPixelColor(payload->red, payload->green, payload->blue);
    }
    else if (payload->index < neoPixelCount_)
    {
        UserIO::setNeoPixelColor(payload->red, payload->green, payload->blue);
    }
}

// ============================================================================
// MAGNETOMETER CALIBRATION HANDLER
// ============================================================================

void MessageCenter::handleMagCalCmd(const PayloadMagCalCmd *payload)
{
    // IDLE state only — enforced by routeMessage()
    switch ((MagCalCmdType)payload->command)
    {
    case MAG_CAL_START:
        deferredMagCalAction_ = DEFER_MAG_START;
        break;

    case MAG_CAL_STOP:
        deferredMagCalAction_ = DEFER_MAG_STOP;
        break;

    case MAG_CAL_SAVE:
        deferredMagCalAction_ = DEFER_MAG_SAVE;
        break;

    case MAG_CAL_APPLY:
        deferredMagCalOffsetX_ = payload->offsetX;
        deferredMagCalOffsetY_ = payload->offsetY;
        deferredMagCalOffsetZ_ = payload->offsetZ;
        deferredMagCalAction_ = DEFER_MAG_APPLY;
        break;

    case MAG_CAL_CLEAR:
        deferredMagCalAction_ = DEFER_MAG_CLEAR;
        break;

    default:
        break;
    }
}

// ============================================================================
// TELEMETRY APPENDERS
// ============================================================================
// Each function appends one TLV to the current frame via addTlvPacket().
// Must be called after beginFrame() and before sendFrame().

void MessageCenter::sendDCStatusAll()
{
    PayloadDCStatusAll payload;

    for (uint8_t i = 0; i < 4; i++)
    {
        DCMotorStatus &s = payload.motors[i];
        s.mode = (uint8_t)dcMotors[i].getMode();
        s.faultFlags = dcMotors[i].isEncoderFailed() ? 0x02 : 0;
        s.position = dcMotors[i].getPosition();
        s.velocity = (int32_t)dcMotors[i].getVelocity();
        s.targetPos = dcMotors[i].getTargetPosition();
        s.targetVel = (int32_t)dcMotors[i].getTargetVelocity();
        s.pwmOutput = dcMotors[i].getPWMOutput();
        s.currentMa = dcMotors[i].getCurrent();
        s.posKp = dcMotors[i].getPosKp();
        s.posKi = dcMotors[i].getPosKi();
        s.posKd = dcMotors[i].getPosKd();
        s.velKp = dcMotors[i].getVelKp();
        s.velKi = dcMotors[i].getVelKi();
        s.velKd = dcMotors[i].getVelKd();
    }

    appendTlv(DC_STATUS_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendStepStatusAll()
{
    PayloadStepStatusAll payload;

    for (uint8_t i = 0; i < 4; i++)
    {
        StepperStatus &s = payload.steppers[i];
        const StepperMotor *sm = StepperManager::getStepper(i);
        if (!sm) continue;
        s.enabled = sm->isEnabled() ? 1 : 0;
        s.motionState = (uint8_t)sm->getState();
        s.limitHit = 0;
        s.reserved = 0;
        s.commandedCount = sm->getPosition();
        s.targetCount = sm->getTargetPosition();
        s.currentSpeed = sm->getCurrentSpeed();
        s.maxSpeed = sm->getMaxVelocity();
        s.acceleration = sm->getAcceleration();
    }

    appendTlv(STEP_STATUS_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendServoStatusAll()
{
    PayloadServoStatusAll payload;

    payload.pca9685Connected = ServoController::isInitialized() ? 1 : 0;
    payload.pca9685Error = ServoController::getLastI2CError();
    payload.enabledMask = servoEnabledMask_;

    for (uint8_t i = 0; i < 16; i++)
    {
        if (servoEnabledMask_ & (1u << i))
        {
            payload.pulseUs[i] = ServoController::getPositionUs(i);
        }
        else
        {
            payload.pulseUs[i] = 0;
        }
    }

    appendTlv(SERVO_STATUS_ALL, sizeof(payload), &payload);
}

void MessageCenter::sendSensorIMU()
{
    if (!SensorManager::isIMUAvailable())
        return;

    PayloadSensorIMU payload;

    SensorManager::getQuaternion(
        payload.quatW, payload.quatX, payload.quatY, payload.quatZ);
    SensorManager::getEarthAcceleration(
        payload.earthAccX, payload.earthAccY, payload.earthAccZ);

    payload.rawAccX = SensorManager::getRawAccX();
    payload.rawAccY = SensorManager::getRawAccY();
    payload.rawAccZ = SensorManager::getRawAccZ();
    payload.rawGyroX = SensorManager::getRawGyrX();
    payload.rawGyroY = SensorManager::getRawGyrY();
    payload.rawGyroZ = SensorManager::getRawGyrZ();
    payload.magX = SensorManager::getRawMagX();
    payload.magY = SensorManager::getRawMagY();
    payload.magZ = SensorManager::getRawMagZ();

    payload.magCalibrated = SensorManager::isMagCalibrated() ? 1 : 0;
    payload.reserved = 0;
    payload.timestamp = micros();

    appendTlv(SENSOR_IMU, sizeof(payload), &payload);
}

void MessageCenter::sendSensorRange(bool lidarOnly)
{
    if (lidarOnly)
    {
        // Lidar sensors (10 Hz path)
        for (uint8_t i = 0; i < SensorManager::getLidarConfiguredCount(); i++)
        {
            PayloadSensorRange p;
            p.sensorId = i;
            p.sensorType = 1; // lidar
            p.reserved = 0;
            p.reserved2 = 0;
            p.timestamp = micros();
            if (SensorManager::isLidarFound(i))
            {
                p.status = 0; // valid
                p.distanceMm = SensorManager::getLidarDistanceMm(i);
            }
            else
            {
                p.status = 3; // not installed
                p.distanceMm = 0;
            }
            if (!appendTlv(SENSOR_RANGE, sizeof(p), &p))
                break;
        }
    }
    else
    {
        // Ultrasonic sensors (10 Hz path)
        for (uint8_t i = 0; i < SensorManager::getUltrasonicConfiguredCount(); i++)
        {
            PayloadSensorRange p;
            p.sensorId = i;
            p.sensorType = 0; // ultrasonic
            p.reserved = 0;
            p.reserved2 = 0;
            p.timestamp = micros();
            if (SensorManager::isUltrasonicFound(i))
            {
                p.status = 0; // valid
                p.distanceMm = SensorManager::getUltrasonicDistanceMm(i);
            }
            else
            {
                p.status = 3; // not installed
                p.distanceMm = 0;
            }
            if (!appendTlv(SENSOR_RANGE, sizeof(p), &p))
                break;
        }
    }
}

void MessageCenter::sendSensorKinematics()
{
    RobotKinematics::update(
        dcMotors[ODOM_LEFT_MOTOR].getPosition(),
        dcMotors[ODOM_RIGHT_MOTOR].getPosition(),
        dcMotors[ODOM_LEFT_MOTOR].getVelocity(),
        dcMotors[ODOM_RIGHT_MOTOR].getVelocity());

    PayloadSensorKinematics payload;
    payload.x = RobotKinematics::getX();
    payload.y = RobotKinematics::getY();
    payload.theta = RobotKinematics::getTheta();
    payload.vx = RobotKinematics::getVx();
    payload.vy = RobotKinematics::getVy();
    payload.vTheta = RobotKinematics::getVTheta();
    payload.timestamp = micros();

    appendTlv(SENSOR_KINEMATICS, sizeof(payload), &payload);
}

void MessageCenter::sendVoltageData()
{
    PayloadSensorVoltage payload;

    payload.batteryMv = (uint16_t)(SensorManager::getBatteryVoltage() * 1000.0f);
    payload.rail5vMv = (uint16_t)(SensorManager::get5VRailVoltage() * 1000.0f);
    payload.servoRailMv = (uint16_t)(SensorManager::getServoVoltage() * 1000.0f);
    payload.reserved = 0;

    appendTlv(SENSOR_VOLTAGE, sizeof(payload), &payload);
}

void MessageCenter::sendIOStatus()
{
    // Variable-length payload: 12 bytes fixed + 3 bytes per NeoPixel
    uint8_t buf[12 + 3 * NEOPIXEL_COUNT];
    memset(buf, 0, sizeof(buf));

    PayloadIOStatus *p = (PayloadIOStatus *)buf;
    p->buttonMask = UserIO::getButtonStates();
    for (uint8_t i = 0; i < 5; i++) {
        p->ledBrightness[i] = UserIO::getLEDBrightness(i);
    }
    p->reserved = 0;
    p->timestamp = millis();
    // NeoPixel RGB bytes appended after fixed struct

    uint8_t sendLen = 12 + (uint8_t)(neoPixelCount_ * 3);
    if (sendLen > sizeof(buf))
        sendLen = sizeof(buf);

    appendTlv(IO_STATUS, sendLen, buf);
}

void MessageCenter::sendSystemStatus()
{
    PayloadSystemStatus payload;
    memset(&payload, 0, sizeof(payload));

    payload.firmwareMajor = (FIRMWARE_VERSION >> 24) & 0xFF;
    payload.firmwareMinor = (FIRMWARE_VERSION >> 16) & 0xFF;
    payload.firmwarePatch = (FIRMWARE_VERSION >> 8) & 0xFF;
    payload.state = (uint8_t)SystemManager::getState();
    payload.uptimeMs = millis();
    payload.lastRxMs = getTimeSinceHeartbeat();
    payload.lastCmdMs = millis() - lastCmdMs_;
    payload.batteryMv = (uint16_t)(SensorManager::getBatteryVoltage() * 1000.0f);
    payload.rail5vMv = (uint16_t)(SensorManager::get5VRailVoltage() * 1000.0f);

    // Error flags — live conditions OR latched fault cause
    payload.errorFlags = faultLatchFlags_;   // always show what triggered ERROR
    if (SensorManager::isBatteryLow())
        payload.errorFlags |= ERR_UNDERVOLTAGE;
    if (SensorManager::isBatteryOvervoltage())
        payload.errorFlags |= ERR_OVERVOLTAGE;
    // LIVENESS_LOST: live check only valid in RUNNING; latch covers the ERROR case
    if (!heartbeatValid_ && SystemManager::getState() == SYS_STATE_RUNNING)
        payload.errorFlags |= ERR_LIVENESS_LOST;
    if (ServoController::hasI2CError())
        payload.errorFlags |= ERR_I2C_ERROR;
#if IMU_ENABLED
    if (!SensorManager::isIMUAvailable())
        payload.errorFlags |= ERR_IMU_ERROR;
#endif
    if (LoopMonitor::getFaultMask() != 0)
        payload.errorFlags |= ERR_LOOP_OVERRUN;
    // Per-motor encoder stall: any failed motor sets the shared flag
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++) {
        if (dcMotors[i].isEncoderFailed()) {
            payload.errorFlags |= ERR_ENCODER_FAIL;
            break;
        }
    }
    // Attached sensors bitmask: bit0=IMU, bit1=Lidar, bit2=Ultrasonic
    if (SensorManager::isIMUAvailable())
        payload.attachedSensors |= 0x01;
    if (SensorManager::getLidarCount() > 0)
        payload.attachedSensors |= 0x02;
    if (SensorManager::getUltrasonicCount() > 0)
        payload.attachedSensors |= 0x04;

    payload.freeSram = getFreeRAM();
    payload.loopTimeAvgUs = loopTimeAvgUs_;
    payload.loopTimeMaxUs = loopTimeMaxUs_;
    payload.uartRxErrors = uartRxErrors_;
    payload.motorDirMask = motorDirMask_;
    payload.neoPixelCount = neoPixelCount_;
    payload.heartbeatTimeoutMs = heartbeatTimeoutMs_;
    payload.limitSwitchMask = 0;

    // 0xFF = no home limit GPIO configured for this stepper
    memset(payload.stepperHomeLimitGpio, 0xFF, sizeof(payload.stepperHomeLimitGpio));

    appendTlv(SYS_STATUS, sizeof(payload), &payload);
}

void MessageCenter::sendMagCalStatus()
{
    const MagCalData &cal = SensorManager::getMagCalData();

    PayloadMagCalStatus payload;
    payload.state = (uint8_t)cal.state;
    payload.sampleCount = cal.sampleCount;
    payload.reserved = 0;
    payload.minX = cal.minX;
    payload.maxX = cal.maxX;
    payload.minY = cal.minY;
    payload.maxY = cal.maxY;
    payload.minZ = cal.minZ;
    payload.maxZ = cal.maxZ;
    payload.offsetX = cal.offsetX;
    payload.offsetY = cal.offsetY;
    payload.offsetZ = cal.offsetZ;
    payload.savedToEeprom = cal.savedToEeprom ? 1 : 0;
    memset(payload.reserved2, 0, sizeof(payload.reserved2));

    appendTlv(SENSOR_MAG_CAL_STATUS, sizeof(payload), &payload);
}

// ============================================================================
// HELPERS
// ============================================================================

void MessageCenter::disableAllActuators()
{
    for (uint8_t i = 0; i < NUM_DC_MOTORS; i++)
    {
        if (dcMotors[i].isEnabled())
            dcMotors[i].disable();
    }
    pendingDCStatus_ = true;
    StepperManager::emergencyStopAll();
    ServoController::disable();
    servoEnabledMask_ = 0;
    servoHardwareDirty_ = false;
    servoPulseDirtyMask_ = 0;
    servoOffDirtyMask_ = 0;
    pendingServoStatus_ = true;
    servoUnavailableLogged_ = false;
    servoI2CFaultLogged_ = false;
}

void MessageCenter::latchFaultFlags(uint8_t flags)
{
    // OR-in the new flags so multiple faults accumulate.
    // Called from SafetyManager ISR — volatile write is atomic on AVR for uint8_t.
    faultLatchFlags_ |= flags;
}

uint16_t MessageCenter::getFreeRAM()
{
    extern int __heap_start, *__brkval;
    uint8_t v;
    const uint8_t *stackPtr = &v;
    const uint8_t *heapEnd = (__brkval == 0)
                                 ? (const uint8_t *)&__heap_start
                                 : (const uint8_t *)__brkval;
    if (stackPtr > heapEnd)
    {
        return (uint16_t)(stackPtr - heapEnd);
    }
    return 0;
}
