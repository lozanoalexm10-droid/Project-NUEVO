/**
 * @file test_uart_loop_drain.ino
 * @brief Validate the 250 kbps loop-drain architecture before mainline integration
 *
 * Architecture under test:
 *   - TIMER3 @ 10 kHz: short stepper-style stub, no sei(), checks DOR2/FE2 first.
 *   - TIMER1 @ 800 Hz: one-motor-per-tick 16-bit control stub with an auto-escalating load ladder, no sei().
 *   - loop():
 *       1. Drain Serial2 RX continuously into the TLV decoder.
 *       2. Drain queued Serial2 TX by polling UDRE2 directly.
 *       3. Flush queued debug output using Serial.availableForWrite() only.
 *
 * Why this sketch exists:
 *   The production firmware currently mixes several architecture experiments.
 *   This sketch is the isolated proving ground for the new rules:
 *   short ISRs, loop-owned UART drain, non-blocking outbound UART, and no
 *   blocking debug prints once the timers are running.
 *
 * DOR2 detection note:
 *   DOR2 must be sampled inside TIMER3 before any delay. The core USART2_RX_vect
 *   clears it when it drains UDR2 into HardwareSerial's ring buffer. We only
 *   observe UCSR2A here; we do not read UDR2, so bytes remain owned by
 *   HardwareSerial.
 *
 * Build requirement:
 *   Add to platform.local.txt (AVR hardware directory):
 *     compiler.cpp.extra_flags=-DSERIAL_RX_BUFFER_SIZE=256
 *     compiler.c.extra_flags=-DSERIAL_RX_BUFFER_SIZE=256
 */

#include <Arduino.h>
#include <avr/io.h>
#include <stdio.h>
#include <string.h>
#include <util/delay.h>

extern "C" {
#include "src/lib/tlvcodec.h"
}

#include "src/config.h"
#include "src/messages/TLV_Payloads.h"
#include "src/messages/TLV_TypeDefs.h"

// ============================================================================
// Buffer size checks
// ============================================================================

#ifndef SERIAL_RX_BUFFER_SIZE
#error "SERIAL_RX_BUFFER_SIZE not defined. Add -DSERIAL_RX_BUFFER_SIZE=1024 to compiler flags."
#elif SERIAL_RX_BUFFER_SIZE < 256
#error "SERIAL_RX_BUFFER_SIZE < 256, which is recommanded for 250 kbps."
#endif

// ============================================================================
// Compile-time constants
// ============================================================================

#define STATUS_PERIOD_MS            1000UL
#define TX_STRESS_PERIOD_MS         10UL
#define LADDER_STAGE_DWELL_MS       10000UL

#define TIMER3_FREQ_HZ              10000UL
#define TIMER1_FREQ_HZ              800UL
#define TIMER3_STUB_BURN_US         1.0

#define TIMER1_FIXED_PID_TEST       1
#define PID_CHANNELS                4U
#define PID_OUTPUT_LIMIT            1024

#define DECODE_BUFFER_SIZE          256U
#define ENCODE_BUFFER_SIZE          272U
#define TX_PENDING_BUFFER_SIZE      272U
#define DEBUG_RING_SIZE             512U

static const uint8_t kTimer1ExtraOpsStages[] = { 0, 2, 4, 6, 8, 10, 12, 16, 20, 24, 28, 32 };

// ============================================================================
// ISR counters and timing (volatile: ISR writes, loop reads)
// ============================================================================

volatile uint32_t timer3Ticks      = 0;
volatile uint32_t timer1Ticks      = 0;
volatile uint16_t isr3MaxTicks     = 0;  // TCNT3 ticks elapsed in TIMER3 ISR (0.5 us/tick)
volatile uint16_t isr1MaxTicks     = 0;  // TCNT1 ticks elapsed in TIMER1 ISR
volatile uint32_t dor2Count        = 0;  // DOR2 rising edges
volatile uint32_t fe2Count         = 0;  // FE2 rising edges
volatile uint8_t  pidDiagMotor     = 0;
volatile int16_t  pidDiagOutput    = 0;
volatile int16_t  pidDiagVelocity  = 0;
volatile int16_t  pidDiagError     = 0;
volatile uint8_t  ladderStageActive = 0;
volatile uint8_t  ladderExtraOpsActive = 0;

// ============================================================================
// TLV decode state (loop context only)
// ============================================================================

static TlvDecodeDescriptor decodeDesc;
static uint32_t framesDecoded   = 0;
static uint32_t decodeErrors    = 0;
static uint32_t crcErrors       = 0;
static uint32_t pktLenErrors    = 0;
static uint32_t lastFrameNum    = 0;
static uint32_t bytesReceived   = 0;
static uint32_t lastHeartbeatMs = 0;
static uint32_t maxLoopGapUs    = 0;

// ============================================================================
// Non-blocking TX stress state (loop context only)
// ============================================================================

static TlvEncodeDescriptor encodeDesc;
static uint8_t txPending[TX_PENDING_BUFFER_SIZE];
static uint16_t txPendingLen      = 0;
static uint16_t txPendingOffset   = 0;
static uint32_t txFramesQueued    = 0;
static uint32_t txFramesSent      = 0;
static uint32_t txFramesBusy      = 0;
static uint32_t txEncodeErrors    = 0;
static uint32_t txBytesWritten    = 0;

// ============================================================================
// Timer1 load ladder state (loop owns policy, ISR consumes active stage)
// ============================================================================

static bool     ladderStarted          = false;
static bool     ladderFrozen           = false;
static bool     ladderCompleted        = false;
static uint8_t  ladderLastStableStage  = 0;
static uint8_t  ladderFailedStage      = 0xFF;
static uint32_t ladderStageStartMs     = 0;
static uint32_t ladderStageDorStart    = 0;
static uint32_t ladderStageErrStart    = 0;

// ============================================================================
// Buffered debug output state (loop context only)
// ============================================================================

static char dbgRing[DEBUG_RING_SIZE];
static uint16_t dbgHead       = 0;
static uint16_t dbgTail       = 0;
static uint32_t dbgDropped    = 0;

// ============================================================================
// Minimal 16-bit control stub state (TIMER1 ISR only)
// ============================================================================

static int16_t pidTarget[PID_CHANNELS]   = { 120, -85, 60, -40 };
static int16_t pidVelocity[PID_CHANNELS] = { 0, 0, 0, 0 };
static uint8_t pidMotorIndex           = 0;

// ============================================================================
// Helpers
// ============================================================================

static inline int16_t clampI16(int16_t value, int16_t lo, int16_t hi) {
    if (value < lo) return lo;
    if (value > hi) return hi;
    return value;
}

static inline void __attribute__((always_inline)) burnTimer3Stub() {
    _delay_us(TIMER3_STUB_BURN_US);
}

static inline uint16_t timerTicksElapsed(uint16_t start, uint16_t end, uint16_t top) {
    if (end >= start) return (uint16_t)(end - start);
    return (uint16_t)(end + (uint16_t)(top + 1U) - start);
}

static inline uint8_t ladderStageCount() {
    return (uint8_t)(sizeof(kTimer1ExtraOpsStages) / sizeof(kTimer1ExtraOpsStages[0]));
}

static inline uint32_t snapshotDor2Count() {
    uint32_t dor;
    noInterrupts();
    dor = dor2Count;
    interrupts();
    return dor;
}

static void dbgWrite(const char *text) {
    while (*text) {
        uint16_t next = (uint16_t)((dbgHead + 1U) % DEBUG_RING_SIZE);
        if (next == dbgTail) {
            dbgDropped++;
            break;
        }
        dbgRing[dbgHead] = *text++;
        dbgHead = next;
    }
}

static void dbgFlush() {
    while (dbgTail != dbgHead) {
        int space = DEBUG_SERIAL.availableForWrite();
        if (space <= 0) break;
        while (space-- > 0 && dbgTail != dbgHead) {
            DEBUG_SERIAL.write((uint8_t)dbgRing[dbgTail]);
            dbgTail = (uint16_t)((dbgTail + 1U) % DEBUG_RING_SIZE);
        }
    }
}

static void armLoadLadderStage(uint8_t stage, uint32_t nowMs, bool announce) {
    if (stage >= ladderStageCount()) {
        stage = (uint8_t)(ladderStageCount() - 1U);
    }

    ladderStageActive = stage;
    ladderExtraOpsActive = kTimer1ExtraOpsStages[stage];
    ladderStageStartMs = nowMs;
    ladderStageDorStart = snapshotDor2Count();
    ladderStageErrStart = decodeErrors;

    if (announce) {
        char line[96];
        snprintf(
            line,
            sizeof(line),
            "[LADDER] stage=%u extra_ops=%u dwell=%lums\n",
            (unsigned)stage,
            (unsigned)ladderExtraOpsActive,
            (unsigned long)LADDER_STAGE_DWELL_MS
        );
        dbgWrite(line);
    }
}

static void updateLoadLadder(uint32_t nowMs) {
    if (!ladderStarted) {
        if (framesDecoded == 0) return;
        ladderStarted = true;
        armLoadLadderStage(0, nowMs, true);
        return;
    }

    if (ladderFrozen) return;

    uint32_t dorNow = snapshotDor2Count();
    uint32_t errNow = decodeErrors;

    if (dorNow > ladderStageDorStart || errNow > ladderStageErrStart) {
        uint8_t failedStage = ladderStageActive;
        uint8_t revertStage = (failedStage == 0U) ? 0U : (uint8_t)(failedStage - 1U);
        uint32_t dorDelta = dorNow - ladderStageDorStart;
        uint32_t errDelta = errNow - ladderStageErrStart;

        ladderFailedStage = failedStage;
        ladderLastStableStage = revertStage;
        ladderFrozen = true;
        armLoadLadderStage(revertStage, nowMs, false);

        char line[128];
        snprintf(
            line,
            sizeof(line),
            "[LADDER] fail stage=%u revert=%u extra_ops=%u dor_delta=%lu err_delta=%lu\n",
            (unsigned)failedStage,
            (unsigned)revertStage,
            (unsigned)kTimer1ExtraOpsStages[failedStage],
            (unsigned long)dorDelta,
            (unsigned long)errDelta
        );
        dbgWrite(line);
        return;
    }

    if ((uint32_t)(nowMs - ladderStageStartMs) < LADDER_STAGE_DWELL_MS) return;

    ladderLastStableStage = ladderStageActive;
    if ((uint8_t)(ladderStageActive + 1U) >= ladderStageCount()) {
        ladderCompleted = true;
        ladderFrozen = true;
        dbgWrite("[LADDER] complete top stage held stable\n");
        return;
    }

    armLoadLadderStage((uint8_t)(ladderStageActive + 1U), nowMs, true);
}

static void drainTxStress() {
    if (txPendingOffset >= txPendingLen) return;

    // Poll the UART data register directly instead of using Serial2.write().
    // This keeps TX non-blocking without enabling USART2_UDRE_vect for every byte.
    if (bit_is_set(UCSR2A, UDRE2)) {
        UDR2 = txPending[txPendingOffset++];
        txBytesWritten++;
    }

    if (txPendingOffset >= txPendingLen) {
        txPendingLen = 0;
        txPendingOffset = 0;
        txFramesSent++;
    }
}

static void queueTxStressFrame(uint32_t nowMs) {
    if (txPendingOffset < txPendingLen) {
        txFramesBusy++;
        return;
    }

    int16_t velocitySnap[PID_CHANNELS];
    uint32_t timer1TickSnap;
    noInterrupts();
    timer1TickSnap = timer1Ticks;
    for (uint8_t i = 0; i < PID_CHANNELS; i++) {
        velocitySnap[i] = pidVelocity[i];
    }
    interrupts();

    PayloadDCStatusAll dcPayload;
    PayloadSystemStatus sysPayload;
    memset(&dcPayload, 0, sizeof(dcPayload));
    memset(&sysPayload, 0, sizeof(sysPayload));

    for (uint8_t i = 0; i < PID_CHANNELS; i++) {
        DCMotorStatus &motor = dcPayload.motors[i];
        motor.mode = DC_MODE_VELOCITY;
        motor.position = (int32_t)(timer1TickSnap * (uint32_t)(i + 1U));
        motor.velocity = (int32_t)velocitySnap[i] * 10L;
        motor.targetVel = (int32_t)pidTarget[i] * 10L;
        motor.pwmOutput = clampI16((int16_t)(velocitySnap[i] >> 1), -255, 255);
        motor.posKp = 0.0f;
        motor.posKi = 0.0f;
        motor.posKd = 0.0f;
        motor.velKp = 0.0f;
        motor.velKi = 0.0f;
        motor.velKd = 0.0f;
    }

    sysPayload.state = SYS_STATE_RUNNING;
    sysPayload.uptimeMs = nowMs;
    sysPayload.lastRxMs = (lastHeartbeatMs > 0) ? (nowMs - lastHeartbeatMs) : 0;
    sysPayload.lastCmdMs = sysPayload.lastRxMs;
    sysPayload.freeSram = 0;
    sysPayload.loopTimeAvgUs = 0;
    sysPayload.loopTimeMaxUs = 0;
    sysPayload.uartRxErrors = (uint16_t)(decodeErrors & 0xFFFFU);
    sysPayload.neoPixelCount = NEOPIXEL_COUNT;
    sysPayload.heartbeatTimeoutMs = HEARTBEAT_TIMEOUT_MS;
    memset(sysPayload.stepperHomeLimitGpio, 0xFF, sizeof(sysPayload.stepperHomeLimitGpio));

    resetDescriptor(&encodeDesc);
    addTlvPacket(&encodeDesc, DC_STATUS_ALL, sizeof(dcPayload), &dcPayload);
    addTlvPacket(&encodeDesc, SYS_STATUS, sizeof(sysPayload), &sysPayload);

    int frameLen = wrapupBuffer(&encodeDesc);
    if (frameLen <= 0 || frameLen > (int)sizeof(txPending)) {
        txEncodeErrors++;
        return;
    }

    memcpy(txPending, encodeDesc.buffer, (size_t)frameLen);
    txPendingLen = (uint16_t)frameLen;
    txPendingOffset = 0;
    txFramesQueued++;
}

// ============================================================================
// TLV decode callback (called from loop() context)
// ============================================================================

void onFrameDecoded(DecodeErrorCode *error,
                    const FrameHeader *frameHeader,
                    TlvHeader *tlvHeaders,
                    uint8_t **tlvData) {
    (void)tlvHeaders;
    (void)tlvData;

    if (*error != NoError) {
        decodeErrors++;
        if (*error == CrcError) crcErrors++;
        if (*error == TotalPacketLenError) pktLenErrors++;
        return;
    }

    framesDecoded++;
    lastFrameNum = frameHeader->frameNum;
    lastHeartbeatMs = millis();
}

// ============================================================================
// Timer setup
// ============================================================================

static void initTimer3() {
    TCCR3A = (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);
    ICR3 = (uint16_t)((F_CPU / (8UL * TIMER3_FREQ_HZ)) - 1UL);
    TCNT3 = 0;
    TIMSK3 = (1 << TOIE3);
}

static void initTimer1() {
    TCCR1A = (1 << WGM11);
    TCCR1B = (1 << WGM13) | (1 << WGM12) | (1 << CS11);
    ICR1 = (uint16_t)((F_CPU / (8UL * TIMER1_FREQ_HZ)) - 1UL);
    TCNT1 = 0;
    TIMSK1 = (1 << TOIE1);
}

// ============================================================================
// TIMER3: 10 kHz short stepper-style stub, plus DOR2/FE2 observation
// ============================================================================

ISR(TIMER3_OVF_vect) {
    static bool prevDor2 = false;
    static bool prevFe2  = false;

    uint8_t ucsr = UCSR2A;
    bool curDor = (ucsr & _BV(DOR2)) != 0;
    bool curFe  = (ucsr & _BV(FE2)) != 0;
    if (curDor && !prevDor2) dor2Count++;
    if (curFe && !prevFe2) fe2Count++;
    prevDor2 = curDor;
    prevFe2 = curFe;

    uint16_t t0 = TCNT3;
    timer3Ticks++;
    burnTimer3Stub();
    uint16_t elapsed = timerTicksElapsed(t0, TCNT3, ICR3);
    if (elapsed > isr3MaxTicks) isr3MaxTicks = elapsed;
}

// ============================================================================
// TIMER1: 800 Hz round-robin minimal 16-bit control stub
// ============================================================================

ISR(TIMER1_OVF_vect) {
    uint16_t t0 = TCNT1;
    timer1Ticks++;

#if TIMER1_FIXED_PID_TEST
    uint8_t motor = pidMotorIndex;
    uint8_t extraOps = ladderExtraOpsActive;
    int16_t err = (int16_t)(pidTarget[motor] - pidVelocity[motor]);
    int16_t out = clampI16(err, -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    int16_t work = out;

    for (uint8_t i = 0; i < extraOps; i++) {
        work = (int16_t)(work + (err >> 1) - (work >> 2));
        work = (int16_t)(work - (pidVelocity[motor] >> 2) + (work >> 3));
    }

    out = clampI16((int16_t)(out + (work >> 3)), -PID_OUTPUT_LIMIT, PID_OUTPUT_LIMIT);
    pidVelocity[motor] = (int16_t)(pidVelocity[motor] + ((out - pidVelocity[motor]) >> 2));

    pidDiagMotor = motor;
    pidDiagOutput = out;
    pidDiagVelocity = pidVelocity[motor];
    pidDiagError = err;

    pidMotorIndex = (uint8_t)((motor + 1U) % PID_CHANNELS);
#else
    burnTimer3Stub();
#endif

    uint16_t elapsed = timerTicksElapsed(t0, TCNT1, ICR1);
    if (elapsed > isr1MaxTicks) isr1MaxTicks = elapsed;
}

// ============================================================================
// Setup
// ============================================================================

void setup() {
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000) {}

    DEBUG_SERIAL.println(F("========================================="));
    DEBUG_SERIAL.println(F(" UART Loop-Drain Architecture Test v5"));
    DEBUG_SERIAL.println(F("========================================="));
    DEBUG_SERIAL.print(F(" Serial2 @ "));
    DEBUG_SERIAL.print(RPI_BAUD_RATE);
    DEBUG_SERIAL.println(F(" baud"));
#ifdef SERIAL_RX_BUFFER_SIZE
    DEBUG_SERIAL.print(F(" RX buf = "));
    DEBUG_SERIAL.println(SERIAL_RX_BUFFER_SIZE);
#else
    DEBUG_SERIAL.println(F(" RX buf = default (64)"));
#endif
    DEBUG_SERIAL.print(F(" TIMER3 @ "));
    DEBUG_SERIAL.print(TIMER3_FREQ_HZ);
    DEBUG_SERIAL.print(F(" Hz, TIMER1 @ "));
    DEBUG_SERIAL.print(TIMER1_FREQ_HZ);
    DEBUG_SERIAL.println(F(" Hz"));
    DEBUG_SERIAL.print(F(" TIMER3_STUB_BURN_US = "));
    DEBUG_SERIAL.println((float)TIMER3_STUB_BURN_US);
    DEBUG_SERIAL.println(F(" TIMER1 PID test = minimal 16-bit + auto load ladder"));
    DEBUG_SERIAL.print(F(" Ladder stages = "));
    DEBUG_SERIAL.print(ladderStageCount());
    DEBUG_SERIAL.print(F(", dwell = "));
    DEBUG_SERIAL.print(LADDER_STAGE_DWELL_MS);
    DEBUG_SERIAL.println(F(" ms"));
    DEBUG_SERIAL.println(F(" Runtime debug output is queued + flushed non-blocking"));
    DEBUG_SERIAL.println(F("========================================="));

    RPI_SERIAL.begin(RPI_BAUD_RATE);

    initDecodeDescriptor(&decodeDesc, DECODE_BUFFER_SIZE,
                         (ENABLE_CRC_CHECK ? true : false),
                         onFrameDecoded);
    initEncodeDescriptor(&encodeDesc, ENCODE_BUFFER_SIZE,
                         DEVICE_ID, (ENABLE_CRC_CHECK ? true : false));

    noInterrupts();
    initTimer3();
    initTimer1();
    interrupts();

    DEBUG_SERIAL.println(F("Timers started. Waiting for RPi traffic..."));
}

// ============================================================================
// Loop: RX drain first, then non-blocking TX and debug flush
// ============================================================================

void loop() {
    static uint32_t lastDrainUs = 0;
    static uint32_t lastStatusMs = 0;
    static uint32_t lastTxStressMs = 0;

    uint32_t nowUs = micros();
    if (lastDrainUs != 0) {
        uint32_t gap = nowUs - lastDrainUs;
        if (gap > maxLoopGapUs) maxLoopGapUs = gap;
    }
    lastDrainUs = nowUs;

    while (RPI_SERIAL.available()) {
        uint8_t b = (uint8_t)RPI_SERIAL.read();
        bytesReceived++;
        decodePacket(&decodeDesc, &b);
    }

    uint32_t nowMs = millis();
    updateLoadLadder(nowMs);

    if ((uint32_t)(nowMs - lastTxStressMs) >= TX_STRESS_PERIOD_MS) {
        lastTxStressMs = nowMs;
        queueTxStressFrame(nowMs);
    }

    drainTxStress();
    dbgFlush();

    if ((uint32_t)(nowMs - lastStatusMs) >= STATUS_PERIOD_MS) {
        lastStatusMs = nowMs;

        uint32_t t3, t1, dor, fe;
        uint16_t m3, m1;
        uint8_t pidMotor;
        uint8_t ladderStage;
        uint8_t ladderOps;
        int16_t pidOut, pidVel, pidErr;
        noInterrupts();
        t3 = timer3Ticks;
        t1 = timer1Ticks;
        dor = dor2Count;
        fe = fe2Count;
        m3 = isr3MaxTicks;
        m1 = isr1MaxTicks;
        isr3MaxTicks = 0;
        isr1MaxTicks = 0;
        pidMotor = pidDiagMotor;
        ladderStage = ladderStageActive;
        ladderOps = ladderExtraOpsActive;
        pidOut = pidDiagOutput;
        pidVel = pidDiagVelocity;
        pidErr = pidDiagError;
        interrupts();

        uint32_t hbAge = (lastHeartbeatMs > 0) ? (nowMs - lastHeartbeatMs) : 0;
        int16_t ladderFail = (ladderFailedStage == 0xFF) ? -1 : (int16_t)ladderFailedStage;
        int16_t pidOut10 = (int16_t)(pidOut * 10);
        int16_t pidVel10 = (int16_t)(pidVel * 10);
        int16_t pidErr10 = (int16_t)(pidErr * 10);

        char line[384];
        snprintf(
            line,
            sizeof(line),
            "[STAT] rx_frames=%lu errs=%lu(pkt=%lu,crc=%lu) dor=%lu fe=%lu rx=%luB "
            "tx_q=%lu tx_sent=%lu tx_busy=%lu tx_err=%lu tx_bytes=%lu dbg_drop=%lu "
            "hb=%s gap=%luus isr3=%uu isr1=%uu ladder[s=%u ops=%u ok=%u fail=%d %s] "
            "pid[m%u o=%d v=%d e=%d] last=%lu t3=%lu t1=%lu\n",
            (unsigned long)framesDecoded,
            (unsigned long)decodeErrors,
            (unsigned long)pktLenErrors,
            (unsigned long)crcErrors,
            (unsigned long)dor,
            (unsigned long)fe,
            (unsigned long)bytesReceived,
            (unsigned long)txFramesQueued,
            (unsigned long)txFramesSent,
            (unsigned long)txFramesBusy,
            (unsigned long)txEncodeErrors,
            (unsigned long)txBytesWritten,
            (unsigned long)dbgDropped,
            (lastHeartbeatMs == 0) ? "NONE" : ((hbAge < HEARTBEAT_TIMEOUT_MS) ? "OK" : "LOST"),
            (unsigned long)maxLoopGapUs,
            (unsigned)m3 / 2U,
            (unsigned)m1 / 2U,
            (unsigned)ladderStage,
            (unsigned)ladderOps,
            (unsigned)ladderLastStableStage,
            (int)ladderFail,
            ladderCompleted ? "done" : (ladderFrozen ? "held" : (ladderStarted ? "ramp" : "wait")),
            (unsigned)(pidMotor + 1U),
            (int)pidOut10,
            (int)pidVel10,
            (int)pidErr10,
            (unsigned long)lastFrameNum,
            (unsigned long)t3,
            (unsigned long)t1
        );
        dbgWrite(line);
        maxLoopGapUs = 0;
    }
}
