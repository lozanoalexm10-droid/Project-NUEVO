/**
 * @file test_uart_rx_10khz.ino
 * @brief UART RX stress test — 10 kHz drain + 100 Hz decode (MessageCenter)
 *
 * Purpose:
 *   Replicates the current RX architecture used in firmware:
 *     - TIMER3 @ 10 kHz: drain UART HW buffer into MessageCenter ring buffer
 *     - TIMER4 @ 100 Hz: run MessageCenter::uartTask() (decode + heartbeat)
 *
 * This sketch is intended to debug missing bytes / framing errors from the RPi.
 *
 * How to use:
 *   1) Create the src symlink (see firmware/tests/README.md)
 *   2) Upload this sketch
 *   3) On the RPi, send heartbeat frames (same as test_uart_arduino.py)
 *   4) Watch USB Serial @ 115200 for liveness + error counters
 *
 * Notes:
 *   - DEBUG_TLV_PACKETS in MessageCenter.cpp prints every RX byte. That output
 *     can be very heavy inside ISR context. If you want clean timing, disable it.
 */

#include <Arduino.h>
#include <avr/io.h>
#include "src/config.h"
#include "src/modules/MessageCenter.h"
#include "src/drivers/DCMotor.h"
#include "src/drivers/StepperMotor.h"

// Required by MessageCenter (extern references)
DCMotor      dcMotors[NUM_DC_MOTORS];
StepperMotor steppers[NUM_STEPPERS];

// Status print period on USB serial
#define TEST_STATUS_PERIOD_MS             1000

// ---------------------------------------------------------------------------
// Timer helpers (match firmware timing)
// ---------------------------------------------------------------------------

static void initTimer3_10kHz() {
    // Fast PWM mode 14, prescaler=8 → 2 MHz timer clock
    // ICR3 = (2,000,000 / 10,000) - 1 = 199
    TCCR3A = (1 << WGM31);
    TCCR3B = (1 << WGM33) | (1 << WGM32) | (1 << CS31);
    ICR3   = (uint16_t)((F_CPU / (8UL * STEPPER_TIMER_FREQ_HZ)) - 1);
    TCNT3  = 0;
    TIMSK3 = (1 << TOIE3);  // overflow interrupt
}

static void initTimer4_10kHz() {
    // Fast PWM mode 14, prescaler=8 → 2 MHz timer clock
    // ICR4 = (2,000,000 / 10,000) - 1 = 199
    TCCR4A = (1 << WGM41);
    TCCR4B = (1 << WGM43) | (1 << WGM42) | (1 << CS41);
    ICR4   = (uint16_t)((F_CPU / (8UL * STEPPER_TIMER_FREQ_HZ)) - 1);
    TCNT4  = 0;
    TIMSK4 = (1 << TOIE4);  // overflow interrupt
}

// ---------------------------------------------------------------------------
// ISR Counters / Flags
// ---------------------------------------------------------------------------

volatile uint32_t timer3Ticks = 0;       // 10 kHz tick count
volatile uint32_t timer4Ticks = 0;       // 100 Hz tick count (after /100 divider)
volatile uint32_t uartTaskRuns = 0;      // how many decode cycles executed
volatile uint32_t dor2Count = 0;         // UART2 data overrun count
volatile uint32_t fe2Count = 0;          // UART2 framing error count

// ---------------------------------------------------------------------------
// TIMER3 — 10 kHz drain
// ---------------------------------------------------------------------------

ISR(TIMER3_OVF_vect) {
    timer3Ticks++;

    TIMSK3 &= ~(1 << TOIE3);  // block Timer3 re-entry

    uint8_t status = UCSR2A;
    if (status & _BV(DOR2)) dor2Count++;
    if (status & _BV(FE2))  fe2Count++;

    sei();  // allow UART RX ISR to preempt

    MessageCenter::drainUart();

    TIMSK3 |= (1 << TOIE3);
}

// ---------------------------------------------------------------------------
// TIMER4 — 10 kHz carrier, divide to 100 Hz decode tick
// ---------------------------------------------------------------------------

ISR(TIMER4_OVF_vect) {
    static uint8_t counter = 0;
    if (++counter < 100) return;  // 10 kHz / 100 = 100 Hz
    counter = 0;
    timer4Ticks++;

    sei();  // allow TIMER3 / UART RX to preempt
    MessageCenter::uartTask();
    uartTaskRuns++;
}

// ---------------------------------------------------------------------------
// Setup / Loop
// ---------------------------------------------------------------------------

void setup() {
    DEBUG_SERIAL.begin(DEBUG_BAUD_RATE);
    while (!DEBUG_SERIAL && millis() < 2000) {}

    DEBUG_SERIAL.println(F("================================"));
    DEBUG_SERIAL.println(F(" UART RX 10kHz Drain Test"));
    DEBUG_SERIAL.println(F("================================"));
    DEBUG_SERIAL.print(F(" Serial2 @ "));
    DEBUG_SERIAL.print(RPI_BAUD_RATE);
    DEBUG_SERIAL.println(F(" baud"));
    DEBUG_SERIAL.println(F("================================"));

    // Init MessageCenter before enabling timers
    MessageCenter::init();

    noInterrupts();

    initTimer3_10kHz();
    initTimer4_10kHz();
    interrupts();
}

void loop() {
    static uint32_t lastStatusMs = 0;
    uint32_t now = millis();
    if (now - lastStatusMs >= TEST_STATUS_PERIOD_MS) {
        lastStatusMs = now;

        uint32_t t3, t4, runs, dor, fe;
        noInterrupts();
        t3 = timer3Ticks;
        t4 = timer4Ticks;
        runs = uartTaskRuns;
        dor = dor2Count;
        fe  = fe2Count;
        interrupts();

        DEBUG_SERIAL.print(F("[UART] hb="));
        DEBUG_SERIAL.print(MessageCenter::isHeartbeatValid() ? F("OK") : F("LOST"));
        DEBUG_SERIAL.print(F(" last="));
        DEBUG_SERIAL.print(MessageCenter::getTimeSinceHeartbeat());
        DEBUG_SERIAL.print(F("ms rxErr="));
        DEBUG_SERIAL.print(MessageCenter::getUartRxErrors());
        DEBUG_SERIAL.print(F(" dor="));
        DEBUG_SERIAL.print(dor);
        DEBUG_SERIAL.print(F(" fe="));
        DEBUG_SERIAL.print(fe);
        DEBUG_SERIAL.print(F(" t3="));
        DEBUG_SERIAL.print(t3);
        DEBUG_SERIAL.print(F(" t4="));
        DEBUG_SERIAL.print(t4);
        DEBUG_SERIAL.print(F(" task="));
        DEBUG_SERIAL.println(runs);
    }
}
