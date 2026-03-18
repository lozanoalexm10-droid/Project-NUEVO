/**
 * @file LoopMonitor.h
 * @brief Per-slot execution-time monitor for hard and soft real-time tasks
 *
 * Tracks wall-clock duration for each scheduled slot, maintains an exponential
 * moving average (α = 1/8), and latches a fault bit whenever a measurement
 * exceeds the slot's timing budget.
 *
 * PID timing note:
 *   DC control uses a mixed architecture:
 *   - TIMER1 runs a short 800 Hz round-robin slot that latches one motor's
 *     encoder count and applies one motor's cached H-bridge output.
 *   - taskMotors() runs the heavier 200 Hz PID/control compute in loop().
 *   We therefore track two different constraints:
 *   - SLOT_PID_ISR: one Timer1 motor slice against the UART-safe budget
 *   - PID round:    wall-clock span from the first motor slice to the last
 *                   motor slice in one 4-motor round
 *
 * Slots and budgets:
 *   SLOT_PID_ISR     800 Hz     80 µs  (TIMER1_OVF_vect, one motor slice)
 *   SLOT_STEPPER_ISR 10 kHz     60 µs  (TIMER3_OVF_vect)
 *   SLOT_MOTOR_TASK  200 Hz   5000 µs  (taskMotors())
 *   SLOT_SENSOR_ISR  100 Hz   8000 µs  (taskSensors(); includes I2C/ADC work)
 *   SLOT_UART_TASK    50 Hz  20000 µs  (soft scheduler; includes ISR preemption)
 *   SLOT_USERIO       20 Hz  40000 µs  (soft scheduler)
 *
 * ISR safety:
 *   record() writes only volatile fields. No heap allocation, no locking.
 *   Reads of 16-bit fields (avg/peak/max) are NOT guaranteed atomic on AVR.
 *   This is acceptable for the once-per-second diagnostic display.
 *
 * Usage:
 *   // In setup():
 *   LoopMonitor::init();
 *
 *   // At the top and bottom of each measured slot:
 *   uint32_t t0 = micros();
 *   ... work ...
 *   LoopMonitor::record(SLOT_PID_ISR, (uint16_t)(micros() - t0));
 *
 *   // In systemInfo() or sendSystemStatus():
 *   uint8_t faults = LoopMonitor::getFaultMask();
 *   LoopMonitor::report();   // prints table to DEBUG_SERIAL
 */

#ifndef LOOP_MONITOR_H
#define LOOP_MONITOR_H

#include <Arduino.h>
#include <stdint.h>

// ============================================================================
// SLOT IDS
// ============================================================================

enum LoopSlot : uint8_t {
    SLOT_PID_ISR     = 0,   ///< TIMER1 @ 800 Hz;  budget 80 µs (single motor slice)
    SLOT_STEPPER_ISR = 1,   ///< TIMER3 @ 10 kHz;  budget   60 µs
    SLOT_MOTOR_TASK  = 2,   ///< soft motor task  @ 100 Hz; budget 5000 µs
    SLOT_SENSOR_ISR  = 3,   ///< soft sensor task @ 100 Hz; budget 8000 µs
    SLOT_UART_TASK   = 4,   ///< soft UART task   @  50 Hz; budget 20000 µs
    SLOT_USERIO      = 5,   ///< soft   @  20 Hz;  budget 40000 µs
    LOOP_SLOT_COUNT  = 6
};

// Fault mask bit for each slot (bit N = slot N)
#define LOOP_FAULT_PID_ISR      (1u << SLOT_PID_ISR)
#define LOOP_FAULT_STEPPER_ISR  (1u << SLOT_STEPPER_ISR)
#define LOOP_FAULT_MOTOR_TASK   (1u << SLOT_MOTOR_TASK)
#define LOOP_FAULT_SENSOR_ISR   (1u << SLOT_SENSOR_ISR)
#define LOOP_FAULT_UART_TASK    (1u << SLOT_UART_TASK)
#define LOOP_FAULT_USERIO       (1u << SLOT_USERIO)
#define LOOP_FAULT_PID_ROUND    (1u << 6)

// ============================================================================
// LOOP MONITOR CLASS (static)
// ============================================================================

class LoopMonitor
{
public:
    /**
     * @brief Initialise all slots (zero stats, set budgets).
     * Must be called once from setup() before any ISRs fire.
     */
    static void init();

    /**
     * @brief Record a timing measurement for a slot (ISR-safe).
     *
     * Updates EMA average, peak, and all-time max.
     * Sets the fault-mask bit and increments the fault counter if us > budget.
     *
     * @param slot  Which slot to record for
     * @param us    Measured duration in microseconds
     * @return true if the measurement exceeded the budget
     */
    static bool record(LoopSlot slot, uint16_t us);

    /**
     * @brief Record one full TIMER1 round-robin span.
     *
     * Tracks the wall-clock span from the first motor slice to the last motor
     * slice in one 4-motor TIMER1 round. Call with active=false to ignore the
     * sample when control is idle.
     */
    static void recordPidRoundSpan(uint16_t spanUs, bool active);

    // ---- Fault reporting ------------------------------------------------

    /**
     * @brief Returns the OR of all slot fault bits that have fired since
     * the last clearFaults() call.  Bit N corresponds to LoopSlot N.
     * uint8_t is atomic on AVR — safe to read from non-ISR context.
     */
    static uint8_t getFaultMask() { return faultMask_; }

    /**
     * @brief Clear the cumulative fault-mask (call on CMD_RESET).
     * Does NOT reset faultCount per slot.
     */
    static void clearFaults() { faultMask_ = 0; }

    // ---- Per-slot accessors (non-ISR context only) ----------------------

    /** Timing budget µs for this slot */
    static uint16_t getBudgetUs(LoopSlot slot);

    /** EMA average µs for this slot (α = 1/8) */
    static uint16_t getAvgUs(LoopSlot slot);

    /**
     * @brief Peak µs since the last clearPeaks() call.
     * Useful for a rolling worst-case window (clear every report interval).
     */
    static uint16_t getPeakUs(LoopSlot slot);

    /** All-time maximum µs (never cleared automatically) */
    static uint16_t getMaxUs(LoopSlot slot);

    /** Cumulative overrun count for this slot (wraps at 65535) */
    static uint16_t getFaultCount(LoopSlot slot);

    /** Reset rolling-peak accumulators for all slots (call after report()) */
    static void clearPeaks();

    // ---- PID round accessors --------------------------------------------

    static uint16_t getPidRoundBudgetUs();
    static uint16_t getPidRoundAvgUs();
    static uint16_t getPidRoundPeakUs();
    static uint16_t getPidRoundMaxUs();
    static uint16_t getPidRoundFaultCount();

    // ---- Diagnostic output ----------------------------------------------

    /**
     * @brief Print a per-slot timing table to DEBUG_SERIAL.
     * Clears rolling peaks after printing.
     */
    static void report();

private:
    struct SlotData {
        uint16_t         budgetUs;    // Timing budget (µs)
        volatile uint16_t avgUs;      // EMA average (α = 1/8)
        volatile uint16_t peakUs;     // Rolling peak since clearPeaks()
        volatile uint16_t maxUs;      // All-time max
        volatile uint16_t faultCount; // Overrun count
        volatile bool     seeded;     // True after first sample
    };

    struct RoundData {
        uint16_t         budgetUs;       // Full-round timing budget (µs)
        volatile uint16_t avgUs;         // EMA average (α = 1/8)
        volatile uint16_t peakUs;        // Rolling peak since clearPeaks()
        volatile uint16_t maxUs;         // All-time max
        volatile uint16_t faultCount;    // Overrun count
        volatile bool     seeded;        // True after first full round
    };

    static SlotData         slots_[LOOP_SLOT_COUNT];
    static RoundData        pidRound_;
    static volatile uint8_t faultMask_; ///< OR of fault bits; atomic on AVR
};

#endif // LOOP_MONITOR_H
