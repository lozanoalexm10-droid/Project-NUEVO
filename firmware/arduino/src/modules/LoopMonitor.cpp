/**
 * @file LoopMonitor.cpp
 * @brief Per-slot execution-time monitor implementation
 */

#include "LoopMonitor.h"
#include "../config.h"

// ============================================================================
// SLOT BUDGETS (µs)
// ============================================================================

static const uint16_t kBudgets[LOOP_SLOT_COUNT] = {
       PID_ISR_UART_BUDGET_US,      // SLOT_PID_ISR     (single ISR slice)
       STEPPER_ISR_UART_BUDGET_US,  // SLOT_STEPPER_ISR (10kHz, 100µs period)
     5000,  // SLOT_MOTOR_TASK  (200Hz soft task)
     8000,  // SLOT_SENSOR_ISR  (100Hz soft task; includes I2C/ADC time)
    20000,  // SLOT_UART_TASK   (50Hz, 20ms period; includes ISR preemption)
    40000,  // SLOT_USERIO      (20Hz,  50ms period)
};

// ============================================================================
// STATIC MEMBER DEFINITIONS
// ============================================================================

LoopMonitor::SlotData    LoopMonitor::slots_[LOOP_SLOT_COUNT] = {};
LoopMonitor::RoundData   LoopMonitor::pidRound_               = {};
volatile uint8_t         LoopMonitor::faultMask_              = 0;

// ============================================================================
// PUBLIC METHODS
// ============================================================================

void LoopMonitor::init()
{
    for (uint8_t i = 0; i < LOOP_SLOT_COUNT; i++) {
        slots_[i].budgetUs   = kBudgets[i];
        slots_[i].avgUs      = 0;
        slots_[i].peakUs     = 0;
        slots_[i].maxUs      = 0;
        slots_[i].faultCount = 0;
        slots_[i].seeded     = false;
    }

    pidRound_.budgetUs = (uint16_t)PID_ROUND_BUDGET_US;
    pidRound_.avgUs = 0;
    pidRound_.peakUs = 0;
    pidRound_.maxUs = 0;
    pidRound_.faultCount = 0;
    pidRound_.seeded = false;

    faultMask_ = 0;
}

bool LoopMonitor::record(LoopSlot slot, uint16_t us)
{
    SlotData &s = slots_[slot];

    // Seed EMA with the first real sample to avoid a long initial transient
    if (!s.seeded) {
        s.avgUs  = us;
        s.seeded = true;
    } else {
        // Integer EMA:  new = old - floor(old/8) + floor(val/8)
        // Equivalent to α=1/8 filter with fixed-point quantisation.
        s.avgUs = (uint16_t)(s.avgUs - (s.avgUs >> 3) + (us >> 3));
    }

    // Rolling peak and all-time max
    if (us > s.peakUs) s.peakUs = us;
    if (us > s.maxUs)  s.maxUs  = us;

    // Overrun detection
    bool overrun = (us > s.budgetUs);
    if (overrun) {
        if (s.faultCount < 0xFFFF) s.faultCount++;
        faultMask_ |= (uint8_t)(1u << slot);
    }
    return overrun;
}

void LoopMonitor::recordPidRoundSpan(uint16_t spanUs, bool active)
{
    if (!active) {
        return;
    }

    uint16_t roundUs = spanUs;

    if (!pidRound_.seeded) {
        pidRound_.avgUs = roundUs;
        pidRound_.seeded = true;
    } else {
        pidRound_.avgUs = (uint16_t)(pidRound_.avgUs - (pidRound_.avgUs >> 3) + (roundUs >> 3));
    }

    if (roundUs > pidRound_.peakUs) pidRound_.peakUs = roundUs;
    if (roundUs > pidRound_.maxUs)  pidRound_.maxUs  = roundUs;

    if (roundUs > pidRound_.budgetUs) {
        if (pidRound_.faultCount < 0xFFFF) pidRound_.faultCount++;
        faultMask_ |= LOOP_FAULT_PID_ROUND;
    }
}

uint16_t LoopMonitor::getAvgUs(LoopSlot slot)    { return slots_[slot].avgUs;      }
uint16_t LoopMonitor::getBudgetUs(LoopSlot slot) { return slots_[slot].budgetUs;   }
uint16_t LoopMonitor::getPeakUs(LoopSlot slot)   { return slots_[slot].peakUs;     }
uint16_t LoopMonitor::getMaxUs(LoopSlot slot)    { return slots_[slot].maxUs;      }
uint16_t LoopMonitor::getFaultCount(LoopSlot slot){ return slots_[slot].faultCount; }
uint16_t LoopMonitor::getPidRoundBudgetUs()      { return pidRound_.budgetUs;       }
uint16_t LoopMonitor::getPidRoundAvgUs()         { return pidRound_.avgUs;          }
uint16_t LoopMonitor::getPidRoundPeakUs()        { return pidRound_.peakUs;         }
uint16_t LoopMonitor::getPidRoundMaxUs()         { return pidRound_.maxUs;          }
uint16_t LoopMonitor::getPidRoundFaultCount()    { return pidRound_.faultCount;     }

void LoopMonitor::clearPeaks()
{
    for (uint8_t i = 0; i < LOOP_SLOT_COUNT; i++) {
        slots_[i].peakUs = 0;
    }
    pidRound_.peakUs = 0;
}

// ============================================================================
// REPORT
// ============================================================================

void LoopMonitor::report()
{
    // Slot label widths are padded to align the table columns.
    static const __FlashStringHelper * const kNames[LOOP_SLOT_COUNT] = {
        F("PID_ISR    "),
        F("STEPPER_ISR"),
        F("MOTOR_TASK "),
        F("SENSOR_ISR "),
        F("UART_TASK  "),
        F("USERIO     "),
    };

    DEBUG_SERIAL.println(F("[timing] slot          budget    avg   peak    max  faults"));
    for (uint8_t i = 0; i < LOOP_SLOT_COUNT; i++) {
        const SlotData &s = slots_[i];
        bool overrun = (faultMask_ >> i) & 1u;

        DEBUG_SERIAL.print(F("[timing] "));
        DEBUG_SERIAL.print(kNames[i]);
        DEBUG_SERIAL.print(F("  "));
        DEBUG_SERIAL.print(s.budgetUs);
        DEBUG_SERIAL.print(F("us  "));
        DEBUG_SERIAL.print(s.avgUs);
        DEBUG_SERIAL.print(F("  "));
        DEBUG_SERIAL.print(s.peakUs);
        DEBUG_SERIAL.print(F("  "));
        DEBUG_SERIAL.print(s.maxUs);
        DEBUG_SERIAL.print(F("  "));
        DEBUG_SERIAL.print(s.faultCount);
        if (overrun) DEBUG_SERIAL.print(F("  OVERRUN!"));
        DEBUG_SERIAL.println();
    }

    DEBUG_SERIAL.print(F("[timing] PID_ROUND    "));
    DEBUG_SERIAL.print(pidRound_.budgetUs);
    DEBUG_SERIAL.print(F("us  "));
    DEBUG_SERIAL.print(pidRound_.avgUs);
    DEBUG_SERIAL.print(F("  "));
    DEBUG_SERIAL.print(pidRound_.peakUs);
    DEBUG_SERIAL.print(F("  "));
    DEBUG_SERIAL.print(pidRound_.maxUs);
    DEBUG_SERIAL.print(F("  "));
    DEBUG_SERIAL.print(pidRound_.faultCount);
    if ((faultMask_ & LOOP_FAULT_PID_ROUND) != 0) DEBUG_SERIAL.print(F("  OVERRUN!"));
    DEBUG_SERIAL.println();

    clearPeaks();
}
