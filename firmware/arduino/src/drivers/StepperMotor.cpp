/**
 * @file StepperMotor.cpp
 * @brief Implementation of stepper motor driver with acceleration
 */

#include "StepperMotor.h"
#include "../config.h"
#include <util/atomic.h>

#define TIMER_FREQ_HZ   STEPPER_TIMER_FREQ_HZ

namespace {
constexpr uint16_t kMinStartSpeedSps = 100U;
constexpr uint16_t kMinStopSpeedSps = 10U;
constexpr uint32_t kQ16Shift = 16U;
} // namespace

// ============================================================================
// CONSTRUCTOR AND INITIALIZATION
// ============================================================================

StepperMotor::StepperMotor()
    : stepperId_(0)
    , pinStep_(0)
    , pinDir_(0)
    , pinEnable_(0)
    , pinLimit_(0)
    , limitActiveState_(LOW)
    , hasLimit_(false)
    , enabled_(false)
    , stepOutReg_(nullptr)
    , dirOutReg_(nullptr)
    , enableOutReg_(nullptr)
    , limitInReg_(nullptr)
    , stepMask_(0)
    , dirMask_(0)
    , enableMask_(0)
    , limitMask_(0)
    , maxVelocity_(1000)
    , acceleration_(500)
    , state_(STEPPER_IDLE)
    , currentPosition_(0)
    , targetPosition_(0)
    , stepsRemaining_(0)
    , direction_(1)
    , stepInterval_(0)
    , stepCounter_(0)
    , minInterval_(0)
    , startInterval_(0)
    , intervalQ16_(0)
    , accelDeltaQ16_(0)
    , decelDeltaQ16_(0)
    , accelSteps_(0)
    , decelSteps_(0)
    , cruiseSteps_(0)
    , stepCount_(0)
    , currentVelocity_(0)
{
}

void StepperMotor::init(uint8_t stepperId) {
    stepperId_ = stepperId;
    state_ = STEPPER_IDLE;
    currentPosition_ = 0;
    targetPosition_ = 0;
    stepsRemaining_ = 0;
    currentVelocity_ = 0;

    minInterval_ = TIMER_FREQ_HZ / maxVelocity_;
    if (minInterval_ < 1) minInterval_ = 1;
}

void StepperMotor::setPins(uint8_t pinStep, uint8_t pinDir, uint8_t pinEnable) {
    pinStep_ = pinStep;
    pinDir_ = pinDir;
    pinEnable_ = pinEnable;

    pinMode(pinStep_, OUTPUT);
    pinMode(pinDir_, OUTPUT);
    pinMode(pinEnable_, OUTPUT);

    digitalWrite(pinStep_, LOW);
    digitalWrite(pinDir_, LOW);
    digitalWrite(pinEnable_, HIGH);

    stepOutReg_ = portOutputRegister(digitalPinToPort(pinStep_));
    dirOutReg_ = portOutputRegister(digitalPinToPort(pinDir_));
    enableOutReg_ = portOutputRegister(digitalPinToPort(pinEnable_));
    stepMask_ = digitalPinToBitMask(pinStep_);
    dirMask_ = digitalPinToBitMask(pinDir_);
    enableMask_ = digitalPinToBitMask(pinEnable_);
}

void StepperMotor::setLimitPin(uint8_t pinLimit, uint8_t activeState) {
    pinLimit_ = pinLimit;
    limitActiveState_ = activeState;
    hasLimit_ = true;

    if (activeState == LOW) {
        pinMode(pinLimit_, INPUT_PULLUP);
    } else {
        pinMode(pinLimit_, INPUT);
    }

    limitInReg_ = portInputRegister(digitalPinToPort(pinLimit_));
    limitMask_ = digitalPinToBitMask(pinLimit_);
}

void StepperMotor::enable() {
    enabled_ = true;
    if (enableOutReg_) {
        *enableOutReg_ &= (uint8_t)~enableMask_;
    } else {
        digitalWrite(pinEnable_, LOW);
    }

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.println(F("] Enabled"));
#endif
}

void StepperMotor::disable() {
    enabled_ = false;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_ = STEPPER_IDLE;
        currentVelocity_ = 0;
        stepsRemaining_ = 0;
    }
    if (enableOutReg_) {
        *enableOutReg_ |= enableMask_;
    } else {
        digitalWrite(pinEnable_, HIGH);
    }

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.println(F("] Disabled"));
#endif
}

// ============================================================================
// MOTION PARAMETERS
// ============================================================================

void StepperMotor::setMaxVelocity(uint16_t stepsPerSec) {
    uint16_t maxVelocity = stepsPerSec;
    if (maxVelocity > STEPPER_MAX_RATE_SPS) {
        maxVelocity = STEPPER_MAX_RATE_SPS;
    }

    uint16_t minInterval = TIMER_FREQ_HZ / maxVelocity;
    if (minInterval < 1) minInterval = 1;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        maxVelocity_ = maxVelocity;
        minInterval_ = minInterval;
    }
}

void StepperMotor::setAcceleration(uint16_t stepsPerSecSq) {
    if (stepsPerSecSq < 1) stepsPerSecSq = 1;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        acceleration_ = stepsPerSecSq;
    }
}

// ============================================================================
// MOTION COMMANDS
// ============================================================================

void StepperMotor::moveSteps(int32_t steps) {
    if (steps == 0 || !enabled_) return;

    int8_t direction = (steps > 0) ? 1 : -1;
    uint16_t maxVelocity = maxVelocity_;
    uint16_t minInterval = minInterval_;
    uint16_t acceleration = acceleration_;

    int32_t currentPosition;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currentPosition = currentPosition_;
    }

    uint32_t moveStepsAbs = (uint32_t)abs(steps);
    uint32_t accelSteps = ((uint32_t)maxVelocity * (uint32_t)maxVelocity) / (2UL * acceleration);
    uint32_t decelSteps = accelSteps;
    uint32_t cruiseSteps = 0;
    if (accelSteps + decelSteps >= moveStepsAbs) {
        accelSteps = moveStepsAbs / 2U;
        decelSteps = moveStepsAbs - accelSteps;
    } else {
        cruiseSteps = moveStepsAbs - accelSteps - decelSteps;
    }

    uint16_t startSpeed = maxVelocity / 4U;
    if (startSpeed < kMinStartSpeedSps) startSpeed = kMinStartSpeedSps;
    if (startSpeed > maxVelocity) startSpeed = maxVelocity;

    uint16_t startInterval = TIMER_FREQ_HZ / startSpeed;
    if (startInterval < minInterval) startInterval = minInterval;

    uint16_t stepInterval = startInterval;
    uint32_t intervalQ16 = ((uint32_t)stepInterval) << kQ16Shift;
    uint32_t accelDeltaQ16 = 0;
    uint32_t decelDeltaQ16 = 0;
    if (accelSteps > 0 && startInterval > minInterval) {
        accelDeltaQ16 = (((uint32_t)(startInterval - minInterval)) << kQ16Shift) / accelSteps;
        decelDeltaQ16 = accelDeltaQ16;
    }
    StepperState nextState = STEPPER_DECEL;
    if (accelSteps > 0) nextState = STEPPER_ACCEL;
    else if (cruiseSteps > 0) nextState = STEPPER_CRUISE;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        direction_ = direction;
        targetPosition_ = currentPosition + steps;
        stepsRemaining_ = (int32_t)moveStepsAbs;
        accelSteps_ = accelSteps;
        decelSteps_ = decelSteps;
        cruiseSteps_ = cruiseSteps;
        startInterval_ = startInterval;
        stepInterval_ = stepInterval;
        stepCounter_ = stepInterval;
        stepCount_ = 0;
        currentVelocity_ = startSpeed;
        intervalQ16_ = intervalQ16;
        accelDeltaQ16_ = accelDeltaQ16;
        decelDeltaQ16_ = decelDeltaQ16;
        state_ = nextState;
        if (dirOutReg_) {
            if (direction_ > 0) *dirOutReg_ |= dirMask_;
            else *dirOutReg_ &= (uint8_t)~dirMask_;
        } else {
            digitalWrite(pinDir_, (direction_ > 0) ? HIGH : LOW);
        }
    }

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.print(F("] Moving "));
    DEBUG_SERIAL.print(steps);
    DEBUG_SERIAL.print(F(" steps (accel="));
    DEBUG_SERIAL.print(accelSteps_);
    DEBUG_SERIAL.print(F(", cruise="));
    DEBUG_SERIAL.print(cruiseSteps_);
    DEBUG_SERIAL.print(F(", decel="));
    DEBUG_SERIAL.print(decelSteps_);
    DEBUG_SERIAL.println(F(")"));
#endif
}

void StepperMotor::moveToPosition(int32_t position) {
    int32_t currentPosition;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currentPosition = currentPosition_;
    }
    moveSteps(position - currentPosition);
}

void StepperMotor::home(int8_t direction) {
    if (!enabled_ || !hasLimit_) return;

    int8_t homeDir = (direction > 0) ? 1 : -1;

    uint16_t homeSpeed = maxVelocity_ / 4U;
    if (homeSpeed < kMinStartSpeedSps) homeSpeed = kMinStartSpeedSps;
    uint16_t stepInterval = TIMER_FREQ_HZ / homeSpeed;
    if (stepInterval < minInterval_) stepInterval = minInterval_;

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        direction_ = homeDir;
        stepInterval_ = stepInterval;
        stepCounter_ = stepInterval;
        currentVelocity_ = (stepInterval > 0) ? (TIMER_FREQ_HZ / stepInterval) : 0;
        state_ = STEPPER_HOMING;
        stepsRemaining_ = INT32_MAX;
        if (dirOutReg_) {
            if (direction_ > 0) *dirOutReg_ |= dirMask_;
            else *dirOutReg_ &= (uint8_t)~dirMask_;
        } else {
            digitalWrite(pinDir_, (direction_ > 0) ? HIGH : LOW);
        }
    }

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.println(F("] Homing started"));
#endif
}

void StepperMotor::stop() {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        state_ = STEPPER_IDLE;
        stepsRemaining_ = 0;
        currentVelocity_ = 0;
    }

#ifdef DEBUG_STEPPER
    DEBUG_SERIAL.print(F("[Stepper "));
    DEBUG_SERIAL.print(stepperId_);
    DEBUG_SERIAL.println(F("] Emergency STOP"));
#endif
}

void StepperMotor::smoothStop() {
    if (state_ == STEPPER_IDLE) return;

    uint16_t currentVelocity;
    uint16_t stepInterval;
    uint16_t acceleration;
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currentVelocity = currentVelocity_;
        stepInterval = stepInterval_;
        acceleration = acceleration_;
    }

    uint32_t decelDistance = ((uint32_t)currentVelocity * (uint32_t)currentVelocity) / (2UL * acceleration);
    if (decelDistance < 1U) {
        ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
            state_ = STEPPER_IDLE;
            stepsRemaining_ = 0;
            currentVelocity_ = 0;
        }
        return;
    }

    uint16_t stopInterval = TIMER_FREQ_HZ / kMinStopSpeedSps;
    uint32_t decelDeltaQ16 = 0;
    if (stopInterval > stepInterval) {
        decelDeltaQ16 = (((uint32_t)(stopInterval - stepInterval)) << kQ16Shift) / decelDistance;
    }

    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        stepsRemaining_ = (int32_t)decelDistance;
        accelSteps_ = 0;
        cruiseSteps_ = 0;
        decelSteps_ = decelDistance;
        stepCount_ = 0;
        state_ = STEPPER_DECEL;
        decelDeltaQ16_ = decelDeltaQ16;
        intervalQ16_ = ((uint32_t)stepInterval) << kQ16Shift;
    }
}

void StepperMotor::setPosition(int32_t position) {
    ATOMIC_BLOCK(ATOMIC_RESTORESTATE) {
        currentPosition_ = position;
    }
}

// ============================================================================
// MOTION PROFILE CALCULATION
// ============================================================================

void StepperMotor::calculateProfile(int32_t totalSteps) {
    uint32_t accelDist = ((uint32_t)maxVelocity_ * (uint32_t)maxVelocity_) / (2UL * acceleration_);
    accelSteps_ = accelDist;
    decelSteps_ = accelDist;

    if (accelSteps_ + decelSteps_ >= (uint32_t)totalSteps) {
        accelSteps_ = totalSteps / 2;
        decelSteps_ = totalSteps - accelSteps_;
        cruiseSteps_ = 0;
    } else {
        cruiseSteps_ = totalSteps - accelSteps_ - decelSteps_;
    }
}

// ============================================================================
// TIMER CALLBACK (called from Timer3 ISR @ 10kHz)
// ============================================================================

void StepperMotor::timerCallback() {
    if (state_ == STEPPER_IDLE) {
        return;
    }

    if (stepCounter_ > 0) {
        stepCounter_--;
        return;
    }

    stepCounter_ = stepInterval_;

    if (state_ == STEPPER_HOMING && isLimitTriggered()) {
        state_ = STEPPER_IDLE;
        currentPosition_ = 0;
        stepsRemaining_ = 0;
        currentVelocity_ = 0;
        return;
    }

    if (stepOutReg_) {
        *stepOutReg_ |= stepMask_;
        *stepOutReg_ &= (uint8_t)~stepMask_;
    } else {
        digitalWrite(pinStep_, HIGH);
        digitalWrite(pinStep_, LOW);
    }

    currentPosition_ += direction_;
    stepsRemaining_--;
    stepCount_++;

    if (stepsRemaining_ <= 0) {
        state_ = STEPPER_IDLE;
        currentVelocity_ = 0;
        return;
    }

    switch (state_) {
        case STEPPER_ACCEL:
            if (accelDeltaQ16_ > 0 && stepInterval_ > minInterval_) {
                uint32_t minIntervalQ16 = ((uint32_t)minInterval_) << kQ16Shift;
                if (intervalQ16_ > minIntervalQ16 + accelDeltaQ16_) intervalQ16_ -= accelDeltaQ16_;
                else intervalQ16_ = minIntervalQ16;
                stepInterval_ = (uint16_t)(intervalQ16_ >> kQ16Shift);
                if (stepInterval_ < minInterval_) stepInterval_ = minInterval_;
            }
            if (stepCount_ >= accelSteps_) {
                state_ = (cruiseSteps_ > 0) ? STEPPER_CRUISE : STEPPER_DECEL;
                intervalQ16_ = ((uint32_t)stepInterval_) << kQ16Shift;
            }
            break;

        case STEPPER_CRUISE:
            if (stepCount_ >= accelSteps_ + cruiseSteps_) {
                state_ = STEPPER_DECEL;
                intervalQ16_ = ((uint32_t)stepInterval_) << kQ16Shift;
            }
            break;

        case STEPPER_DECEL:
            if (decelDeltaQ16_ > 0) {
                uint16_t stopInterval = TIMER_FREQ_HZ / kMinStopSpeedSps;
                uint32_t stopIntervalQ16 = ((uint32_t)stopInterval) << kQ16Shift;
                if (intervalQ16_ + decelDeltaQ16_ < stopIntervalQ16) intervalQ16_ += decelDeltaQ16_;
                else intervalQ16_ = stopIntervalQ16;
                stepInterval_ = (uint16_t)(intervalQ16_ >> kQ16Shift);
                if (stepInterval_ < minInterval_) stepInterval_ = minInterval_;
                if (stepInterval_ > stopInterval) stepInterval_ = stopInterval;
            }
            break;

        case STEPPER_HOMING:
        default:
            break;
    }

    currentVelocity_ = (stepInterval_ > 0) ? (TIMER_FREQ_HZ / stepInterval_) : 0;
}

bool StepperMotor::isLimitTriggered() const {
    if (!hasLimit_) return false;

    if (limitInReg_) {
        bool pinHigh = ((*limitInReg_ & limitMask_) != 0);
        return pinHigh == (limitActiveState_ == HIGH);
    }

    return digitalRead(pinLimit_) == limitActiveState_;
}
