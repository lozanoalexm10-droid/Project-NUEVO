/**
 * @file pins.h
 * @brief [Rev. B] Hardware pin definitions for Arduino Mega 2560
 *
 * This file contains all GPIO pin assignments matching the hardware design.
 * Pin definitions are separated from config.h for clarity and maintainability.
 *
 * NOTE: This file mirrors the GPIO table in README.md and pin_table_rev_B.md.
 *       Any hardware changes should be reflected here.
 *
 * Key changes from Rev. A (see pins_rev_A.h for archive):
 *   - M1 and M2 wheel motors now use both encoder channels on dedicated INT
 *     pins (INT0/INT1 for M1, INT5/INT4 for M2) for full 4x quadrature.
 *   - M3 and M4 encoders relocated to PCINT pins (A14/A15 and pins 11/12).
 *   - M1_EN moved 5→6 (Timer4 OC4A), M2_EN moved 6→7 (Timer4 OC4B).
 *   - LED_RED moved 11→5 (Timer3 OC3A).
 *   - M2_IN1 moved 12→4, M2_IN2 moved 13→30.
 *   - Pins 13 and 31 freed as general-purpose user GPIOs.
 *   - Analog expansion reduced to A7–A13 (A14/A15 used for M3 encoders).
 */

#ifndef PINS_H
#define PINS_H

// ============================================================================
// SERIAL COMMUNICATION
// ============================================================================

// Serial0 (USB) - pins 0/1 used by Arduino core
#define PIN_USB_RX              0       // USB Serial RX (programming/debug)
#define PIN_USB_TX              1       // USB Serial TX (programming/debug)

// Serial2 (Raspberry Pi UART) - pins 16/17
#define PIN_RPI_TX              16      // TX to RPi5 (via level shifter 5V→3.3V)
#define PIN_RPI_RX              17      // RX from RPi5 (via level shifter 3.3V→5V)

// NOTE: Serial1 (pins 18/19) NOT AVAILABLE — used by M2_ENC_A / M2_ENC_B
// NOTE: Serial3 (pins 14/15) NOT AVAILABLE — used by ST1_STEP / ST2_STEP

// ============================================================================
// I2C BUS
// ============================================================================

#define PIN_I2C_SDA             20      // I2C Data (Qwiic + PCA9685)
#define PIN_I2C_SCL             21      // I2C Clock (Qwiic + PCA9685)

// ============================================================================
// DC MOTOR 1 (Default Left/Right Wheel)
// ============================================================================

#define PIN_M1_EN               6       // PWM Enable (speed control) [Timer4 OC4A]
#define PIN_M1_IN1              8       // Direction control 1
#define PIN_M1_IN2              43      // Direction control 2
#define PIN_M1_ENC_A            2       // Encoder Phase A (INT0) — 4x quadrature
#define PIN_M1_ENC_B            3       // Encoder Phase B (INT1) — 4x quadrature
#define PIN_M1_CT               A3      // Current sense (analog)

// ============================================================================
// DC MOTOR 2 (Default Left/Right Wheel)
// ============================================================================

#define PIN_M2_EN               7       // PWM Enable (speed control) [Timer4 OC4B]
#define PIN_M2_IN1              4       // Direction control 1
#define PIN_M2_IN2              30      // Direction control 2
#define PIN_M2_ENC_A            18      // Encoder Phase A (INT5) — 4x quadrature
#define PIN_M2_ENC_B            19      // Encoder Phase B (INT4) — 4x quadrature
#define PIN_M2_CT               A4      // Current sense (analog)

// ============================================================================
// DC MOTOR 3 (Manipulator - Optional)
// ============================================================================

#define PIN_M3_EN               9       // PWM Enable (speed control)
#define PIN_M3_IN1              34      // Direction control 1
#define PIN_M3_IN2              35      // Direction control 2
#define PIN_M3_ENC_A            A14     // Encoder Phase A (PCINT14) — 4x via PCINT
#define PIN_M3_ENC_B            A15     // Encoder Phase B (PCINT15) — 4x via PCINT
#define PIN_M3_CT               A5      // Current sense (analog)

// ============================================================================
// DC MOTOR 4 (Manipulator - Optional)
// ============================================================================

#define PIN_M4_EN               10      // PWM Enable (speed control)
#define PIN_M4_IN1              36      // Direction control 1
#define PIN_M4_IN2              37      // Direction control 2
#define PIN_M4_ENC_A            11      // Encoder Phase A (PCINT5) — 4x via PCINT
#define PIN_M4_ENC_B            12      // Encoder Phase B (PCINT6) — 4x via PCINT
#define PIN_M4_CT               A6      // Current sense (analog)

// ============================================================================
// STEPPER MOTOR 1
// ============================================================================

#define PIN_ST1_STEP            14      // Step pulse
#define PIN_ST1_DIR             22      // Direction
#define PIN_ST1_EN              26      // Enable (active low)

// ============================================================================
// STEPPER MOTOR 2
// ============================================================================

#define PIN_ST2_STEP            15      // Step pulse
#define PIN_ST2_DIR             23      // Direction
#define PIN_ST2_EN              27      // Enable (active low)

// ============================================================================
// STEPPER MOTOR 3
// ============================================================================

#define PIN_ST3_STEP            32      // Step pulse
#define PIN_ST3_DIR             24      // Direction
#define PIN_ST3_EN              28      // Enable (active low)

// ============================================================================
// STEPPER MOTOR 4
// ============================================================================

#define PIN_ST4_STEP            33      // Step pulse
#define PIN_ST4_DIR             25      // Direction
#define PIN_ST4_EN              29      // Enable (active low)

// ============================================================================
// USER BUTTONS (Active Low - INPUT_PULLUP)
// ============================================================================

#define PIN_BTN1                38      // On-board user button 1
#define PIN_BTN2                39      // On-board user button 2
#define PIN_BTN3                40      // Button 3 (shared with LIM1)
#define PIN_BTN4                41      // Button 4 (shared with LIM2)
#define PIN_BTN5                48      // Button 5 (shared with LIM3)
#define PIN_BTN6                49      // Button 6 (shared with LIM4)
#define PIN_BTN7                50      // Button 7 (shared with LIM5)
#define PIN_BTN8                51      // Button 8 (shared with LIM6)
#define PIN_BTN9                52      // Button 9 (shared with LIM7)
#define PIN_BTN10               53      // Button 10 (shared with LIM8)

// ============================================================================
// LIMIT SWITCHES (Shared with Buttons 3-10)
// ============================================================================

#define PIN_LIM1                40      // Limit switch 1 (shared with BTN3)
#define PIN_LIM2                41      // Limit switch 2 (shared with BTN4)
#define PIN_LIM3                48      // Limit switch 3 (shared with BTN5)
#define PIN_LIM4                49      // Limit switch 4 (shared with BTN6)
#define PIN_LIM5                50      // Limit switch 5 (shared with BTN7)
#define PIN_LIM6                51      // Limit switch 6 (shared with BTN8)
#define PIN_LIM7                52      // Limit switch 7 (shared with BTN9)
#define PIN_LIM8                53      // Limit switch 8 (shared with BTN10)

// ============================================================================
// STATUS AND USER LEDs
// ============================================================================

#define PIN_LED_RED             5       // Status LED Red (error/low battery) [Timer3 OC3A]
#define PIN_LED_GREEN           44      // Status LED Green (system OK)
#define PIN_LED_BLUE            45      // User LED Blue (exposed)
#define PIN_LED_ORANGE          46      // User LED Orange (exposed)
#define PIN_LED_PURPLE          47      // User LED Purple (exposed, non-PWM)

// ============================================================================
// NEOPIXEL (WS2812B RGB LED)
// ============================================================================

#define PIN_NEOPIXEL            42      // NeoPixel data line

// ============================================================================
// USER GPIO (General Purpose — freed in Rev. B)
// ============================================================================

#define PIN_USER_P13            13      // User GPIO (was M2_IN2 in Rev. A)
#define PIN_USER_P31            31      // User GPIO (was M4_ENC_B in Rev. A)

// ============================================================================
// SERVO CONTROLLER (PCA9685)
// ============================================================================

// I2C pins are shared (PIN_I2C_SDA, PIN_I2C_SCL)
// PCA9685 Output Enable pin (active LOW) - optional for power control
// #define PIN_SERVO_OE            xx    // Define if OE pin is connected

// ============================================================================
// ANALOG VOLTAGE SENSING
// ============================================================================

#define PIN_VBAT_SENSE          A0      // Battery voltage (1:6 divider)
#define PIN_V5_SENSE            A1      // 5V rail voltage (1:2 divider)
#define PIN_VSERVO_SENSE        A2      // Servo rail voltage (1:3 divider)

// ============================================================================
// ANALOG EXPANSION (Available for Sensors)
// NOTE: A14/A15 are now used by M3 encoders — only A7–A13 are free.
// ============================================================================

#define PIN_ANALOG_EXP_1        A7      // Analog expansion 1
#define PIN_ANALOG_EXP_2        A8      // Analog expansion 2
#define PIN_ANALOG_EXP_3        A9      // Analog expansion 3
#define PIN_ANALOG_EXP_4        A10     // Analog expansion 4
#define PIN_ANALOG_EXP_5        A11     // Analog expansion 5
#define PIN_ANALOG_EXP_6        A12     // Analog expansion 6
#define PIN_ANALOG_EXP_7        A13     // Analog expansion 7

// ============================================================================
// TIMER OCR REGISTER ASSIGNMENTS — Rev B
// ============================================================================
//
// These macros map motor EN and LED pins to their controlling timer compare
// and ICR registers. Drivers write OCRnx directly instead of calling
// analogWrite() to avoid corrupting the ISR timer configuration.
//
// The "IS_OCnx" flags tell ISRScheduler::init() which output compare channels
// to connect (set COMnx bits) when configuring the timers in Fast PWM mode.
//
// Rev B assignments:
//   LED_RED(pin 5)  — Timer3 OC3A  (10 kHz carrier, from ISRScheduler)
//   M1_EN  (pin 6)  — Timer4 OC4A  (10 kHz carrier, from ISRScheduler)
//   M2_EN  (pin 7)  — Timer4 OC4B  (10 kHz carrier, same timer as M1_EN)
//

// LED Red (pin 5) — Timer3 OC3A
#define PIN_LED_RED_IS_OC3A             // Timer3 OC3A connected to pin 5 (LED_RED)
#define LED_RED_OCR      OCR3A          // Write brightness here instead of analogWrite()
#define LED_RED_ICR      ICR3           // Timer TOP for scaling: duty = (bri * ICR3) / 255

// Motor 1 EN (pin 6) — Timer4 OC4A
#define PIN_M1_EN_IS_OC4A               // Timer4 OC4A connected to pin 6 (M1_EN)
#define PIN_M1_EN_OCR    OCR4A          // Write motor speed here instead of analogWrite()
#define PIN_M1_EN_ICR    ICR4           // Timer TOP for scaling: duty = (speed * ICR4) / 255

// Motor 2 EN (pin 7) — Timer4 OC4B (shares Timer4 with M1_EN)
#define PIN_M2_EN_IS_OC4B               // Timer4 OC4B connected to pin 7 (M2_EN)
#define PIN_M2_EN_OCR    OCR4B          // Write motor speed here instead of analogWrite()
#define PIN_M2_EN_ICR    ICR4           // Same timer as M1_EN — ICR4 is shared TOP

// Motor 3 EN (pin 9) — Timer2 OC2B
#define PIN_M3_EN_IS_OC2B               // Timer2 OC2B connected to pin 9 (M3_EN)
#define PIN_M3_EN_OCR    OCR2B          // Timer2 is 8-bit fast PWM; write duty directly

// Motor 4 EN (pin 10) — Timer2 OC2A
#define PIN_M4_EN_IS_OC2A               // Timer2 OC2A connected to pin 10 (M4_EN)
#define PIN_M4_EN_OCR    OCR2A          // Timer2 is 8-bit fast PWM; write duty directly

// ============================================================================
// CONVENIENCE ARRAYS FOR ITERATION
// ============================================================================

// DC Motor EN pins (PWM)
const uint8_t DC_MOTOR_EN_PINS[4] = {
  PIN_M1_EN, PIN_M2_EN, PIN_M3_EN, PIN_M4_EN
};

// DC Motor IN1 pins
const uint8_t DC_MOTOR_IN1_PINS[4] = {
  PIN_M1_IN1, PIN_M2_IN1, PIN_M3_IN1, PIN_M4_IN1
};

// DC Motor IN2 pins
const uint8_t DC_MOTOR_IN2_PINS[4] = {
  PIN_M1_IN2, PIN_M2_IN2, PIN_M3_IN2, PIN_M4_IN2
};

// DC Motor Encoder A pins (INT0/INT5 for M1/M2; PCINT14 for M3; PCINT5 for M4)
const uint8_t DC_MOTOR_ENC_A_PINS[4] = {
  PIN_M1_ENC_A, PIN_M2_ENC_A, PIN_M3_ENC_A, PIN_M4_ENC_A
};

// DC Motor Encoder B pins (INT1/INT4 for M1/M2; PCINT15 for M3; PCINT6 for M4)
const uint8_t DC_MOTOR_ENC_B_PINS[4] = {
  PIN_M1_ENC_B, PIN_M2_ENC_B, PIN_M3_ENC_B, PIN_M4_ENC_B
};

// DC Motor Current Sense pins
const uint8_t DC_MOTOR_CT_PINS[4] = {
  PIN_M1_CT, PIN_M2_CT, PIN_M3_CT, PIN_M4_CT
};

// Stepper STEP pins
const uint8_t STEPPER_STEP_PINS[4] = {
  PIN_ST1_STEP, PIN_ST2_STEP, PIN_ST3_STEP, PIN_ST4_STEP
};

// Stepper DIR pins
const uint8_t STEPPER_DIR_PINS[4] = {
  PIN_ST1_DIR, PIN_ST2_DIR, PIN_ST3_DIR, PIN_ST4_DIR
};

// Stepper ENABLE pins
const uint8_t STEPPER_EN_PINS[4] = {
  PIN_ST1_EN, PIN_ST2_EN, PIN_ST3_EN, PIN_ST4_EN
};

// User button pins
const uint8_t BUTTON_PINS[10] = {
  PIN_BTN1, PIN_BTN2, PIN_BTN3, PIN_BTN4, PIN_BTN5,
  PIN_BTN6, PIN_BTN7, PIN_BTN8, PIN_BTN9, PIN_BTN10
};

// Limit switch pins (same as buttons 3-10)
const uint8_t LIMIT_PINS[8] = {
  PIN_LIM1, PIN_LIM2, PIN_LIM3, PIN_LIM4,
  PIN_LIM5, PIN_LIM6, PIN_LIM7, PIN_LIM8
};

// User LED pins
const uint8_t USER_LED_PINS[3] = {
  PIN_LED_BLUE, PIN_LED_ORANGE, PIN_LED_PURPLE
};

#endif // PINS_H
