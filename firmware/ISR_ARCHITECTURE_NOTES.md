# ISR Architecture Notes (UART @ 500 kbps)

This document summarizes the final ISR design for reliable UART RX while keeping
hard real-time tasks deterministic. It reflects both the original findings at 1 Mbps
and the validated architecture after relaxing to 500 kbps.

## Findings (Original State at 1 Mbps)

- At 1 Mbps, the AVR UART can only tolerate ~20 µs of interrupt-off time
  (roughly two byte times). Anything longer risks `DOR2` (overrun).
- Dropped bytes show up as:
  - `DOR2` increments (overrun) when the UART RX ISR can't run in time.
  - CRC errors in TLV decode (payload corruption).
- Adding `sei()` inside ISRs was tested and does not fix the problem — it shifts
  overruns to other tasks because nested interrupts create unpredictable timing.
- At 500 kbps (8N1): byte every 20 µs, 2-byte HW FIFO → `USART2_RX_vect` must
  fire within **40 µs**. This doubles the ISR budget and makes the architecture
  viable without any `sei()`.

## Constraints on ATmega2560

- No configurable interrupt priorities. There is only global enable/disable.
- Nested interrupts are possible by calling `sei()` inside an ISR, but create
  unpredictable timing and were explicitly rejected.
- Worst-case ISR pile-up at 500 kbps: TIMER3 (~3 µs) + TIMER1 (~15 µs) +
  TIMER0/millis (~5 µs) = **~23 µs** — well inside the 40 µs budget.

## Final Architecture

### Layer 1: Hard Real-Time ISRs (short, no `sei()`)

```
USART2_RX_vect     per-byte     ~2 µs
  - Provided by Arduino core (HardwareSerial).
  - Pushes bytes into the 1024-byte HardwareSerial ring buffer.
  - Not replaced; all other ISRs must stay short enough to not block it.

TIMER3_OVF_vect    10 kHz       <5 µs
  - Stepper step pulses only (direct PORTx writes — no digitalWrite).
  - Precompute step intervals in loop; ISR decrements counter, pulses STEP pin.
  - Remove delayMicroseconds(2); use 8× nop for ~500 ns STEP pulse width.

TIMER1_OVF_vect    800 Hz       <20 µs
  - Full fixed-point round-robin PID: 1 motor per tick = 200 Hz per motor.
  - Uses Q16.16 fixed-point, constant dt, no ADC, no prints, no micros().
  - Direct OCR register writes for PWM (no analogWrite/digitalWrite).
  - Velocity estimate precomputed in loop and latched into a volatile Q16.16.

ENCODER ISRs       edge-triggered   ~1–2 µs
  - Edge counting only (INT0/INT1/INT4/INT5).
```

Key rule: no ISR may call `sei()`. Each ISR must exit before the next byte
arrives (every 20 µs at 500 kbps). The 40 µs tolerance means even back-to-back
TIMER3 + TIMER1 firings leave safe margin.

### Layer 2: Loop Tasks via Scheduler (millis-based, preemptible)

```cpp
loop() {
    // 1. Highest priority: drain Serial2 into TLV decoder
    while (Serial2.available()) {
        uint8_t b = (uint8_t)Serial2.read();
        decodePacket(&decodeDesc_, &b);
    }

    // 2. Run one scheduled task per iteration
    Scheduler::tick();
}
```

Scheduled tasks:
| Task | Rate | Work |
|------|------|------|
| UART telemetry TX | 100 Hz | `MessageCenter::sendTelemetry()` |
| Heartbeat timeout check | 100 Hz | `MessageCenter::checkHeartbeatTimeout()` |
| Sensor I2C reads | 100 / 50 / 10 Hz | IMU, lidar, ultrasonic |
| Buttons + safety | 100 Hz | UserIO, SafetyManager |
| UserIO / LEDs | 20 Hz | NeoPixel, status LED |
| **Debug print flush** | 10 Hz | `taskDebugFlush()` — see below |

### Layer 3: Debug Print Buffer (non-blocking)

Direct `DEBUG_SERIAL.print()` calls block for up to ~7 ms (80 chars at 115200 baud),
creating long loop() gaps that delay Serial2 draining. Replace all debug prints with
a ring buffer and a non-blocking flush task:

```cpp
// ---- DebugLog (simple inline ring buffer) ----

#define DBG_BUF_SIZE 512

static char     dbgBuf_[DBG_BUF_SIZE];
static uint16_t dbgHead_ = 0;
static uint16_t dbgTail_ = 0;

// Call only from loop() / Scheduler context (not ISR-safe)
inline void dbgPrint(const char *str) {
    while (*str) {
        uint16_t next = (dbgTail_ + 1) % DBG_BUF_SIZE;
        if (next == dbgHead_) return;  // buffer full — drop
        dbgBuf_[dbgTail_] = *str++;
        dbgTail_ = next;
    }
}

// Scheduler task @ 10 Hz — non-blocking, outputs only what the TX buffer can absorb
void taskDebugFlush() {
    int avail = DEBUG_SERIAL.availableForWrite();
    while (avail > 0 && dbgHead_ != dbgTail_) {
        DEBUG_SERIAL.write(dbgBuf_[dbgHead_]);
        dbgHead_ = (dbgHead_ + 1) % DBG_BUF_SIZE;
        avail--;
    }
}
```

At 115200 baud, `taskDebugFlush` at 10 Hz passes up to 64 bytes/call (the USB TX
buffer size) → 640 bytes/sec throughput. This is enough for all status output
without ever blocking loop().

## Confirmed Decisions

- **Baud rate: 500 kbps** (relaxed from 1 Mbps to double the ISR budget to 40 µs).
- Do not replace `USART2_RX_vect`. It already exists in Arduino core.
- **All ISRs < 20 µs body time, no `sei()` anywhere.**
- `SERIAL_RX_BUFFER_SIZE = 1024` (via `-DSERIAL_RX_BUFFER_SIZE=1024` compiler flag).
  At 500 kbps the buffer fills in ~20 ms — loop drain is easily fast enough.
- Stepper ISR uses direct port writes; no `delayMicroseconds()` or `digitalWrite()`.
- **Full fixed-point PID in TIMER1 ISR** (viable at 500 kbps budget).
- **Serial2 drained continuously in loop()** — not in any ISR.
- TLV decode (`decodePacket`) runs in loop(), called directly from the drain loop.
- Decode callback (`decodeCallback`) only updates variables — no I2C, no prints.
- Sensor I2C reads move to Scheduler tasks; TIMER4 removed entirely.
- All debug output goes through `dbgPrint()` → ring buffer → `taskDebugFlush()`.

## ISR Budget Summary

| ISR | Frequency | Estimated body | Total (with overhead) | Budget |
|-----|-----------|---------------|----------------------|--------|
| USART2_RX_vect | per-byte | ~1 µs | ~2 µs | — |
| TIMER3_OVF | 10 kHz | ~1–2 µs | ~4 µs | < 40 µs |
| TIMER1_OVF | 800 Hz | ~10–15 µs | ~17 µs | < 40 µs |
| TIMER0_OVF (millis) | ~977 Hz | ~3 µs | ~5 µs | unavoidable |
| Worst-case pile-up | — | — | TIMER3 + TIMER1 + TIMER0 ≈ 26 µs | ✓ |

## Fixed-Point PID Structure (TIMER1 ISR)

```cpp
// Q16.16 fixed-point. All gains precomputed in setup()/loop().
struct PIDFx {
    int32_t kp;      // Q16.16
    int32_t ki_dt;   // ki × dt, precomputed
    int32_t kd_div_dt; // kd / dt, precomputed
    int32_t i_acc;   // integrator accumulator, Q16.16
    int32_t e_prev;  // previous error, Q16.16
};

inline int16_t pid_step(PIDFx& p, int32_t e_q16,
                        int16_t out_min, int16_t out_max) {
    p.i_acc += ((int64_t)p.ki_dt * e_q16) >> 16;
    if (p.i_acc > ((int32_t)out_max << 16)) p.i_acc = ((int32_t)out_max << 16);
    if (p.i_acc < ((int32_t)out_min << 16)) p.i_acc = ((int32_t)out_min << 16);

    int32_t p_term = ((int64_t)p.kp       * e_q16)             >> 16;
    int32_t d_term = ((int64_t)p.kd_div_dt * (e_q16 - p.e_prev)) >> 16;
    p.e_prev = e_q16;

    int32_t out = p_term + p.i_acc + d_term;
    if (out > ((int32_t)out_max << 16)) out = ((int32_t)out_max << 16);
    if (out < ((int32_t)out_min << 16)) out = ((int32_t)out_min << 16);
    return (int16_t)(out >> 16);
}

ISR(TIMER1_OVF_vect) {
    static uint8_t motor = 0;  // round-robin 0..3
    int32_t pos    = enc[motor].getCount();          // atomic volatile read
    int32_t vel    = velEstQ16[motor];               // latched from loop
    int32_t e_q16  = (targetVel[motor] - vel);       // or position error
    int16_t pwm    = pid_step(pid[motor], e_q16, -255, 255);
    setPWM_fast(motor, pwm);   // direct OCRnx write
    motor = (motor + 1) & 0x03;
    // NO sei()
}
```

## Changes from Previous Architecture

| What | Before | After |
|------|--------|-------|
| UART baud | 1 Mbps | **500 kbps** |
| ISR budget | 20 µs | **40 µs** |
| UART drain | TIMER3 ISR at 10 kHz | **loop() continuous drain** |
| PID | TIMER1 ISR, float, 4 motors/tick, ~200 µs | **TIMER1 ISR, Q16.16, 1 motor/tick, <17 µs** |
| Stepper | TIMER3 with digitalWrite | **TIMER3 with direct port writes** |
| Sensor I2C | TIMER4 ISR with sei() | **Scheduler tasks (no ISR)** |
| TLV decode | TIMER4 ISR | **loop() immediately after drain** |
| TLV TX | TIMER4 ISR | **Scheduler task @ 100 Hz** |
| TIMER4 | Sensor + UART decode/TX | **Removed entirely** |
| Debug prints | Blocking DEBUG_SERIAL.print() | **Ring buffer + taskDebugFlush @ 10 Hz** |

## Implementation Order

1. **Implement and validate `test_uart_loop_drain`** (done — zero DOR2 at 500 kbps confirmed).

2. **Main firmware changes**:
   - Set `RPI_BAUD_RATE = 500000` in `config.h` (done).
   - Add `-DSERIAL_RX_BUFFER_SIZE=1024` compiler flag.
   - Strip TIMER1 to fixed-point round-robin PID only.
   - Strip TIMER3 to stepper port-writes only; remove `drainUart()`.
   - Remove TIMER4 ISR entirely.
   - Move decode + drain to top of `loop()`.
   - Move sensors, buttons, safety, TX to `Scheduler::tick()`.
   - Replace all `DEBUG_SERIAL.print()` with `dbgPrint()` + `taskDebugFlush`.

3. **Validate**:
   - `DOR2 == 0` sustained at 500 kbps.
   - PID control quality (step response, position hold) acceptable.
   - Stepper motion smooth.
   - LoopMonitor reports all slots within budget.
