# Code Base Organization Plan

## Goal
- Make the firmware easy to read, reason about, and modify without changing the current working behavior.
- Keep `arduino.ino` as the top-level orchestration file only.
- Separate hard real-time code, fast loop services, periodic soft tasks, and status/debug reporting into clear modules.
- Use this cleanup as the structural step before `v0.9.0`.

## Refactor Principles
- Refactor behavior last. First move code to the right ownership boundaries, then simplify internals.
- Do not move code into a module just because it is currently large. Move it only if the responsibility clearly belongs there.
- Do not create a generic `utility.cpp` dumping ground. Use small purpose-specific modules instead.
- Preserve the current mixed DC control architecture:
  - `TIMER1`: short latch/apply round-robin ISR
  - `loop()`: fast-path polling and staged compute trigger
  - `Scheduler`: millis-based periodic soft tasks only
- Keep debug/status code out of the hot control path as much as possible.

## Important Design Decisions

### 1. `arduino.ino` should be small, but not artificially small
- Target: about `250-350` lines including comments is reasonable.
- The main requirement is not the exact line count; it is that `arduino.ino` reads like an entry point.
- It should answer only these questions:
  - what initializes at boot
  - what ISR vectors exist
  - what soft tasks exist
  - what the main loop order is

### 2. System info code should NOT go into `SystemManager`
- `SystemManager` should stay the firmware state machine and transition owner.
- Status/debug snapshot and printing are a different responsibility.
- Create a dedicated module instead, for example:
  - `src/modules/StatusReporter.h/.cpp`
- That module should own:
  - status snapshot capture
  - chunked debug/status printing
  - formatting of `[SYSTEM]`, `[TIMING]`, `[UART]`
  - windowed counters used only for reporting

### 3. Do not force the fast loop path into the current periodic scheduler
- The current scheduler is millis-based periodic dispatch.
- The current fast loop path is different:
  - `MessageCenter::drainUart()`
  - `MessageCenter::drainTx()`
  - DC compute trigger handling
  - debug chunk emission / flush
- Those need opportunistic servicing every loop iteration, not “run every X ms”.
- So instead of overloading `Scheduler`, create a separate fast-path loop service, for example:
  - `src/runtime/LoopService.h/.cpp`
- Then `loop()` can stay very small while preserving the correct semantics.

### 4. `LoopMonitor` should be integrated at the right level
- Soft periodic tasks can be wrapped by the scheduler.
- ISR timing must remain measured at the ISR call sites.
- So the monitor belongs in two places:
  - inside `Scheduler` for soft tasks
  - explicit calls inside ISR vectors and fast loop services
- Do not try to hide ISR timing inside the scheduler.

## Target Module Boundaries

### `arduino.ino`
Should contain only:
- includes
- global module instances already required by Arduino linkage
- ISR vectors
- `setup()`
- `loop()`
- task callback declarations that are one-line wrappers if Arduino requires them

It should not contain:
- long status formatting code
- large chunks of motor-control coordination logic
- pin/interrupt attachment details
- protocol formatting details

### `src/runtime/AppRuntime.h/.cpp`
Own:
- high-level boot sequence
- registering scheduler tasks
- starting ISR scheduler
- top-level loop service order

Potential public API:
- `AppRuntime::init()`
- `AppRuntime::serviceLoop()`

### `src/runtime/LoopService.h/.cpp`
Own the fast-path operations that must run every loop:
- `MessageCenter::drainUart()`
- `MessageCenter::drainTx()`
- DC compute trigger handling
- status chunk emission
- debug flush timing

Potential public API:
- `LoopService::pollFastPath()`
- `LoopService::pollDeferred()`

### `src/runtime/MotorControlCoordinator.h/.cpp`
Own the mixed ISR/soft coordination state for DC motors:
- round counters
- request/compute/apply bookkeeping
- pipeline counters like `missed/reused/cross`
- helper functions for ISR side and loop side

This is one of the biggest current responsibilities living in `arduino.ino`.

### `src/modules/StatusReporter.h/.cpp`
Own:
- status snapshot struct
- formatting helpers
- windowed reporting counters
- the 1 Hz user-facing report
- immediate event log helpers if needed

This module should consume data from other modules, not own system behavior.

### `src/board/PinMap.h` or improved `pins.h`
Pin constants can stay as a header, but the ownership should be explicit.
- Keep pin numbers and hardware register mapping in one place.
- Do not mix pin definitions with pin attachment logic.

### `src/board/BoardInterrupts.h/.cpp` or fold into existing scheduler modules
Own:
- `attachInterrupt()` calls
- PCINT registration for M3/M4 encoders
- pin-change mask setup
- any board-level interrupt attachment details

The current code base already has `ISRScheduler.*`; that may be the right home if kept focused.

## Concrete Cleanup Tasks

### A. Extract status and debug reporting out of `arduino.ino`
- Move:
  - `StatusSnapshot`
  - formatting helpers
  - chunked `[SYSTEM] / [TIMING] / [UART]` output
  - reporting window resets
- Keep only a single call in the loop path, for example:
  - `StatusReporter::service()`

### B. Extract mixed DC control coordination out of `arduino.ino`
- Move:
  - round counters
  - compute/apply pipeline flags
  - control window counters
  - request/apply bookkeeping
- Keep ISR usage explicit, for example:
  - `MotorControlCoordinator::onTimer1Slot(slot)`
  - `MotorControlCoordinator::serviceSoftCompute()`

### C. Make soft task definitions read as high-level wrappers
- Current task functions are already smaller than before, but they still own some policy.
- Goal:
  - `taskUART()` should call one or two named functions
  - `taskSensors()` should call one named function
  - `taskUserIO()` should not include detailed battery warning policy inline

### D. Clarify hard RT vs fast loop vs periodic soft tasks
- Add a short structure comment near the top of the runtime code:
  - hard RT: ISR vectors only
  - fast loop: every `loop()` iteration
  - periodic soft tasks: scheduler-managed

This matters more than adding many comments to individual lines.

### E. Move board-specific attachment/setup details out of `setup()`
- `setup()` should read like:
  - init board pins
  - init modules
  - register tasks
  - start ISRs
- It should not contain register-level attachment details inline.

### F. Replace the idea of `utility.cpp` with narrow helper modules
- Good:
  - `FormatUtils.cpp`
  - `BoardUtils.cpp`
  - `TimingUtils.cpp`
  - `ControlMath.cpp`
- Bad:
  - one generic `utility.cpp` for unrelated helpers

### G. Clean generated files out of the source tree
- `firmware/arduino/build/` is generated output and should not be treated as source.
- `.DS_Store` should not live in the firmware tree.
- This cleanup belongs in the organization pass too.

## Scheduler and Monitoring Refinement

### Scheduler
- Keep `Scheduler` focused on periodic soft tasks.
- It should know:
  - task interval
  - task callback
  - task priority
  - optional timing/monitor slot

### LoopMonitor
- For periodic soft tasks, measure inside scheduler wrappers.
- For ISR tasks, measure at the ISR site.
- For fast-path loop work, measure in the loop service.
- The monitor output should reflect real ownership boundaries, not arbitrary file boundaries.

## Proposed End-State for `loop()`

The target should look roughly like this:

```cpp
void loop() {
  AppRuntime::serviceLoop();
}
```

And inside `AppRuntime::serviceLoop()`:

```cpp
void AppRuntime::serviceLoop() {
  LoopService::pollFastPath();
  Scheduler::tick();
  LoopService::pollFastPath();
}
```

This keeps the loop readable without forcing the fast path into the periodic scheduler.

## Proposed End-State for `setup()`

The target should look roughly like this:

```cpp
void setup() {
  AppRuntime::init();
}
```

And then `AppRuntime::init()` would call clearly named steps such as:
- `Board::initPins()`
- `Board::attachInterrupts()`
- `Modules::initDrivers()`
- `Modules::initManagers()`
- `AppRuntime::registerTasks()`
- `ISRScheduler::init()`

## Versioning / Scope
- This organization pass should be a structural cleanup only.
- Avoid changing protocol behavior, control timing, or tuning while doing the file moves.
- After the structure is stable and readable, that is the point to advance to `v0.9.0`.

## Suggested Execution Order
1. Extract status reporting into `StatusReporter`.
2. Extract mixed DC round coordination into a dedicated runtime/control module.
3. Extract fast loop path into `LoopService`.
4. Clean up `setup()` and task registration.
5. Move interrupt attachment / board setup out of `arduino.ino`.
6. Integrate soft-task `LoopMonitor` wrapping into `Scheduler`.
7. Remove generated files and stale tree clutter.

## Open Questions
- Do we want `AppRuntime` to be a real module, or do we prefer keeping only a thin `arduino.ino` with free functions?
- Should board-level interrupt attachment live in `ISRScheduler`, or in a separate `BoardInterrupts` module?
- Do we want the status reporter to stay debug-only, or should it also own user-facing summary strings for the UI in the future?
