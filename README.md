# ECEN-5813-Final_Project_mu2d2

Automatic plant watering system using an STM32F091RC microcontroller. The system monitors soil moisture via an ADC resistive sensor and controls a servo-actuated water valve based on moisture thresholds.

## Overview

**Target Hardware:**
- **MCU**: STM32F091RC (Cortex-M0, 48 MHz)
- **Build Tool**: STM32CubeIDE with GNU Make
- **Serial Output**: 9600 baud (8-N-1) for debug logging

**Key Features:**
- Non-blocking soil moisture measurement with power-gating to reduce sensor corrosion
- Servo-controlled water valve with smooth angle ramping
- State machine-based watering logic with thresholds for dry/wet detection
- Comprehensive unit tests with error sentinel handling
- ADC timeout protection and sensor health checks

## Code Structure

### Module Interactions Diagram

```
main.c (Application Entry)
    ├── init_uled() / set_uled()           [pwm.c]        (Heartbeat LED)
    ├── init_PWM_SERVO()                   [pwm.c]        (Servo PWM init)
    ├── init_systick()                     [timers.c]     (Millisecond tick)
    ├── soil_sensor_init()                 [soil_sensor.c]
    │   ├── init_ADC()                     [adc.c]        (ADC peripheral init)
    │   └── init_soil_sensor_power()       [soil_sensor.c](GPIO for power control)
    │
    └── Main Loop:
        ├── soil_measure_fsm()              [soil_sensor.c]
        │   ├── adc_manual_sample()         [adc.c]       (Read ADC with timeout)
        │   └── soil_raw_to_pct()           [soil_sensor.c](Convert to %)
        │
        ├── water_fsm_run()                 [state_machine.c]
        │   ├── soil_sensor_get_raw()       [soil_sensor.c](Query measurement)
        │   ├── servo_set_angle()           [pwm.c]       (Actuate valve)
        │   └── LOG()                       [log.h]       (Debug output)
        │
        └── heartbeat_led()                 [main.c]      (Blink indicator)
```

### Module Descriptions

#### **adc.c / adc.h**
**Purpose**: STM32F091 ADC peripheral control with timeouts and error handling.

**Key Functions:**
- `init_ADC()` — Initializes ADC1 with HSI14 oscillator, configures PA0 as analog input, performs calibration with loop-iteration timeouts to prevent hangs.
- `adc_manual_sample()` — Triggers conversion on ADC1_IN0, waits for EOC with timeout, returns raw 12-bit value (0–4095) or `ERROR_CODE` (0xFFFF) on failure.
- `adc_initialized()` — Returns non-zero if ADC init succeeded, zero if hardware failed.

**Error Handling:**
- If HSI14 oscillator fails to start: logs "ADC Error: HSI14 oscillator timeout", sets internal `adc_ok = 0`, returns early.
- If conversion timeout: logs "ADC Error: conversion EOC timeout", returns `ERROR_CODE`.
- Callers (soil_sensor, state_machine) check for `ERROR_CODE` and abort/retry on sensor error.

**Dependencies:**
- `utilities.h` (for `ERROR_CODE`, `MODIFY_FIELD`, `LOG`)
- STM32F091 HAL headers

#### **soil_sensor.c / soil_sensor.h**
**Purpose**: Non-blocking soil moisture measurement with power-gating and ADC-to-percentage conversion.

**State Machine:**
- `SOIL_IDLE` — Waiting for measurement request.
- `SOIL_WAIT` — Sensor powered on, stabilizing (50 ms).
- `SOIL_DONE` — Measurement complete, power off, data ready.

**Key Functions:**
- `soil_sensor_init()` — Calls `init_ADC()` and `init_soil_sensor_power()`, sets state to IDLE.
- `soil_sensor_begin_measurement()` — Powers on sensor, resets stabilization timer, transitions to WAIT.
- `soil_measure_fsm()` — Non-blocking state handler. On WAIT → checks timer; if stabilized, samples ADC (checks for `ERROR_CODE`), converts to %, powers off, transitions to DONE.
- `soil_raw_to_pct(uint16_t raw)` — Linear conversion: clamps raw to [SOIL_RAW_MIN, SOIL_RAW_MAX] and maps to [0, 100]%.
- `soil_sensor_get_raw()`, `soil_sensor_get_percent()` — Accessors for measurement results.

**Calibration Constants:**
```c
#define SOIL_RAW_MIN (1U)    // Dry point (ADC reading)
#define SOIL_RAW_MAX (4095U) // Wet point (ADC reading)
```

**Dependencies:**
- `adc.h`, `timers.h`, `log.h`

#### **pwm.c / pwm.h**
**Purpose**: PWM-based servo control and user LED (heartbeat).

**Hardware:**
- **Servo PWM**: Timer 3, Channel 2 on PA7, 50 Hz frequency (20 ms period), 500–2500 µs pulse width.
- **User LED**: PA5 (ULED) for heartbeat indicator.
- **Servo Angle Range**: 0–270°, mapped to 500–2500 µs.

**Key Functions:**
- `init_PWM_SERVO()` — Configures TIM3 for 50 Hz PWM, sets neutral position (1500 µs), enables interrupt.
- `servo_set_angle(uint16_t angle)` — Updates target angle; TIM3 ISR smoothly ramps current angle toward target at `CCR_STEP_SIZE` (10 µs/tick).
- `servo_angle_to_ccr(uint16_t angle)` — Converts 0–270° angle to 500–2500 µs with clamping. **Used by tests.**
- `init_uled()` — Configures PA5 as output.
- `set_uled(uint8_t state)` — Sets LED on (1) or off (0).
- `TIM3_IRQHandler()` — Servo ramping ISR (50 Hz tick).

**Dependencies:**
- `utilities.h` (for `MODIFY_FIELD`, `ON`, `OFF`)

#### **state_machine.c / state_machine.h**
**Purpose**: Main application logic. FSM controls watering decisions based on soil moisture thresholds.

**States (7):**
- `WATER_IDLE` — Idle, sampling every 30 seconds.
- `WATER_MEASURE` — Initiate soil measurement.
- `WATER_WAIT_MEASURE` — Wait for sensor stabilization.
- `WATER_DECIDE` — Check soil level; if dry, open valve; else, return to idle.
- `WATER_OPEN_VALVE` — Actuate servo to open position (270°), begin fast sampling (300 ms).
- `WATER_WATERING` — Monitor soil; stop when wet.
- `WATER_CLOSE_VALVE` — Close valve (0°), return to idle sampling.

**Thresholds:**
```c
#define DYING_OF_THIRST_THRESHOLD (500)   // Critical (warning only)
#define DRY_THRESHOLD             (1000)  // Decision point: open valve
#define WET_THRESHOLD             (2100)  // Decision point: close valve
#define SOAKING_THRESHOLD         (2800)  // Critical (warning only)
```

**Key Functions:**
- `water_fsm_init()` — Initialize state, schedule first idle sample.
- `water_fsm_run()` — Main FSM loop; each call advances state machine, logs transitions.
- `water_state_to_string(water_state_t)` — Converts enum to readable state name (for logging).
- `schedule_next_sample(uint8_t rate)` — Centralized scheduling: rate 0 → 30 sec idle, rate 1 → 300 ms watering.

**Error Handling:**
- If `soil_sensor_get_raw()` returns `ERROR_CODE`, FSM logs error, aborts measurement, schedules retry.
- If during WATERING phase ADC fails, FSM closes valve and returns to IDLE.

**Dependencies:**
- `soil_sensor.h`, `pwm.h`, `timers.h`, `log.h`, `utilities.h` (for `ERROR_CODE`)

#### **timers.c / timers.h**
**Purpose**: Non-blocking timing via SysTick.

**Key Functions:**
- `init_systick()` — Configures SysTick for 1 ms interrupts.
- `now()` — Returns milliseconds since boot (volatile counter).
- `reset_timer(uint8_t id)` — Resets one of several software timers.
- `get_timer(uint8_t id)` — Returns elapsed time since last reset (in ms).

**Timers Used:**
- `TIMER_START_ID` — Main loop timing.
- `TIMER_START_ULED_ID` — Heartbeat LED blink (1 sec period).
- `TIMER_SOIL_SAMPLING_ID` — Soil sensor stabilization timer.

**Dependencies:**
- `utilities.h` (for `ERROR` macro)

#### **main.c**
**Purpose**: Application entry point and main loop orchestration.

**Initialization Order:**
1. Configure UART for `LOG()` output (9600 baud).
2. Initialize LED, servo PWM, SysTick.
3. Initialize soil sensor (which calls `init_ADC()`).
4. Optional: Run unit tests (if `#define RUN_TESTS` enabled).
5. Start main loop.

**Main Loop:**
```c
for(;;) {
    soil_measure_fsm();      // Non-blocking moisture measurement
    water_fsm_run();         // Non-blocking watering logic
    heartbeat_led();         // Blink indicator every 1 second
}
```

**Dependencies:**
- All module headers, test headers (if `RUN_TESTS` enabled).

#### **log.h**
**Purpose**: Debug macro for serial output (provided, not modified).

**Usage:**
```c
LOG("Message format string %u\r\n", value);
```

Outputs to UART at 9600 baud for real-time debugging.

## Unit Testing Plan

### Test Files Location
All tests reside in `Src/test/` with matching module structure:
- `test_pwm.c/h` — Servo angle-to-CCR conversion tests.
- `test_soil_sensor.c/h` — Raw-to-percentage conversion tests.
- `test_adc.c/h` — ADC sampling and initialization tests.
- `test_state_machine.c/h` — State name and threshold constant tests.

### Test Architecture

**Conditional Compilation:**
- Tests are compiled into the firmware but only run if `#define RUN_TESTS` is uncommented in `main.c`.
- When enabled, tests run at startup before the main loop.
- If all tests pass, the user LED turns ON; if any fail, LED remains OFF.

**Error Sentinel:**
- Invalid ADC reads return `ERROR_CODE (0xFFFFU)` — outside valid 12-bit range (0–4095).
- Tests and FSM check for this sentinel and fail gracefully.

### Test Coverage

#### **test_pwm.c**
- 6 test vectors covering 0°, 90°, 135°, 180°, 270°, and 360° (clamp test).
- Validates linear angle-to-CCR conversion formula.
- **No hardware required** (pure math test).

#### **test_soil_sensor.c**
- 5 test vectors: raw ADC values 0, 1, 2048, 4095, 4096.
- Validates percentage mapping and clamping.
- **No hardware required** (pure conversion test).
- Uses calibration constants from `soil_sensor.h` (`SOIL_RAW_MIN`, `SOIL_RAW_MAX`).

#### **test_adc.c**
- Initialization test: Confirms `adc_initialized()` returns true after init.
- Sampling test: Calls `adc_manual_sample()`, checks for `ERROR_CODE` or valid 12-bit range.
- **Requires ADC input connected** to stable voltage (0–3.3V) or sensor.
- If ADC is floating or hardware fails, test detects and reports `ERROR_CODE`.

#### **test_state_machine.c**
- 7 state name tests: Validates `water_state_to_string()` enum conversion.
- 4 threshold tests: Validates `DRY_THRESHOLD`, `WET_THRESHOLD`, etc. constants.
- **No hardware required** (pure logic test).

### Running Tests

1. **Enable tests in main.c:**
   ```c
   #define RUN_TESTS
   ```

2. **Build and flash:**
   ```
   # Build via STM32cubeIDE
   # Flash via STM32CubeIDE or ST-Link CLI
   ```

3. **Monitor serial output at 9600 baud:**
   - Observe test execution with PASS/FAIL results.
   - Check heartbeat LED: ON = all pass, OFF = failures detected.

4. **Disable tests for production:**
   ```c
   //#define RUN_TESTS
   ```
   - Zero test overhead in final binary.

## Integration Testing Plan

### Prerequisites

**Hardware Setup:**
- STM32F091RC NUCLEO board.
- Resistive soil moisture sensor wired to ADC1_IN0 (PA0) with common ground.
- Servo motor (e.g., SG90) wired to:
  - PWM signal: PA7 (TIM3_CH2)
  - Power: 5V
  - Ground: Common with MCU
- Optional: Potentiometer on ADC input for simulating sensor voltage.

### General Build & Flash Steps

```
# Build using STM32CubeIDE
# Flash using STM32CubeIDE or ST-Link CLI
# Open serial monitor: 9600 baud, 8-N-1
```

### Test Phases

#### **Phase 1: Unit Tests (No Hardware)**
- Enable `#define RUN_TESTS` in `main.c`.
- Run tests for PWM, soil sensor conversion, and state machine.
- Verify: LED on = all pass, serial logs show PASS for each test.
- **Expected Result**: 100% pass rate (unless ADC test detects unconnected sensor).

#### **Phase 2: ADC & Sensor Validation**

**Setup:**
- Connect soil moisture sensor to PA0 (ADC1_IN0).
- Connect common ground between sensor and MCU.

**Test Steps:**
1. Flash with unit tests disabled (`//#define RUN_TESTS`).
2. Open serial monitor (9600 baud).
3. Observe initial log: "Initialization Complete".
4. Watch soil readings as they appear in main loop.
5. Manually change sensor wetness (dip in water or let dry) and observe ADC values change.

**Expected Behavior:**
- Dry sensor: ADC reads ~1–500.
- Wet sensor: ADC reads ~2000–4095.
- No `ADC Error` messages in logs.

**Troubleshooting:**
- If ADC values are all `0xFFFF` or unchanging: check wiring, ground, and VREF.
- If ADC is erratic: verify stable power supply and add a decoupling capacitor (0.1 µF) near sensor.

#### **Phase 3: Servo Control**

**Setup:**
- Connect servo to PA7 (PWM) with power and ground.
- Disable soil sensor if not ready (comment out `soil_sensor_init()` in `main.c`).

**Test Steps:**
1. Flash firmware.
2. Observe servo: Should start at neutral position (~1500 µs = ~90°).
3. Manually trigger state machine to WATER_OPEN_VALVE (e.g., by logging decision code).
4. Watch servo ramp smoothly to 270° (open valve).
5. Trigger WATER_CLOSE_VALVE: servo ramps back to 0°.

**Expected Behavior:**
- Smooth, audible servo movement (not jerky).
- No servo buzzing or stalling.
- Ramp rate: ~10 µs per 20 ms tick = ~500 µs/sec → ~2 sec to full range.

**Troubleshooting:**
- Servo doesn't move: Check PA7 wiring, servo power, and timer initialization logs.
- Servo jerky or incomplete: Verify PWM frequency is 50 Hz and pulse width is 500–2500 µs.

#### **Phase 4: Full System Integration**

**Setup:**
- Sensor and servo both wired.
- `#define RUN_TESTS` disabled.
- Main loop running continuously.

**Test Procedure:**
1. **Dry test:** Place sensor in dry soil. Watch FSM transition to IDLE → MEASURE → WAIT → DECIDE. If soil < DRY_THRESHOLD, FSM opens valve (servo moves to 270°).
2. **Wet test:** Simulate watering by gradually wetting sensor (or connecting a test voltage that increases ADC reading). Watch FSM transition to WATERING state with fast 300 ms sampling. When soil > WET_THRESHOLD, FSM closes valve (servo moves to 0°) and returns to IDLE.
3. **Edge cases:**
   - Soil extremely dry (< DYING_OF_THIRST_THRESHOLD): FSM logs warning.
   - Soil extremely wet (> SOAKING_THRESHOLD): FSM logs warning.
   - ADC fails (e.g., disconnect sensor): FSM detects ERROR_CODE, logs error, aborts measurement and returns to IDLE (safe state).

**Expected Sequence:**
```
Initialization Complete
Main Loop Starting
Transition: UNKNOWN -> WATER_IDLE
Soil Reading: 850
Transition: WATER_IDLE -> WATER_MEASURE
Transition: WATER_MEASURE -> WATER_WAIT_MEASURE
Soil Reading: 850
Transition: WATER_WAIT_MEASURE -> WATER_DECIDE
Soil is dry -> start watering
Transition: WATER_DECIDE -> WATER_OPEN_VALVE
Opening valve!
Transition: WATER_OPEN_VALVE -> WATER_WATERING
Watering reading: 1050
Watering reading: 1600
Watering reading: 2150
Soil is wet -> stopping water
Transition: WATER_WATERING -> WATER_CLOSE_VALVE
Closing valve...
Transition: WATER_CLOSE_VALVE -> WATER_IDLE
```

**Performance Metrics:**
- **Measurement latency**: ~50 ms from request to ready (sensor stabilization).
- **Sampling intervals**: 30 sec idle, 300 ms during watering.
- **Valve response time**: ~2 sec servo ramp from closed to open.

### Safety Notes

- **Valve actuation**: Start with conservative angles and test in a safe environment (no plants/water damage initially).
- **Power supply**: Ensure servo and MCU share common ground. Use separate 5V supply for servo if MCU 3.3V is weak.
- **Sensor wiring**: Keep leads short and twisted to minimize noise. Use a resistor pull-up if needed.
- **Timeout protection**: All blocking waits in ADC have loop-iteration timeouts to prevent system hang.

## Error Handling & Sentinel Values

**ADC Error Sentinel:**
- Invalid reads return `ERROR_CODE (0xFFFFU)`.
- Callers (soil FSM, state machine) detect this and abort measurement safely.
- Root causes: ADC init failure, conversion timeout, or ADC not initialized.

**Log-based Diagnostics:**
- All errors logged with `LOG()` at 9600 baud.
- Monitor serial output in real-time to diagnose issues.

## Next Steps

1. **Verify build**
2. **Flash firmware** and test each phase sequentially.
3. **Tune calibration constants** (`SOIL_RAW_MIN`, `SOIL_RAW_MAX`, thresholds) based on your specific sensor.
4. **Deploy on actual plants** once integration tests pass.

