# Testing

## Test layers

| Layer | What's tested | How to run |
|-------|--------------|------------|
| **Native unit tests** | Pure math (Rodrigues rotation, quantum gates, QuatTracker, BLE command parser) | `pio test -e native_test` |
| **Wokwi simulation** | Full firmware compile + runtime | Wokwi CLI / VS Code extension |
| **On-device** | Motor calibration, real BLE, hardware integration | Flash + BLE terminal / Python script |

---

## Native unit tests

28 tests covering:
- **Rodrigues rotation** — identity, X/Y/Z/H/S/T gates, norm preservation, gate sequences
- **QuatTracker** — rotation accumulation, calculateReset, shortest path, H²=I
- **Motor matrix** — row norms, Z-rotation symmetry, wheel symmetry
- **BLE command parser** — single/multi-command CSV, `led`/`pulse` with arguments, empty input

### Run (from project root)

On Windows, MSYS2's gcc requires its own bin dirs on PATH before running pio:

```powershell
$env:PATH = "C:\msys64\ucrt64\bin;C:\msys64\usr\bin;" + $env:PATH
pio test -e native_test
```

Or add those directories permanently to your system PATH so you don't need to prepend them each time.

### Adding tests

New test files go in `test/test_native/`. New shared math utilities go in `src/` as header-only files (no Arduino/hardware includes). The `test/mock/Arduino.h` provides `PI`, `millis()`, and a stub `Serial` for any math headers that transitively include Arduino.

---

## Wokwi simulation

The `wokwi_sim` PlatformIO environment compiles the full firmware with `-D WOKWI`.
A `wokwi.toml` is already present pointing at the built binary.

To run:
1. Build: `pio run -e wokwi_sim`
2. Open in [Wokwi](https://wokwi.com) or use the [Wokwi CLI](https://docs.wokwi.com/wokwi-ci/getting-started)

The Wokwi simulator lets you test:
- The full state machine (`RESET → IDLE → MOVE_START → MOVE_BUSY → MOVE_END`)
- BLE UART simulation (inject bytes into the RX characteristic)
- Display output via LVGL
- Stepper motor step counts

To use Wokwi CI headlessly (e.g. in GitHub Actions), add a `diagram.json` describing the M5Stack CoreS3 wiring and use `wokwi-cli test`.

---

## On-device / BLE integration

Connect via BLE serial (e.g. [nRF Connect](https://www.nordicsemi.com/Products/Development-tools/nRF-Connect-for-mobile) or a Python script with `bleak`) and send newline-terminated command strings:

```
Xgate\n
led,255,0,0\n
Xgate,Ygate,reset\n
```

The device echoes each processed token back over BLE TX. A simple Python smoke-test script using `bleak` can verify all gate commands are echoed and the reset command produces no error response.

---

## Known environment note

MSYS2's `g++` requires `C:\msys64\ucrt64\bin` on PATH to spawn sub-processes (`cc1plus`, `collect2`). PlatformIO picks up the system PATH, so if you start PlatformIO from a shell where MSYS2 is not on PATH the native build will fail silently with exit code 1 and no error output.

## Motor stutter timing capture

Build and upload the diagnostic firmware, then capture USB serial at 115200 baud:

```powershell
pio device list
pio device monitor --port COM5 --baud 115200 --filter log2file
```

Replace COM5 with the device port. The monitor saves a timestamped log in the
current directory; stop with Ctrl+C. Close any other serial monitor first.

Capture 20 seconds idle, 30 seconds of repeated ordinary moves with no touching,
then the same moves while using the UI. Also capture 30 seconds of wander without
and with UI activity. Note the approximate time of each phase and any visible
stutter, plus the speed setting. Send the log and these notes for analysis.

Every two seconds `[timing]` reports GUI/receiver maximum durations and motor-core
service gaps in microseconds. `motor_samples=0` means no active-motion samples.
`motor_over_budget` counts gaps above the 66 us maximum-speed budget (15000 steps/s);
`motor_over_1ms` counts gaps above 1 ms. `motor_gap_max_us` is the longest gap.
Idle sleeps are excluded. Motion transitions can still appear in these measurements.
Counters are independent snapshots, so boundary events can fall in adjacent reports.

These are scheduling diagnostics, not measured step pulses or proof of lost physical
steps. Lower speeds have longer step periods, and instrumentation adds some overhead.
Compare equivalent movements and speeds. Wander currently yields for one scheduler
tick after steering updates; these pauses should be visible in the motor measurements.
Do not remove that yield without providing another way to service the idle watchdog.
If timing is clean but stutter remains, inspect step pulses with a logic analyzer
and investigate the driver, power supply, and mechanical traction.
