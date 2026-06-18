# OzoneTrckr

A merge of **ozone_v2** (UV/ozone measurement instrument) and **sunflower** (LDR sun
tracker), running on the Adafruit ESP32-S2 TFT Feather. The instrument points itself at
the sun before and during a measurement run.

## Sun-tracker integration

Self-contained module `sunflower.h` / `sunflower.cpp`, imported like `datalink` /
`SolarCalculator`. It owns its two servos and four LDRs.

**API**
- `sunflowerBegin(hPin, vPin, ldrLT, ldrRT, ldrLD, ldrRD)` — attach servos, move to rest position.
- `sunflowerFindSun(timeoutMs)` — step onto the sun until locked, or bail after the timeout.
  Returns the instant it locks; on timeout it holds the last position.

**Pins** (macros in `ozone_trckr.cpp`, passed into the module — macros don't cross `.cpp` files):
- Servos: `TRK_H_PIN = 12` (horizontal/azimuth), `TRK_V_PIN = 13` (vertical/elevation, MG90 metal gear)
- LDRs: `LDR_LT = A3`, `LDR_RT = A1`, `LDR_LD = A0`, `LDR_RD = A2`

## Behavior

- **Boot:** splash animation disabled; `sunflowerBegin()` + `sunflowerFindSun(30000)` find and hold
  the sun (30 s cap so a cloudy/blind start can't hang boot).
- **Loop:** every 10th iteration calls `sunflowerFindSun(3000)` to re-aim. Tracking **never runs
  inside a measurement block**, so the servos add no movement noise to the ADC.
- **Timing:** one loop iteration ≈ 22 s (20 measurement blocks ≈ 1 s each + 2 filter rotations
  ≈ 1 s). With `MAX_SUBLISTS = 2000` (~100 iterations) it re-aims ~10× per run.

## Control: proportional + capped step (not PID — deliberate)

- Step grows with how far off the sun is (fast acquisition) but is **hard-capped by `MAX_STEP`**, so
  the heavy mount never gets a momentum kick. (A big step once snapped the original plastic vertical
  gear; it's now MG90 metal.)
- `MIN_STEP` keeps it inching the last bit → precise lock without a PID integrator that can wind up.
- Sub-degree moves via `writeMicroseconds` → smooth. `STEP_MS` cadence + `TOL` deadband damp hunting.
- Pulse range pinned to 1000–2000 µs so the servo can't ram the gear stops.
- PID would need output rate-limiting to be gear-safe anyway; for a find-and-hold (not continuous
  tracking) use case, capped-proportional bakes in the safety and precision with fewer failure modes.

**Tuning knobs** (top of `sunflower.cpp`): `KP`, `MIN_STEP`, `MAX_STEP`, `TOL`, `STEP_MS`,
`SETTLE_N`, and the `H_MIN/MAX` / `V_MIN/MAX` travel limits.

## Caveats to verify on the real rig

1. **Direction signs are from the bench.** The quadrant→axis mapping and the `+=`/`-=` directions
   are guesses. If an axis runs *away* from the sun, flip that axis's sign (or swap its two LDR pins).
2. **LDR pins A0–A3 are ADC2** (GPIO18/17/16/15) on the ESP32-S2. `analogRead` on ADC2 only works
   while WiFi is off — fine here, since tracking never overlaps the WiFi upload. Don't add tracking
   during upload.
3. **Three servos now share ESP32Servo** (filter on pin 5, tracker on 12/13). Timers are auto-allocated;
   if the filter servo ever misbehaves, check there first.
4. **Keep `MAX_STEP` modest** — heavy mount, MG90 vertical servo.

## Other features carried over

- **BME280** (temp/humidity/pressure) captured at measurement start.
- **CSV upload** has a 3-row metadata header (time/date, coordinates, environment) before the data
  rows. Coordinates come from the first live GPS fix (fallback: last-known position from flash).
- **GPS** is serviced every measurement block, with a 4 KB UART RX buffer — fixes a starved-parser
  bug (symptom: chars received climbing but 0 fixes parsed).
- **Offset calibration** uses a gap-watchdog timeout (the old total-time budget guaranteed a false
  "ADC timeout": 100 samples need ~5 s but the budget was 3 s).
