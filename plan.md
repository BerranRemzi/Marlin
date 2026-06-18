# Smart Hotend via CH32V003 — Implementation Plan

Status: **Draft / Pre-implementation**
Date: 2026-06-18
Branch: `2.0.x-my-bear-upgrade`

## 1. Goal

Replace the direct thermistor / heater PWM / part-fan / BLTouch / Z-endstop wiring on hotend 0 with a **CH32V003 "dumb I/O expander"** mounted on the toolhead, communicating with the main board (ATmega2560) over a **full-duplex UART** link.

- **Marlin keeps PID, thermal runaway, autotune, and all thermistor math.**
- **CH32V003 stays dumb**: it only relays ADC samples, applies received PWM duties, generates BLTouch servo pulses, and reports endstop state.
- **HSFan (hotend-side cooling fan) is controlled locally by the CH32V003** based on a raw-ADC threshold — Marlin never touches it.

## 2. Hardware

### 2.1 CH32V003 pinout (toolhead MCU)

| Pin | Function | Direction | Notes |
|-----|----------|-----------|-------|
| PC0 | TX_HOTEND | out | UART TX → main board RX |
| PC1 | RX_HOTEND | in | UART RX ← main board TX |
| PC3 | PWM_HSFan | out | Hotend-side fan (CH32v003-owned, raw-ADC rule) |
| PC4 | ADC_NTC | in | Thermistor analog (reported up to Marlin) |
| PC6 | DI_BL_EndStop | in | BLTouch Z-min trigger signal |
| PC7 | DO_BL_Signal | out | BLTouch servo control (deploy/stow/pulse) |
| PD1 | SWD | — | Debug/flash only, never repurposed |
| PD2 | DI_X_EndStop | in | X-axis endstop (toolhead-mounted, reported to Marlin) |
| PD4 | DO_LED_HOTEND | out | Hotend LED (optional, Marlin-controlled via command) |
| PD5 | DI_Z_EndStop | in | Z-axis endstop (bed level / mechanical) |
| PA1 | PWM_Heater | out | Heater MOSFET PWM (duty from Marlin) |
| PA2 | PWM_PFan | out | Part cooling fan (duty from Marlin, M106) |

> **PD6/PD7 are not used.** BLTouch is on PC6/PC7 (servo + Z-min). Z-axis endstop is PD5. X-axis endstop is PD2 (toolhead-mounted — the toolhead strikes the X=0 endstop, so it's natural to read it on the toolhead MCU).

### 2.2 Main board (ATmega2560) side

- **Use USART2** (`SMARTHOTEND_SERIAL_PORT 2`). Confirmed free in this config:
  - USART0 = host serial (`SERIAL_PORT 0`). In use.
  - USART1 = `MALYAN_LCD` only — disabled (`//#define MALYAN_LCD`). Free.
  - USART2 = `PRUSA_MMU2` only — disabled (`//#define PRUSA_MMU2`). **Free → chosen.**
  - USART3 = `ANYCUBIC_LCD` only — disabled. Free (spare).
- On ATmega2560, USART2 is on pins **PH1 (TXD2)** and **PH0 (RXD2)** — check your board's pin mapping / AUX header for the physical pads.
- UART pins on the main board: PH1 (TX) → CH32v003 RX (PC1), PH0 (RX) ← CH32v003 TX (PC0).
- **Baud: 115200** (plenty; 6-byte frame ≈ 0.5 ms wire time).
- 4 wires in the toolhead cable: TX, RX, GND, +5V.

### 2.3 Power & pull resistors

- **CH32v003 powered at 5V** (from the toolhead 5V rail). Decision locked. Verify your specific CH32v003 part/variant is rated for 5V VDD (official CH32v003 is 3.3V; some variants/clones tolerate 5V). At 5V:
  - ADC reference = 5V → size the NTC divider for 0–5V range.
  - UART levels match the ATmega2560 directly — **no level shifting needed**.
- **All digital outputs have a pull-down** so that reset/flash/power-up = safe OFF:
  - PA1 (PWM_Heater) — pull-down → heater OFF on reset. ✅
  - PA2 (PWM_PFan) — pull-down → part fan OFF on reset.
  - PC7 (DO_BL_Signal) — pull-down → BLTouch stowed/idle on reset.
  - PD4 (DO_LED_HOTEND) — pull-down → LED off on reset.
- **HSFan (PC3) has a pull-UP** → hotend-side fan **ON** on reset. This is the safe default: if the CH32v003 dies, the heatsink fan stays on to protect the hotend/heatsink. The CH32v003 drives it low/PWM to control; on reset the pull-up keeps the fan running.
- NTC divider: **NTC100 to GND, 4.7kΩ pullup to 5V**, ADC pin at the junction. Identical to the real board's thermistor circuit → raw ADC values match for a given temperature.

### 2.4 Safety approach — comms loss is the protection

- **No thermal fuse.** By design, stock hotends don't have one; the **comms-loss watchdog is the protection layer.**
- If the UART link dies (cable cut, either MCU reset/crash), both sides fail-safe:
  - **CH32v003:** no valid command for 1 s → HEATER_PWM = 0 (pull-down also guarantees this on reset). HSFan stays ON (pull-up + local rule).
  - **Marlin:** no valid telemetry for 1 s → stop heater, flag temp sensor fault, halt print.
- This dual watchdog replaces a thermal fuse for the comms-failure case. **Note:** it does *not* protect against a shorted/faulty MOSFET that drives the heater on independently of the MCU — that's a hardware failure mode outside the comms watchdog's scope. Accept this as an explicit design decision.
- CH32v003 brown-out detection enabled (so a sagging 5V rail resets the MCU cleanly → heater off via pull-down).

## 3. Communication — UART, async streaming + heartbeat

### 3.1 Why UART

- CPU load on ATmega2560 @ 16 MHz with interrupt-driven hardware USART + ring buffer: **~0.3–0.7%** at 250 Hz telemetry. Negligible.
- Full-duplex, deterministic, robust over a moving cable, no I2C bus-lockup risk.
- Marlin already consumes sensor data (like MAX6675); async streaming fits that model.

### 3.2 Mode: async streaming + event-driven commands + 1 s heartbeat

- **CH32v003 → Marlin (telemetry):** autonomous, fixed rate **250 Hz** (every 4 ms).
- **Marlin → CH32v003 (commands):** event-driven — sent only when PWM/BLTouch/LED changes.
- **Heartbeat:** if Marlin has **no change** to send for **1 s**, it re-sends the last command frame anyway. This keeps the CH32v003's "Marlin alive" watchdog fed without spamming the bus. (Requirement from user.)

### 3.3 Frame format

#### Telemetry frame (CH32v003 → Marlin, 6 bytes, 250 Hz)

```
Offset  Field           Size  Description
 0      SYNC            1     0x55
 1      ADC_HI          1     Raw ADC high byte (10-bit, bits 9..2)
 2      ADC_LO          1     Raw ADC low byte  (10-bit, bits 1..0 in upper nibble)
 3      ENDSTOP_FLAGS   1     bit0: Z endstop (PD5)   1=triggered
                              bit1: BLTouch trigger (PC6)  1=triggered
                              bit2: X endstop (PD2)   1=triggered
                              bit3: reserved
 4      STATUS          1     bit0: comms-ok (last Marlin frame < 1s ago)
                              bit1: reserved (was ADC fault — removed; see §3.5)
                              bit2: watchdog tripped (heater forced off)
                              bit3: reserved
 5      CRC8            1     CRC-8 (poly 0x07, init 0x00) over bytes 0..4
```

- **ADC is 10-bit (0–1023)** — the CH32v003 replicates the exact thermistor circuit from the real board and reads a 10-bit ADC, matching the ATmega2560's `HAL_ADC_RESOLUTION = 10` / `HAL_ADC_RANGE = 1024`. This keeps Marlin's thermistor tables valid unchanged.
- **Scaling to Marlin's raw units:** Marlin's ISR accumulates `OVERSAMPLENR` (=16 on AVR, unless `HAL_ADC_FILTERED`) ADC samples into `temp_hotend[].raw`, so raw is in range 0–16383 (`MAX_RAW_THERMISTOR_VALUE = HAL_ADC_RANGE × OVERSAMPLENR - 1`). The CH32v003 sends a single 10-bit sample (0–1023); `SmartHotend::readRawADC()` returns `adc_10bit × OVERSAMPLENR` (×16) to match what `ACCUMULATE_ADC` would have produced. See §3.5 for the full scaling rationale.
- Marlin converts raw → °C via its normal thermistor table (`analog_to_celsius_hotend`).

#### Command frame (Marlin → CH32v003, 5 bytes, event-driven + 1 s heartbeat)

```
Offset  Field           Size  Description
 0      SYNC            1     0xAA
 1      HEATER_PWM      1     0..255 duty (0 = OFF), applied over a 1 s slow-PWM window
 2      PFAN_PWM        1     0..255 part cooling fan duty
 3      BL_CMD          1     0=idle, 1=deploy, 2=stow, 3=pulse-test, 4=self-test
 4      CRC8            1     CRC-8 (poly 0x07, init 0x00) over bytes 0..3
```

- HSFan is **not** in this frame — CH32v003 owns it.
- LED is **not** in this frame — CH32v003 owns it (PD4 driven locally).
- Fixed length both directions (telemetry = 6 bytes, command = 5 bytes) → no length byte, trivial to parse.

### 3.5 NTC fault handling & ADC scaling

**ADC scaling (critical to get right):**
- The CH32v003 **replicates the exact thermistor circuit** (same pullup, same NTC, same divider) as the original board, and reads a **10-bit ADC (0–1023)** — identical to the ATmega2560's `HAL_ADC_RESOLUTION = 10` / `HAL_ADC_RANGE = 1024`.
- Marlin's thermistor tables are built for 10-bit (`THERMISTOR_TABLE_ADC_RESOLUTION = 10`) with `THERMISTOR_TABLE_SCALE = HAL_ADC_RANGE / 1024 = 1` on AVR, so a 10-bit sample maps directly.
- Marlin's temperature ISR accumulates `OVERSAMPLENR` (= **16** on AVR unless `HAL_ADC_FILTERED` is set) consecutive ADC samples into `temp_hotend[].raw`. So the raw value Marlin expects is in range **0–16383** (`MAX_RAW_THERMISTOR_VALUE = 1024 × 16 - 1`), not 0–1023.
- **Contract:** the CH32v003 sends a single 10-bit sample (0–1023) in the telemetry frame. `SmartHotend::readRawADC()` on the Marlin side returns `adc_10bit × OVERSAMPLENR` (×16) so the injected raw matches what `ACCUMULATE_ADC` would have produced for a local sensor. This makes the smart hotend indistinguishable from a local thermistor to Marlin's thermistor math.
- If `HAL_ADC_FILTERED` is ever enabled (`OVERSAMPLENR = 1`), the multiplier becomes ×1 — `SmartHotend::readRawADC()` must read `OVERSAMPLENR` at compile time, not hardcode 16.

**NTC fault handling (no explicit ADC fault bit):**
- On NTC open/short, the CH32v003 sends the **max 10-bit value (0x3FF = 1023)** in the telemetry frame. Marlin scales it to `1023 × 16 = 16368`, which the thermistor table maps to a very high temperature → trips **`max_temp_error(H_E0)`** via the existing `HEATER_0_MAXTEMP` check.
- No custom fault path or `adcFault()` bit needed on the Marlin side — reuses Marlin's native max-temp error.
- The CH32v003 still turns its heater PWM off locally on ADC fault (its own safety), but signals it to Marlin purely via the max-value ADC.
- STATUS bit1 is reserved (was ADC fault).

### 3.4 Timing & watchdogs

| Watchdog | Timeout | Action |
|----------|---------|--------|
| CH32v003: no valid command frame from Marlin | **1000 ms** | HEATER_PWM=0, PFAN_PWM=0. HSFan stays ON (pull-up + local rule keeps heatsink cooled). Set STATUS bit2. |
| Marlin: no valid telemetry frame from CH32v003 | **1000 ms** | Stop sending heater PWM (send 0), flag `TempSensor_0` fault, halt print via existing `max_temp_error`/`min_temp_error`/`TempError` path. |
| CH32v003: ADC pegged at 0 or max | **500 ms** | HEATER_PWM=0 locally. Send max 10-bit ADC (1023) in telemetry → Marlin scales to 16368 → native `max_temp_error` trips. (No dedicated status bit — see §3.5.) |
| Marlin heartbeat | every **1000 ms** when idle | Re-send last command frame to keep CH32v003 watchdog fed. |

- On CH32v003 **power-up/reset**: all PWM = 0 until first valid command frame.
- On Marlin **startup**: send a zero-duty command frame immediately so CH32v003 leaves reset state cleanly.

## 4. CH32v003 firmware (dumb I/O expander)

### 4.1 Responsibilities

1. Sample PC4 ADC at ≥1 kHz, 10-bit (0–1023), keep latest value. (Circuit replicates the real board's thermistor divider.)
2. Sample PD5 (Z endstop), PC6 (BLTouch trigger), and PD2 (X endstop) every loop iteration.
3. Generate PA1 heater PWM at **1 Hz slow PWM** (0–255 duty applied over a 1 s window, replicating Marlin's soft-PWM), PA2 part-fan PWM from received duty (hardware timer PWM).
4. Generate PC3 HSFan PWM locally from raw-ADC threshold (hysteresis):
   - `adc < HOT_THRESHOLD` → HSFan = 255 (NTC: low ADC = hot)
   - `adc > COLD_THRESHOLD` → HSFan = 0
   - between → keep previous (hysteresis)
5. On `BL_CMD`: drive PC7 servo pulse (timer PWM, ~1 kHz, pulse width per command), then return to idle.
6. Drive PD4 LED locally (e.g. on when heater active) — not controlled by Marlin.
7. UART RX interrupt → parse command frame → update duties/commands.
8. Timer (4 ms / 250 Hz) → assemble & TX telemetry frame.
9. Watchdog: 1 s no valid command → all output PWM = 0.
10. ADC fault: pegged value 500 ms → heater off locally, and send max 10-bit ADC value (1023) in telemetry so Marlin's native max-temp error trips. No dedicated status bit needed.

### 4.2 BLTouch pulse timing (mirror Marlin's `bltouch.cpp`)

| Command | Pulse width on PC7 |
|---------|-------------------|
| idle (0) | no pulse / servo off |
| deploy (1) | ~1.95 ms (SWIM_BLTOUCH_DEPLOY) |
| stow (2) | ~0.92 ms (SWIM_BLTOUCH_STOW) |
| pulse-test (3) | ~1.45 ms short pulse |
| self-test (4) | sequence of pulses |

Pulse held for ~5–10 ms then servo signal released. CH32v003 uses a hardware timer for the servo PWM.

### 4.3 Firmware structure (sketch)

```
ch32v003_hotend/
  main.c              - init, main loop (sample ADC/endstops, apply watchdog)
  uart.c/h            - USART RX/TX interrupt driver, ring buffer, frame parse/assemble
  pwm.c/h             - timer PWM for heater, part fan, HSFan, BLTouch servo
  adc.c/h             - ADC sampling (10-bit, matching ATmega2560)
  endstop.c/h         - PD2/PD5/PC6 sampling + debounce (X, Z, BLTouch)
  protocol.h          - frame format, sync bytes, CRC8, BL_CMD enum, STATUS bits
  watchdog.c/h        - 1s comms watchdog, ADC fault detector
  config.h            - pin map, thresholds, baud rate
```

> CH32v003 firmware lives outside this Marlin repo (separate project). This plan covers the Marlin side; the CH32v003 side is specified here for contract agreement.

## 5. Marlin integration

### 5.1 New feature: `SmartHotend`

New files:
- `Marlin/src/feature/smarthotend/SmartHotend.h`
- `Marlin/src/feature/smarthotend/SmartHotend.cpp`

Class owns:
- UART driver (hardware USART + RX/TX ring buffers, interrupt-driven).
- Latest telemetry cache: `raw_adc`, `endstop_flags`, `status`.
- Outbound command cache: `heater_pwm`, `pfan_pwm`, `bl_cmd`.
- Heartbeat timer (1 s).
- Comms watchdog (1 s → fault).

```cpp
class SmartHotend {
  public:
    static void init();              // setup USART, pins, buffers
    static void tick();              // called from Temperature ISR; parse RX, update cache
    static uint16_t readRawADC();    // latest thermistor raw (10-bit × OVERSAMPLENR → Marlin raw units)
    static bool zEndstopTriggered();
    static bool xEndstopTriggered();
    static bool bltouchTriggered();
    static void setHeaterPWM(uint8_t duty);
    static void setPartFanPWM(uint8_t duty);
    static void setBLTouchCommand(uint8_t cmd);
    // NOTE: No HSFan API — the CH32v003 owns the heatsink fan entirely.
    //       It is controlled locally via a raw-ADC threshold and never appears
    //       in the command frame. Marlin cannot and does not control it.
    // NOTE: No LED API — PD4 hotend LED is owned by the CH32v003 (local rule).
    static bool commsOk();
    static void sendCommand();       // assemble + TX command frame (event or heartbeat)
  private:
    static void parseTelemetry();
    static uint8_t crc8(const uint8_t *p, uint8_t len);
    // ...cached state, buffers, timers
};
```

### 5.2 Configuration.h additions

```cpp
// Smart Hotend (CH32v003 over UART)
#define SMARTHOTEND_ENABLED
#if ENABLED(SMARTHOTEND_ENABLED)
  #define SMARTHOTEND_SERIAL_PORT 2      // USART2 on ATmega2560
  #define SMARTHOTEND_BAUD 115200
  #define SMARTHOTEND_HEATER 0           // which hotend this drives
  #define SMARTHOTEND_TEMP_SENSOR 1      // thermistor type for analog_to_celsius_hotend
  #define SMARTHOTEND_HEARTBEAT_MS 1000
  #define SMARTHOTEND_WATCHDOG_MS 1000
#endif
```

`TEMP_SENSOR_0` stays at its current value (`1` = EPCOS 100K) — **no change needed**. The thermistor circuit is replicated on the hotend with the same NTC100, so raw ADC values are identical for a given temperature. Marlin still owns the thermistor table; the CH32v003 just sends raw ADC.

### 5.3 Integration into `temperature.cpp` (mirror MAX6675 pattern)

The MAX6675 is the existing model for "external temp source injects raw value":

1. **Raw temp injection** — in `updateTemperaturesFromRawValues()`:
   ```cpp
   #if ENABLED(SMARTHOTEND_ENABLED)
     temp_hotend[SMARTHOTEND_HEATER].raw = SmartHotend::readRawADC();
   #endif
   ```
   (analogous to `temp_hotend[0].raw = READ_MAX6675(0);`)

2. **ADC sampling skip** — when `SMARTHOTEND_ENABLED`, do **not** start ADC for `TEMP_0_PIN` in the ISR `PrepareTemp_0`/`MeasureTemp_0` cases (the pin isn't wired). Guard with `#if HAS_TEMP_ADC_0 && DISABLED(SMARTHOTEND_ENABLED)` or similar.

3. **Heater PWM write** — where Marlin writes `soft_pwm_0` to the heater pin, route to `SmartHotend::setHeaterPWM()` instead:
   ```cpp
   #if ENABLED(SMARTHOTEND_ENABLED)
     SmartHotend::setHeaterPWM(soft_pwm_hotend[0]);
   #else
     WRITE(HEATER_0_PIN, ...);
   #endif
   ```

4. **Part fan** — in fan write path, route to `SmartHotend::setPartFanPWM()`.

5. **`tick()` call** — invoke `SmartHotend::tick()` from the temperature ISR (same place as `ACCUMULATE_ADC` / `readings_ready`), non-blocking.

6. **Comms fault** — if `!SmartHotend::commsOk()` for >1s, trigger `min_temp_error(H_E0)` / a `TempError` so Marlin halts (reuses existing fault machinery).

### 5.4 Endstop / BLTouch integration

- In `endstops.cpp`, when `SMARTHOTEND_ENABLED`, the toolhead-mounted endstops pull from the SmartHotend telemetry cache instead of local GPIO reads:
  - **Z-min** (bed level / mechanical): `SmartHotend::zEndstopTriggered()` instead of `READ(Z_MIN_PIN)`.
  - **X-min** (toolhead strikes X=0 stop): `SmartHotend::xEndstopTriggered()` instead of `READ(X_MIN_PIN)`.
  - **BLTouch probe trigger**: `SmartHotend::bltouchTriggered()` instead of `READ(Z_MIN_PROBE_PIN)`.
- In `bltouch.cpp`, deploy/stow/pulse commands call `SmartHotend::setBLTouchCommand(...)` instead of servo writes.
- The main board's `X_MIN_PIN`, `Z_MIN_PIN`, and `Z_MIN_PROBE_PIN` are left unused (not wired) when `SMARTHOTEND_ENABLED` — the CH32v003 owns those inputs on the toolhead.

### 5.5 SanityCheck.h

Add validation:
- `SMARTHOTEND_ENABLED` requires `SMARTHOTEND_SERIAL_PORT`, `SMARTHOTEND_BAUD`, `SMARTHOTEND_HEATER` defined.
- `SMARTHOTEND_HEATER` < `HOTENDS`.
- Warn if `TEMP_0_PIN` / `HEATER_0_PIN` are also assigned (should be unused on main board).
- Warn if `X_MIN_PIN` / `Z_MIN_PIN` / `Z_MIN_PROBE_PIN` are also assigned (should be unused on main board — owned by CH32v003).
- Ensure the chosen USART port isn't the host serial.

## 6. Implementation phases

> **Scope of this plan:** Phase 1 (contract) and Phase 2 (Marlin `SmartHotend` class + wiring) are the main goal of `plan.md`. Phase 3 (CH32v003 firmware) will be a **separate repository** created by the user; it is tracked here only for protocol contract agreement.

### Phase 1 — Protocol & contract ✅
- [x] Define frame format (this doc §3.3).
- [x] Pin assignments finalized (PD6/PD7 dropped, X endstop added on PD2).
- [x] USART port: **USART2** (free — MMU2/LCDs all disabled in this config).
- [x] CH32v003 VDD: **5V** (no level shifting, ADC ref = 5V).
- [x] NTC fault handling: CH32v003 sends max 10-bit ADC (1023) → Marlin native max-temp error (no `adcFault()` bit).
- [x] ADC scaling: 10-bit sample × `OVERSAMPLENR` (×16) = Marlin raw units (0–16383). Circuit replicated.
- [x] Thermistor type: NTC100, circuit replicated → `TEMP_SENSOR_0` stays `1` (no change).
- [x] HSFan/NTC circuit: 4.7kΩ pullup to 5V + NTC100 (same as real board).
- [x] Heater PWM: slow 1 Hz PWM on CH32v003 (replicates Marlin soft-PWM).
- [x] LED: kept local to CH32v003 (removed from command frame).
- **Phase 1 complete — all contract decisions resolved.**

### Phase 2 — Marlin `SmartHotend` class + wiring (main goal of this plan)
- [ ] Create `Marlin/src/feature/smarthotend/SmartHotend.{h,cpp}`.
- [ ] UART RX/TX ring buffer + interrupt driver on USART2.
- [ ] Telemetry parser + CRC8 + cache.
- [ ] Command assembler + event-driven TX + 1 s heartbeat.
- [ ] Comms watchdog + fault flag.
- [ ] `init()` / `tick()`.
- [ ] `Configuration.h` / `Configuration_adv.h` options.
- [ ] `temperature.cpp`: raw injection, ADC skip, heater PWM route, `tick()` call.
- [ ] `endstops.cpp`: X-min + Z-min + BLTouch trigger reads.
- [ ] `bltouch.cpp`: deploy/stow/pulse route.
- [ ] Fan write path: part fan route.
- [ ] `SanityCheck.h` validation.
- [ ] `Conditionals_post.h` / `Conditionals_LCD.h` feature flag plumbing.

### Phase 3 — CH32v003 firmware (separate repository, user-created)
- [ ] Create separate repo for CH32v003 hotend firmware.
- [ ] `protocol.h` matching this doc exactly.
- [ ] UART driver, ADC (10-bit), PWM timers (1 Hz heater, part fan, HSFan, BLTouch servo), endstop sampling.
- [ ] HSFan raw-ADC rule with hysteresis (4.7kΩ + NTC100 thresholds).
- [ ] BLTouch servo pulse generator (mirror `bltouch.cpp` timings).
- [ ] 1 s comms watchdog + ADC fault detection (send max ADC on fault).
- [ ] 250 Hz telemetry TX.
- [ ] Bench test standalone (loopback, fake Marlin).

### Phase 4 — Integration test (after Phase 2 + Phase 3)
- [ ] CH32v003 + Marlin on bench, thermistor simulator.
- [ ] Verify PID autotune works through the link.
- [ ] Verify thermal runaway triggers on comms loss (unplug UART).
- [ ] Verify BLTouch deploy/stow/probe through the link.
- [ ] Verify X-axis homing through the link (PD2 endstop).
- [ ] Verify Z-axis homing through the link (PD5 endstop).
- [ ] Verify part fan M106 through the link.
- [ ] Verify HSFan auto rule independent of Marlin.
- [ ] Verify 1 s heartbeat keeps CH32v003 alive when Marlin idle.

## 7. Open questions (resolve before Phase 2/3)

1. **BLTouch pins**: ✅ Resolved — PC6/PC7 = BLTouch (servo+Zmin), PD5 = Z endstop, PD2 = X endstop, PD6/PD7 dropped.
2. **ADC resolution**: ✅ Resolved — 10-bit (0–1023), replicating the ATmega2560's `HAL_ADC_RANGE = 1024`. `SmartHotend::readRawADC()` scales by `OVERSAMPLENR` (×16) to match Marlin's accumulated-raw units.
3. **Thermistor type**: ✅ Resolved — the thermistor circuit is replicated on the hotend with the same NTC100, so the raw ADC values are identical for a given temperature. `TEMP_SENSOR_0` stays at its current value (`1` = EPCOS 100K) — **no change needed**. Marlin's thermistor table works unchanged because the CH32v003 presents the same raw values the local ADC would have.
4. **Main board USART**: ✅ Resolved — USART2 (PH0/PH1 on ATmega2560). Free in this config.
5. **HSFan thresholds**: ✅ Resolved — 4.7kΩ pullup to 5V with NTC100 (same as the real board). The HSFan hot/cold thresholds are set in raw 10-bit ADC units in the CH32v003 firmware `config.h`. Exact values TBD from the NTC100 + 4.7k divider curve (e.g. ~50 °C → fan on, ~40 °C → fan off), but the circuit is fixed: NTC100 to GND, 4.7kΩ to 5V, ADC pin at the junction.
6. **Heater PWM mode**: ✅ Resolved — **slow PWM at 1 Hz** on the CH32v003, replicating Marlin's soft-PWM behavior. The CH32v003 receives a 0–255 duty byte and applies it over a 1 s window (same as Marlin's `soft_pwm_hotend`). This keeps MOSFET thermal cycling identical to a stock setup.
7. **LED**: ✅ Resolved — **keep PD4 hotend LED local to the CH32v003.** Not routed through the protocol. The LED is controlled by the CH32v003 directly (e.g. on when heater active, or a simple on/off rule). Removed from the command frame.
8. **CH32v003 VDD choice**: ✅ Resolved — **5V**. No level shifting; ADC ref = 5V; size NTC divider for 0–5V.

## 8. Risks & mitigations

| Risk | Mitigation |
|------|-----------|
| UART cable noise/EMI from heater | Twisted pair for TX/RX, shielded cable, keep away from heater leads. 115200 is slow enough to tolerate. |
| CH32v003 reset during print | Heater gate pull-down → OFF. HSFan pull-up → ON. Marlin sees telemetry stop → fault + halt. |
| Marlin crash/hang | CH32v003 1 s watchdog → heater OFF (pull-down). HSFan stays ON (pull-up). |
| Heater MOSFET short (hardware failure) | **Not mitigated** — no thermal fuse by design. Comms watchdog only covers MCU/comms failures, not a shorted MOSFET. Accepted as explicit design decision. |
| CH32v003 VDD | 5V chosen. Verify the specific part is 5V-rated. No level shifting needed. ADC ref = 5V. |
| ADC drift/different reference | **Mitigated by design** — CH32v003 replicates the exact thermistor circuit and reads 10-bit ADC matching ATmega2560 (`HAL_ADC_RANGE = 1024`). `SmartHotend::readRawADC()` scales by `OVERSAMPLENR` (×16) to match Marlin's accumulated-raw units. No table mismatch. |
| Bus contention / framing errors | Fixed-length frames + sync byte + CRC8; invalid CRC → drop, don't act. |
| BLTouch timing mismatch | Mirror Marlin's `bltouch.cpp` pulse widths exactly in CH32v003 firmware. |

> **ADC scaling note (resolved):** Marlin's thermistor code expects `raw` in units of *summed oversampled ADC counts* (`HAL_ADC_RANGE × OVERSAMPLENR`). On AVR: `HAL_ADC_RANGE = 1024` (10-bit), `OVERSAMPLENR = 16` → raw range 0–16383. The CH32v003 replicates the real thermistor circuit and reads a 10-bit ADC (0–1023), so `SmartHotend::readRawADC()` returns `adc_10bit × OVERSAMPLENR` (×16). This makes the smart hotend indistinguishable from a local thermistor. The multiplier is read from `OVERSAMPLENR` at compile time (not hardcoded) so it stays correct if `HAL_ADC_FILTERED` changes it to 1.

## 9. Out of scope (for now)

- Multi-hotend / toolchanger support (protocol is point-to-point single hotend).
- I2C mode (UART chosen; PC1/PC2 I2C pins left unused).
- HSFan control from Marlin (CH32v003 owns it).
- LED control from Marlin (CH32v003 owns PD4 locally).
- PID on CH32v003 (Marlin owns it).