/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */
#pragma once

/**
 * feature/smarthotend/SmartHotend.h
 *
 * Smart Hotend driver — talks to a CH32V003 "dumb I/O expander" on the toolhead
 * over a full-duplex UART link. The CH32V003 owns:
 *   - Thermistor ADC (10-bit, replicated circuit) — reported up to Marlin.
 *   - Heater PWM (1 Hz slow PWM, duty from Marlin).
 *   - Part cooling fan PWM (duty from Marlin).
 *   - BLTouch servo control (deploy/stow/pulse commands from Marlin).
 *   - Heatsink fan (local raw-ADC rule, NOT controlled by Marlin).
 *   - Hotend LED (local, NOT controlled by Marlin).
 *   - X / Z / BLTouch endstop inputs — reported up to Marlin.
 *
 * Marlin keeps PID, thermal runaway, autotune, and all thermistor math.
 * See plan.md for the full protocol contract.
 *
 * Protocol (async streaming + event-driven commands + 1 s heartbeat):
 *   Telemetry (CH32V003 → Marlin, 6 bytes, 20 Hz + endstop changes):
 *     [0x55] [ADC_HI] [ADC_LO] [ENDSTOP_FLAGS] [STATUS] [CRC8]
 *   Command (Marlin → CH32V003, 5 bytes, event-driven + 1 s heartbeat):
 *     [0xAA] [HEATER_PWM] [PFAN_PWM] [BL_CMD] [CRC8]
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(SMARTHOTEND_ENABLED)

// --------------------------------------------------------------------------
// Protocol constants
// --------------------------------------------------------------------------

#define SH_TLM_SYNC       0x55    // Telemetry frame sync byte
#define SH_CMD_SYNC       0xAA    // Command frame sync byte
#define SH_TLM_LEN        6       // Telemetry frame length (bytes)
#define SH_CMD_LEN        5       // Command frame length (bytes)

// BLTouch command codes (BL_CMD field)
#define SH_BL_IDLE        0
#define SH_BL_DEPLOY      1
#define SH_BL_STOW        2
#define SH_BL_PULSE_TEST  3
#define SH_BL_SELF_TEST   4

// ENDSTOP_FLAGS bits
#define SH_ES_Z_BIT       0       // Z endstop (PD5)
#define SH_ES_BL_BIT      1       // BLTouch trigger (PC6)
#define SH_ES_X_BIT       2       // X endstop (PD2)

// STATUS bits
#define SH_ST_COMMS_OK    0       // CH32V003 received a command < 1 s ago
#define SH_ST_WD_TRIPPED  2       // CH32V003 watchdog tripped (heater forced off)

// Sentinel thermistor value used when telemetry is absent (comms loss, boot
// before first frame): ADC pegged at full scale (5 V on the NTC divider).
// Matched to the CH32V003 contract of reporting 0x3FF on its local ADC fault
// (plan.md §3.5); tables map it to their coldest entry (e.g. -15 °C for the
// EPCOS NTC100 table), so Marlin's native MINTEMP path reports the missing
// sensor like an unplugged thermistor.
#define SH_ADC_NO_SENSOR_10BIT 1023

// --------------------------------------------------------------------------
// SmartHotend class
// --------------------------------------------------------------------------

class SmartHotend {
  public:
    // Setup USART, pins, buffers. Call once at boot.
    static void init();

    // Drain and parse telemetry in foreground context. Called from idle()
    // and from manage_heater() (temperature.cpp) — the latter keeps the RX
    // ring fresh during safe_delay()/blocking waits, which never reach
    // idle(); see plan.md §5a.
    static void pollRx();

    // Called from manage_heater() in foreground context (idle(), safe_delay()
    // during blocking waits, planner.synchronize() during moves). Sends
    // command frames when a field changed or the 1 s heartbeat is due — the
    // only place serial writes happen, never from the Temperature ISR.
    static void tick();

    // Latest thermistor raw value in Marlin's accumulated-ADC units.
    // Returns adc_10bit * OVERSAMPLENR so it is indistinguishable from a
    // local thermistor to Marlin's thermistor math. When telemetry is absent
    // (link loss, or boot before the first valid frame) the returned value
    // is pegged full-scale so the reading shows the "no sensor" cold floor
    // (e.g. ≈ -15 degC and the ADC at 5 V for the EPCOS NTC100 table) — the
    // same fault signature an unplugged NTC produces — instead of raw 0
    // mapping to a hot reading.
    static uint16_t readRawADC();

    // Comms health: true while the no-telemetry watchdog has not tripped.
    // While false, readRawADC() pegs full scale (no-sensor signature) and
    // tick() is skipped so the CH32V003's own watchdog forces its heater off.
    static bool commsOk();

    // Endstop states from the telemetry cache.
    static bool zEndstopTriggered();
    static bool xEndstopTriggered();
    static bool bltouchTriggered();

    // Outbound command setters (mark the command dirty so it is sent ASAP).
    static void setHeaterPWM(uint8_t duty);
    static void setPartFanPWM(uint8_t duty);
    static void setBLTouchCommand(uint8_t cmd);

    // NOTE: No HSFan API — the CH32V003 owns the heatsink fan entirely.
    // NOTE: No LED API — PD4 hotend LED is owned by the CH32V003 (local rule).

    // Assemble and transmit a command frame. Foreground context only.
    static void sendCommand();

  private:
    static void parseTelemetry();
    static uint8_t crc8(const uint8_t *p, uint8_t len);

    // --- Telemetry cache (written by parseTelemetry, read by accessors) ---
    static volatile uint16_t raw_adc;        // Latest 10-bit ADC sample (0–1023)
    static volatile uint8_t  endstop_flags;  // Latest ENDSTOP_FLAGS byte
    static volatile uint8_t  status;         // Latest STATUS byte

    // --- Outbound command cache (foreground only: setters + TX) ---
    static uint8_t cmd_heater_pwm;
    static uint8_t cmd_pfan_pwm;
    static uint8_t cmd_bl_cmd;
    static bool cmd_dirty;                   // True when a command field changed

    // --- Timing / watchdog ---
    static volatile millis_t last_tlm_ms;    // millis() of last valid telemetry
    static millis_t last_heartbeat_ms;       // millis() of last heartbeat send
    static volatile bool link_lost;          // No-telemetry watchdog tripped

    // --- RX frame parser state ---
    static uint8_t  rx_buf[SH_TLM_LEN];
    static uint8_t  rx_idx;
    static bool     rx_in_frame;
};

#endif // SMARTHOTEND_ENABLED