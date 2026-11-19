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

/**
 * feature/smarthotend/SmartHotend.cpp
 *
 * Implements the UART protocol to a CH32V003 hotend I/O expander.
 * See SmartHotend.h and plan.md for the full contract.
 */

#include "../../inc/MarlinConfig.h"

#if ENABLED(SMARTHOTEND_ENABLED)

#include "SmartHotend.h"

#include "../../core/millis_t.h"
#include "../../module/temperature.h"

// --------------------------------------------------------------------------
// Static member definitions
// --------------------------------------------------------------------------

volatile uint16_t SmartHotend::raw_adc       = SH_ADC_NO_SENSOR_10BIT;
volatile uint8_t  SmartHotend::endstop_flags = 0;
volatile uint8_t  SmartHotend::status        = 0;

uint8_t      SmartHotend::cmd_heater_pwm = 0;
uint8_t      SmartHotend::cmd_pfan_pwm   = 0;
uint8_t      SmartHotend::cmd_bl_cmd     = SH_BL_IDLE;
bool         SmartHotend::cmd_dirty      = true;   // Send an initial zero frame

volatile millis_t SmartHotend::last_tlm_ms      = 0;
millis_t          SmartHotend::last_heartbeat_ms = 0;
volatile bool     SmartHotend::link_lost         = true;  // Boot: telemetry absent

uint8_t SmartHotend::rx_buf[SH_TLM_LEN];
uint8_t SmartHotend::rx_idx    = 0;
bool    SmartHotend::rx_in_frame = false;

// --------------------------------------------------------------------------
// CRC-8 (poly 0x07, init 0x00) — matches the CH32V003 firmware contract.
// --------------------------------------------------------------------------

uint8_t SmartHotend::crc8(const uint8_t *p, uint8_t len) {
  uint8_t crc = 0;
  for (uint8_t i = 0; i < len; i++) {
    crc ^= p[i];
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x80) ? (crc << 1) ^ 0x07 : (crc << 1);
  }
  return crc;
}

// --------------------------------------------------------------------------
// init() — start the UART, send an initial zero-duty command frame.
// --------------------------------------------------------------------------

void SmartHotend::init() {
  SMARTHOTEND_SERIAL.begin(SMARTHOTEND_BAUD);
  // Send a zero-duty command so the CH32V003 leaves reset state cleanly.
  cmd_heater_pwm = 0;
  cmd_pfan_pwm   = 0;
  cmd_bl_cmd     = SH_BL_IDLE;
  cmd_dirty      = true;
  sendCommand();
}

// --------------------------------------------------------------------------
// pollRx() — drain telemetry in foreground context, called from idle().
// --------------------------------------------------------------------------

void SmartHotend::pollRx() {
  parseTelemetry();
}

// --------------------------------------------------------------------------
// tick() — foreground command TX, called from manage_heater(). Sends a
// frame when a field changed or the 1 s heartbeat is due. Serial writes
// stay out of the Temperature ISR — manage_heater() runs from idle() and
// from safe_delay() during blocking waits, so heartbeats never starve.
// --------------------------------------------------------------------------

void SmartHotend::tick() {
  const millis_t now = millis();

  // Send a command frame when a field changed, or as a 1 s heartbeat.
  if (cmd_dirty || (now - last_heartbeat_ms) >= (millis_t)SMARTHOTEND_HEARTBEAT_MS) {
    sendCommand();
    last_heartbeat_ms = now;
  }
}

// --------------------------------------------------------------------------
// parseTelemetry() — scan the RX ring buffer for a valid 6-byte frame.
//   Frame: [0x55] [ADC_HI] [ADC_LO] [ENDSTOP_FLAGS] [STATUS] [CRC8]
//   ADC is 10-bit: bits 9..2 in ADC_HI, bits 1..0 in ADC_LO upper nibble.
// --------------------------------------------------------------------------

void SmartHotend::parseTelemetry() {
  while (SMARTHOTEND_SERIAL.available()) {
    const uint8_t b = (uint8_t)SMARTHOTEND_SERIAL.read();

    if (!rx_in_frame) {
      if (b == SH_TLM_SYNC) {
        rx_in_frame = true;
        rx_idx = 0;
        rx_buf[rx_idx++] = b;
      }
      continue;
    }

    // In-frame: collect bytes until we have a full frame.
    rx_buf[rx_idx++] = b;

    if (rx_idx < SH_TLM_LEN) continue;

    // Full frame received — validate CRC.
    rx_in_frame = false;
    rx_idx = 0;

    const uint8_t crc = crc8(rx_buf, SH_TLM_LEN - 1);
    if (crc != rx_buf[SH_TLM_LEN - 1]) continue;   // CRC mismatch → drop

    // --- Decode telemetry ---
    const uint16_t adc = ((uint16_t)rx_buf[1] << 2) | ((rx_buf[2] >> 6) & 0x03);
    const millis_t received_ms = millis();
    CRITICAL_SECTION_START();
      raw_adc       = adc;
      endstop_flags = rx_buf[3];
      status        = rx_buf[4];
      last_tlm_ms   = received_ms;
      link_lost     = false;
    CRITICAL_SECTION_END();
  }
}

// --------------------------------------------------------------------------
// readRawADC() — returns the latest thermistor raw in Marlin's accumulated
// units: adc_10bit * OVERSAMPLENR. This makes the smart hotend
// indistinguishable from a local thermistor to Marlin's thermistor math.
// When the link is lost (no valid telemetry for SMARTHOTEND_WATCHDOG_MS) the
// value is pegged full-scale (5 V on the NTC divider → the coldest table
// entry, e.g. -15 degC for the EPCOS NTC100 table) so Marlin shows the same
// sensor-missing signature as an unplugged NTC and its native MINTEMP fault
// path trips, instead of raw 0 mapping to a hot reading.
// --------------------------------------------------------------------------

uint16_t SmartHotend::readRawADC() {
  uint16_t adc;
  if (link_lost)
    adc = SH_ADC_NO_SENSOR_10BIT;
  else {
    CRITICAL_SECTION_START();
      adc = raw_adc;
    CRITICAL_SECTION_END();
  }
  return adc * (uint16_t)OVERSAMPLENR;
}

// --------------------------------------------------------------------------
// Endstop accessors
// --------------------------------------------------------------------------

bool SmartHotend::zEndstopTriggered()  { return TEST(endstop_flags, SH_ES_Z_BIT); }
bool SmartHotend::xEndstopTriggered()  { return TEST(endstop_flags, SH_ES_X_BIT); }
bool SmartHotend::bltouchTriggered()   { return TEST(endstop_flags, SH_ES_BL_BIT); }

// --------------------------------------------------------------------------
// Command setters — mark the command dirty so tick() sends it ASAP.
// --------------------------------------------------------------------------

void SmartHotend::setHeaterPWM(uint8_t duty)      { if (duty != cmd_heater_pwm) { cmd_heater_pwm = duty; cmd_dirty = true; } }
void SmartHotend::setPartFanPWM(uint8_t duty)     { if (duty != cmd_pfan_pwm)   { cmd_pfan_pwm   = duty; cmd_dirty = true; } }
void SmartHotend::setBLTouchCommand(uint8_t cmd)  { if (cmd != cmd_bl_cmd)     { cmd_bl_cmd     = cmd; cmd_dirty = true; } }

// --------------------------------------------------------------------------
// commsOk() — true while the no-telemetry watchdog has not tripped: a valid
//   telemetry frame arrived within the watchdog window. Ticked once per
//   manage_heater() cycle; the tripped state also pegs readRawADC() to the
//   no-sensor sentinel until telemetry resumes (and stops tick() so the
//   CH32V003's own comms watchdog forces the heater and part fan off).
// --------------------------------------------------------------------------

bool SmartHotend::commsOk() {
  const millis_t now = millis();
  if (link_lost) {
    millis_t last;
    CRITICAL_SECTION_START();
      last = last_tlm_ms;
    CRITICAL_SECTION_END();
    if ((now - last) < (millis_t)SMARTHOTEND_WATCHDOG_MS) return true;
    return false;
  }

  if ((now - last_tlm_ms) >= (millis_t)SMARTHOTEND_WATCHDOG_MS)
    link_lost = true;              // Telemetry went missing → peg the ADC raw
  return !link_lost;
}

// --------------------------------------------------------------------------
// sendCommand() — assemble and transmit a 5-byte command frame.
//   Frame: [0xAA] [HEATER_PWM] [PFAN_PWM] [BL_CMD] [CRC8]
// --------------------------------------------------------------------------

void SmartHotend::sendCommand() {
  uint8_t frame[SH_CMD_LEN];
  frame[0] = SH_CMD_SYNC;
  frame[1] = cmd_heater_pwm;
  frame[2] = cmd_pfan_pwm;
  frame[3] = cmd_bl_cmd;
  frame[4] = crc8(frame, SH_CMD_LEN - 1);

  for (uint8_t i = 0; i < SH_CMD_LEN; i++)
    SMARTHOTEND_SERIAL.write(frame[i]);

  cmd_dirty = false;
}

#endif // SMARTHOTEND_ENABLED