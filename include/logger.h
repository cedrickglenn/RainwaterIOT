#ifndef LOGGER_H
#define LOGGER_H

#include <Arduino.h>

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  EVENT LOGGER — Serial2 log frame emitter
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Every significant system event is emitted as a one-line frame on Serial2
 *  so an external receiver (ESP32, Pi, PC) can parse, store, or forward them.
 *
 *  FRAME FORMAT:
 *    L,<LEVEL>,<CATEGORY>,<message>\n
 *
 *  LEVELS:     INFO | WARNING | ERROR
 *  CATEGORIES: SYSTEM | ACTUATOR | SENSOR | FILTER | PUMP | CALIBRATION
 *
 *  EXAMPLES:
 *    L,INFO,SYSTEM,Boot complete
 *    L,INFO,ACTUATOR,V3 opened
 *    L,ERROR,ACTUATOR,V3 failed to open
 *    L,INFO,PUMP,P1 started
 *    L,ERROR,PUMP,P1 dry-run protection triggered
 *    L,INFO,FILTER,BW_FILLING
 *    L,INFO,FILTER,Backwash started
 *    L,INFO,FILTER,Backwash complete
 *    L,WARNING,SENSOR,pH out of range: 9.12
 *    L,WARNING,SENSOR,Turbidity out of range: 7.40
 *    L,INFO,CALIBRATION,CAL_PH,C2,MID accepted
 *    L,INFO,CALIBRATION,pH saved to EEPROM
 *
 *  OUTPUT DESTINATIONS (both written on every logEvent call):
 *    Serial  (USB)   — debug monitor
 *    Serial1 (ESP32) — forwarded to MQTT; prefix [mega] added automatically
 *    Serial2 (pin16) — local log receiver (optional external device)
 *    Baud rates → DEBUG_BAUD_RATE / ESP32_BAUD_RATE / LOG_BAUD_RATE (config.h)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

// ── Level string constants (avoids typos at call sites) ──────────────────
#define LOG_INFO    "INFO"
#define LOG_WARN    "WARN"
#define LOG_WARNING "WARN"   // alias — both resolve to the same wire token
#define LOG_ERROR   "ERROR"

// ── Category string constants ─────────────────────────────────────────────
#define LOG_CAT_SYSTEM      "SYSTEM"
#define LOG_CAT_ACTUATOR    "ACTUATOR"
#define LOG_CAT_SENSOR      "SENSOR"
#define LOG_CAT_FILTER      "FILTER"
#define LOG_CAT_PUMP        "PUMP"
#define LOG_CAT_CALIBRATION "CALIBRATION"

/**
 * Initialise Serial2 for event logging.
 * Call once in setup() before any logEvent() calls.
 */
void logger_init();

/**
 * Emit one log frame on Serial2.
 *
 * Format: L,<level>,<category>,<message>\n
 *
 * @param level     LOG_INFO | LOG_WARNING | LOG_ERROR
 * @param category  LOG_CAT_* constant
 * @param message   Free-form message string
 */
void logEvent(const char* level, const char* category, const String& message);

#endif // LOGGER_H
