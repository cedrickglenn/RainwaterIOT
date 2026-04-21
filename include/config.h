#ifndef CONFIG_H
#define CONFIG_H

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  SYSTEM CONFIGURATION
 *  Thresholds, Timing Constants, and Water Quality Parameters
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Every tunable "magic number" lives here. Adjust these values based on
 *  your physical setup (container dimensions, sensor mounting heights,
 *  local rainfall characteristics, etc.).
 *
 *  SUGGESTIONS:
 *  ● Store critical thresholds in EEPROM so they can be updated remotely
 *    via the ESP32/app without reflashing the Mega.
 *  ● Consider a "learning mode" that auto-calibrates water level
 *    thresholds by measuring empty and full states during initial setup.
 *  ● Add hysteresis to level thresholds to prevent relay chatter.
 *    The current approach uses a HIGH and LOW threshold per container;
 *    the gap between them IS the hysteresis band.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

// ── Timing (milliseconds) ───────────────────────────────────────────────

// First flush: how long consistent flow must be detected before we divert
// water into Container 2.  5 minutes flushes roughly 40–100 L off a
// medium-sized roof — adjust based on your catchment area and climate.
#define FIRST_FLUSH_DURATION_MS    300000UL   // 5 minutes

// After first flush passes, keep valve 1 open for at least this long even
// if rain becomes intermittent.  Prevents constant open/close cycling
// during typical tropical convective rain patterns.
#define COLLECTION_WINDOW_MS       3600000UL  // 1 hour

// If flow drops to zero for longer than this during the DIVERTING state,
// we assume rain has stopped (not just a brief lull) and reset to IDLE.
#define FLOW_TIMEOUT_MS            30000UL    // 30 seconds

// How often the main loop reads ALL sensors (non-blocking millis check).
#define SENSOR_READ_INTERVAL_MS    1000UL     // 1 second

// How often the Mega pushes a full sensor data frame to the ESP32.
#define COMMS_TX_INTERVAL_MS       2000UL     // 2 seconds

// ── Water Level Thresholds (cm from sensor to water surface) ────────────
//
// Ultrasonic sensors are mounted at the TOP of each container, pointing
// DOWN at the water surface.  A SMALLER distance reading = HIGHER water.
//
//    Sensor ────┐
//               │  ← distance reading (cm)
//    ┌──────────┴──────────┐
//    │ ~~~~~~water~~~~~~   │  ← water surface
//    │                     │
//    └─────────────────────┘
//
// For each container we define two thresholds:
//   *_HIGH_CM  — water is "high enough" to start the next stage
//   *_LOW_CM   — water is "too low" to keep the stage running
// The gap between them is the hysteresis band (prevents relay chatter).

// Container 2 (buffer storage — receives rainwater from catchment)
#define C2_LEVEL_OVERFLOW      95    // % — emergency stop: tank about to overflow
#define C2_LEVEL_HIGH_CM       20    // % — start pumping to charcoal filter
#define C2_LEVEL_LOW_CM        80    // % — stop pumping (dry-run risk)

// Container 3 (charcoal filter vessel — thresholds used during backwash)
#define C3_BACKWASH_FULL_CM    15    // Near overflow → open drain valves
#define C3_BACKWASH_EMPTY_CM   90    // Fully drained → close drain, refill

// Container 4 (pre-commercial RO buffer)
#define C4_LEVEL_OVERFLOW      95    // % — emergency stop: tank about to overflow
#define C4_LEVEL_HIGH_CM       20    // % — enough water → run pump 2
#define C4_LEVEL_LOW_CM        80    // % — too low → stop pump 2

// Container 5 (quality-check tank — water tested here before final storage)
#define C5_LEVEL_OVERFLOW      95    // % — emergency stop: tank about to overflow
#define C5_LEVEL_HIGH_CM       25    // % — enough water to test and pump out
#define C5_LEVEL_LOW_CM        80    // % — too low → stop

// Container 6 (final potable storage) — CRITICAL safety thresholds
#define C6_LEVEL_FULL_CM       15    // FULL → EMERGENCY STOP ALL processes!
#define C6_LEVEL_RESUME_CM     30    // Water consumed enough → allow resume
// SUGGESTION: Install a physical float switch as a redundant hardware
//             backup for Container 6.  Software-only overflow protection
//             is a single point of failure.

// ── Backwash Configuration ──────────────────────────────────────────────
#define BACKWASH_CYCLES        3     // Number of fill-drain cycles per session
// SUGGESTION: Track cumulative litres processed since last backwash
//             (via the flow sensor) and trigger automatically — e.g.,
//             every 5 000 L.  This is more reliable than a calendar timer.

// ── Water Quality — PNSDW 2017 (Philippine National Standards for ───────
//    Drinking Water, DOH Administrative Order No. 2017-0010)
//
// These are the MANDATORY limits that water must satisfy before it is
// routed to the final storage container.  If ANY parameter fails, the
// water is recycled back to Container 4 for re-processing.

#define WQ_PH_MIN              6.5f  // Minimum acceptable pH
#define WQ_PH_MAX              8.5f  // Maximum acceptable pH
#define WQ_TURBIDITY_MAX_NTU   5.0f  // Maximum turbidity (NTU)
// NOTE: WHO recommends < 1 NTU for effective disinfection.  If you add
//       a UV or chlorine stage downstream, tighten this to 1.0 NTU.

// Temperature isn't directly regulated by PNSDW, but extreme readings
// indicate a sensor fault or unusual condition worth flagging.
#define WQ_TEMP_MIN_C          10.0f // Flag if below (sensor issue?)
#define WQ_TEMP_MAX_C          40.0f // Flag if above (possible heating)

// SUGGESTION: PNSDW 2017 also mandates bacteriological testing
//             (E. coli, total coliform).  Automated coliform sensors exist
//             but are expensive.  Consider periodic manual lab tests and
//             log compliance dates in the database.

// ── Flow Sensor ─────────────────────────────────────────────────────────
// Minimum flow rate to consider as "active flow".
// Filters out sensor noise when no water is actually moving.
#define FLOW_MIN_THRESHOLD     0.5f  // L/min

// How long flow must be sustained before Valve 8 (first-flush diverter)
// opens.  The flow sensor is upstream of V8 — this delay lets water
// travel the pipe length so we don't actuate on a splash or sensor
// transient.  3 seconds at typical gutter flow is sufficient.
#define FLOW_CONFIRM_MS        3000UL  // 3 seconds

// ── Serial Baud Rates ───────────────────────────────────────────────────
#define DEBUG_BAUD_RATE        115200 // USB Serial (Serial Monitor / debug)
#define ESP32_BAUD_RATE        115200  // Serial1 → ESP32 link
#define LOG_BAUD_RATE          115200  // Serial2 → event log receiver
// SUGGESTION: 115200 baud is fine for the ESP32 link if wires are short
//             (< 30 cm).  For longer runs, stick with 9600 or add a
//             MAX3232 / RS-485 transceiver.

#endif // CONFIG_H
