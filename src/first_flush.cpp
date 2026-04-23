#include "first_flush.h"
#include "pins.h"
#include "config.h"
#include "actuators.h"
#include "logger.h"

/*
 *  IMPLEMENTATION NOTES
 *
 *  ● All timing uses millis() — fully non-blocking.
 *  ● `flowConsistentMs` accumulates how long flow has been seen, but
 *    brief pauses (< FLOW_TIMEOUT_MS) only freeze the counter, they
 *    don't reset it.  This accommodates gusty rain that causes the
 *    gutter flow to pulse.
 *  ● During COLLECTING, the collection window timer protects against
 *    intermittent rain causing rapid state changes.  Even if flow stops
 *    inside the window, we stay in COLLECTING and keep Valve 1 open.
 *
 *  IDLE RAIN-CHECK PULSE:
 *  The flow sensor sits inline between C1 and a Tee junction.  V8 (drain)
 *  exits the Tee left/down; V1 (to C2) exits bottom/right.  With both
 *  valves closed, the Tee is a dead-end — incoming water pressurises the
 *  stub and stops moving, so the sensor reads zero regardless of rainfall.
 *  Solution: in IDLE, pulse V8 open for FLOW_CONFIRM_MS every
 *  IDLE_RAIN_CHECK_INTERVAL_MS.  This creates a drain path so water moves
 *  through the sensor.  If flow is detected during the pulse, we stay open
 *  and enter CONFIRMING normally.  If not, we close V8 and wait for the
 *  next interval.  Duty cycle ≈ 3 s / 60 s = 5%.
 *
 *  SUGGESTION: In addition to time-based first flush, track cumulative
 *  diverted volume (integrate flowRate × dt).  If a minimum litres
 *  target (e.g., 50 L) is reached before the time threshold, you could
 *  transition to COLLECTING early.  Volume-based flushing adapts better
 *  to varying rainfall intensities.
 */

// ── Module state ────────────────────────────────────────────────────────
static FirstFlushState state = FF_IDLE;

// Flush duration — how long consistent flow must be diverted before COLLECTING.
// Settable at runtime via firstFlush_setDuration(); defaults to compile-time constant.
static unsigned long ffDurationMs = FIRST_FLUSH_DURATION_MS;

// When true, firstFlush_update() is skipped and firstFlush_reset() suppresses
// the V8 idle pulse. Lets the operator hold valves manually during calibration.
static bool calModeActive = false;

static unsigned long confirmStartMs   = 0;  // when sustained flow first seen (CONFIRMING)
static unsigned long divertStartMs    = 0;  // when diversion began
static unsigned long flowConsistentMs = 0;  // accumulated consistent-flow time
static unsigned long lastFlowSeenMs   = 0;  // last millis() with active flow
static unsigned long collectStartMs   = 0;  // when collection window opened

// ── IDLE rain-check pulse state ─────────────────────────────────────────
static unsigned long idleCheckStartMs = 0;   // start of current pulse or sleep
static bool          idlePulsing      = false; // true = V8 open for rain-check

// ─────────────────────────────────────────────────────────────────────────
// Internal helpers
// ─────────────────────────────────────────────────────────────────────────

static void enterIdlePulse(unsigned long now)
{
    valve_open(VALVE8_PIN);
    idlePulsing      = true;
    idleCheckStartMs = now;
}

static void enterIdleSleep(unsigned long now)
{
    valve_close(VALVE8_PIN);
    idlePulsing      = false;
    idleCheckStartMs = now;
}

// ═════════════════════════════════════════════════════════════════════════
void firstFlush_init()
{
    unsigned long now = millis();
    state = FF_IDLE;
    valve_close(VALVE1_PIN);
    // Open V8 immediately on startup to purge residual water from the
    // tee-to-V1 pipe stub and begin listening for rain.
    enterIdlePulse(now);
}

// ═════════════════════════════════════════════════════════════════════════
void firstFlush_update(bool flowActive)
{
    if (calModeActive) return;

    unsigned long now = millis();

    switch (state) {

        // ── IDLE — waiting for rain, periodically pulsing V8 ──────────
        case FF_IDLE:
            if (idlePulsing) {
                if (flowActive) {
                    // Rain detected during pulse window — stay open, enter CONFIRMING
                    state          = FF_CONFIRMING;
                    confirmStartMs = now;
                    lastFlowSeenMs = now;
                    idlePulsing    = false;
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_CONFIRMING"));
                    Serial.println(F("[FirstFlush] Flow during pulse -> CONFIRMING"));
                } else if ((now - idleCheckStartMs) >= FLOW_CONFIRM_MS) {
                    // Pulse window expired, no flow — close V8 and sleep
                    enterIdleSleep(now);
                    Serial.println(F("[FirstFlush] Pulse expired, no flow -> sleeping"));
                }
                // else: still within pulse window, wait
            } else {
                // Sleeping — check if it's time for the next pulse
                if ((now - idleCheckStartMs) >= IDLE_RAIN_CHECK_INTERVAL_MS) {
                    enterIdlePulse(now);
                    Serial.println(F("[FirstFlush] IDLE pulse -> V8 open"));
                }
            }
            break;

        // ── CONFIRMING — sustained flow seen, V8 open, waiting for confidence
        case FF_CONFIRMING:
            if (flowActive) {
                lastFlowSeenMs = now;
                if ((now - confirmStartMs) >= FLOW_CONFIRM_MS) {
                    // Sustained flow confirmed — V8 already open, start diverting
                    state            = FF_DIVERTING;
                    divertStartMs    = now;
                    flowConsistentMs = now - confirmStartMs;
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_DIVERTING"));
                    Serial.println(F("[FirstFlush] Confirmed -> DIVERTING (V8 open)"));
                }
            } else {
                // Flow dropped before confirmation — false alarm, back to IDLE sleep
                state = FF_IDLE;
                enterIdleSleep(now);
                logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_IDLE"));
                Serial.println(F("[FirstFlush] Confirm failed -> IDLE (sleeping)"));
            }
            break;

        // ── DIVERTING — first flush in progress ─────────────────────
        case FF_DIVERTING:
            if (flowActive) {
                flowConsistentMs += (now - lastFlowSeenMs);
                lastFlowSeenMs = now;

                if (flowConsistentMs >= ffDurationMs) {
                    // First flush complete — transition to collection
                    state = FF_COLLECTING;
                    valve_close(VALVE8_PIN);
                    valve_open(VALVE1_PIN);
                    collectStartMs = now;
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_COLLECTING"));
                    Serial.println(F("[FirstFlush] Flush complete -> COLLECTING"));
                }
            } else {
                if ((now - lastFlowSeenMs) > FLOW_TIMEOUT_MS) {
                    // Rain stopped — reset to IDLE sleep (no immediate pulse)
                    state = FF_IDLE;
                    valve_close(VALVE8_PIN);
                    valve_close(VALVE1_PIN);
                    idlePulsing      = false;
                    idleCheckStartMs = now;
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_IDLE"));
                    Serial.println(F("[FirstFlush] Flow timeout -> IDLE (sleeping)"));
                }
            }
            break;

        // ── COLLECTING — water flowing into Container 2 ─────────────
        case FF_COLLECTING:
        {
            if (flowActive) {
                lastFlowSeenMs = now;
            }

            bool windowExpired = (now - collectStartMs) >= COLLECTION_WINDOW_MS;

            if (windowExpired && !flowActive) {
                if ((now - lastFlowSeenMs) > FLOW_TIMEOUT_MS) {
                    state = FF_IDLE;
                    valve_close(VALVE1_PIN);
                    idlePulsing      = false;
                    idleCheckStartMs = now;
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_IDLE"));
                    Serial.println(F("[FirstFlush] Collection done -> IDLE (sleeping)"));
                    // SUGGESTION: Log total collected volume here
                }
            }
            break;
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
FirstFlushState firstFlush_getState()
{
    return state;
}

// ═════════════════════════════════════════════════════════════════════════
void firstFlush_reset()
{
    // In calibration mode the operator holds full valve control —
    // skip the reset entirely so V8 doesn't pulse open unexpectedly.
    if (calModeActive) {
        Serial.println(F("[FirstFlush] RESET suppressed (cal mode active)"));
        return;
    }

    unsigned long now = millis();
    state = FF_IDLE;
    valve_close(VALVE1_PIN);
    // Open V8 as a purge pulse — same behaviour as init()
    enterIdlePulse(now);
    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_IDLE"));
    Serial.println(F("[FirstFlush] RESET -> IDLE (V8 pulse)"));
}

// ═════════════════════════════════════════════════════════════════════════
void firstFlush_setCalMode(bool enable)
{
    calModeActive = enable;
    Serial.print(F("[FirstFlush] Cal mode: "));
    Serial.println(enable ? F("ON (state machine suspended)") : F("OFF (resumed)"));
}

// ═════════════════════════════════════════════════════════════════════════
bool firstFlush_isCalMode()
{
    return calModeActive;
}

// ═════════════════════════════════════════════════════════════════════════
void firstFlush_setDuration(unsigned long ms)
{
    ffDurationMs = ms;
    Serial.print(F("[FirstFlush] Flush duration set to "));
    Serial.print(ms / 1000UL);
    Serial.println(F(" s"));
}
