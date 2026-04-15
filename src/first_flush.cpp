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
 *  SUGGESTION: In addition to time-based first flush, track cumulative
 *  diverted volume (integrate flowRate × dt).  If a minimum litres
 *  target (e.g., 50 L) is reached before the time threshold, you could
 *  transition to COLLECTING early.  Volume-based flushing adapts better
 *  to varying rainfall intensities.
 */

// ── Module state ────────────────────────────────────────────────────────
static FirstFlushState state = FF_IDLE;

static unsigned long confirmStartMs   = 0;  // when sustained flow first seen (CONFIRMING)
static unsigned long divertStartMs    = 0;  // when diversion began
static unsigned long flowConsistentMs = 0;  // accumulated consistent-flow time
static unsigned long lastFlowSeenMs   = 0;  // last millis() with active flow
static unsigned long collectStartMs   = 0;  // when collection window opened

// ═════════════════════════════════════════════════════════════════════════
void firstFlush_init()
{
    state = FF_IDLE;
    valve_close(VALVE1_PIN);
    valve_close(VALVE8_PIN);
}

// ═════════════════════════════════════════════════════════════════════════
void firstFlush_update(bool flowActive)
{
    unsigned long now = millis();

    switch (state) {

        // ── IDLE — waiting for rain, V8 sleeping ───────────────────
        case FF_IDLE:
            if (flowActive) {
                state          = FF_CONFIRMING;
                confirmStartMs = now;
                lastFlowSeenMs = now;
                logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_CONFIRMING"));
                Serial.println(F("[FirstFlush] Flow detected -> CONFIRMING"));
            }
            break;

        // ── CONFIRMING — flow seen, waiting before opening V8 ──────
        //    V8 stays closed here.  This filters splashes / transients
        //    and gives water time to travel from the sensor to the valve.
        case FF_CONFIRMING:
            if (flowActive) {
                lastFlowSeenMs = now;
                if ((now - confirmStartMs) >= FLOW_CONFIRM_MS) {
                    // Sustained flow confirmed — open V8 and start diverting
                    state = FF_DIVERTING;
                    valve_open(VALVE8_PIN);
                    valve_close(VALVE1_PIN);
                    divertStartMs    = now;
                    flowConsistentMs = now - confirmStartMs;  // count confirm time
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_DIVERTING"));
                    Serial.println(F("[FirstFlush] Confirmed -> DIVERTING (V8 open)"));
                }
            } else {
                // Flow dropped before confirmation — false alarm, back to IDLE
                state = FF_IDLE;
                logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_IDLE"));
                Serial.println(F("[FirstFlush] Confirm failed -> IDLE"));
            }
            break;

        // ── DIVERTING — first flush in progress ─────────────────────
        case FF_DIVERTING:
            if (flowActive) {
                // Accumulate time since last check
                flowConsistentMs += (now - lastFlowSeenMs);
                lastFlowSeenMs = now;

                // Has enough consistent flow time elapsed?
                if (flowConsistentMs >= FIRST_FLUSH_DURATION_MS) {
                    // First flush complete — transition to collection
                    state = FF_COLLECTING;
                    valve_close(VALVE8_PIN);   // stop draining
                    valve_open(VALVE1_PIN);    // allow into Container 2
                    collectStartMs = now;
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_COLLECTING"));
                    Serial.println(F("[FirstFlush] Flush complete -> COLLECTING"));
                }
            } else {
                // Flow paused — check whether it's been too long
                if ((now - lastFlowSeenMs) > FLOW_TIMEOUT_MS) {
                    // Rain has likely stopped.  Reset.
                    state = FF_IDLE;
                    valve_close(VALVE8_PIN);
                    valve_close(VALVE1_PIN);
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_IDLE"));
                    Serial.println(F("[FirstFlush] Flow timeout -> IDLE"));
                }
                // Within timeout: tolerate the gap (brief gust / lull)
            }
            break;

        // ── COLLECTING — water flowing into Container 2 ─────────────
        case FF_COLLECTING:
        {
            if (flowActive) {
                lastFlowSeenMs = now;   // keep tracking
            }

            bool windowExpired = (now - collectStartMs) >= COLLECTION_WINDOW_MS;

            // Only go back to IDLE if the protection window has expired
            // AND flow has actually stopped for longer than the timeout.
            if (windowExpired && !flowActive) {
                if ((now - lastFlowSeenMs) > FLOW_TIMEOUT_MS) {
                    state = FF_IDLE;
                    valve_close(VALVE1_PIN);
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_IDLE"));
                    Serial.println(F("[FirstFlush] Collection done -> IDLE"));
                    // SUGGESTION: Log total collected volume here
                    //   collectedLitres += flowRate * elapsedSeconds;
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
    state = FF_IDLE;
    valve_close(VALVE1_PIN);
    valve_close(VALVE8_PIN);
    logEvent(LOG_INFO, LOG_CAT_FILTER, F("FF_IDLE"));
    Serial.println(F("[FirstFlush] RESET -> IDLE"));
}
