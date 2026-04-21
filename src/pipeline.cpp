#include "pipeline.h"
#include "pins.h"
#include "config.h"
#include "actuators.h"
#include "logger.h"

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  PIPELINE IMPLEMENTATION
 *
 *  Each "stage" is a static function called from pipeline_update().
 *  Stages run every loop() tick and make their own decisions based on
 *  sensor data + thresholds from config.h.
 *
 *  Stages can operate concurrently:
 *    Container 2 can be filling from rain while Container 4 is draining
 *    through the RO filter.  The only hard constraint is Container 6 —
 *    if it's full, EVERYTHING stops.
 *
 *  Backwash mode takes exclusive control of the Container 2 → Charcoal
 *  Filter path (valve 2 + pump 1).  While backwash is active, the normal
 *  stage_container2() function is skipped.
 * ═══════════════════════════════════════════════════════════════════════════
 */

// ── Module state ────────────────────────────────────────────────────────
static FilterMode    currentFilterMode  = FILTER_CHARCOAL_AND_RO;
static BackwashState backwashState      = BW_IDLE;
static uint8_t       backwashCyclesDone = 0;
static bool          emergencyStopped   = false;

// ═════════════════════════════════════════════════════════════════════════
//  STAGE: Container 2 → Charcoal Filter
// ═════════════════════════════════════════════════════════════════════════
//
//  When Container 2 has enough water, pump it through valve 2 + pump 1
//  to the charcoal filter.  The output routing (valve 3 vs. valve 4)
//  depends on the current FilterMode.
//
//  SKIPPED while a backwash is running (backwash shares the same path).
//
//  HYSTERESIS:
//    START pumping when level ≤ C2_LEVEL_HIGH_CM (water is high)
//    STOP  pumping when level ≥ C2_LEVEL_LOW_CM  (water is low)
//    In between: no change → pump keeps whatever state it was in.
//
static void stage_container2(const SensorData* data)
{
    // Backwash takes exclusive control of this path
    if (backwashState != BW_IDLE) return;

    // Overflow guard — stop all inflow if C2 is nearly full
    if (data->levelC2 >= (float)C2_LEVEL_OVERFLOW) {
        valve_close(VALVE1_PIN);
        logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("C2 overflow protection: inlet closed"));
        return;
    }

    bool levelHigh = (data->levelC2 <= (float)C2_LEVEL_HIGH_CM);
    bool levelLow  = (data->levelC2 >= (float)C2_LEVEL_LOW_CM);

    if (levelHigh && !levelLow) {
        // Water is high enough — pump to charcoal filter
        valve_open(VALVE2_PIN);
        pump_start(PUMP1_PIN);

        // Route based on current filter mode
        if (currentFilterMode == FILTER_CHARCOAL_ONLY) {
            valve_open(VALVE3_PIN);    // direct → Container 6
            valve_close(VALVE4_PIN);
        } else {
            valve_close(VALVE3_PIN);
            valve_open(VALVE4_PIN);    // → Container 4 → RO path
        }
    }

    if (levelLow) {
        // Water too low — shut down to prevent dry-running pump 1
        bool wasRunning = actuator_isOn(PUMP1_PIN);
        valve_close(VALVE2_PIN);
        pump_stop(PUMP1_PIN);
        valve_close(VALVE3_PIN);
        valve_close(VALVE4_PIN);

        if (wasRunning) {
            logEvent(LOG_ERROR, LOG_CAT_PUMP, F("P1 dry-run protection triggered"));
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  STAGE: Charcoal Filter Backwash
// ═════════════════════════════════════════════════════════════════════════
//
//  Backwash flushes accumulated sediment by repeatedly:
//    1. FILLING  — pump water from Container 2 into the filter vessel.
//    2. DRAINING — once the filter is nearly full, open valve 5 (filter
//                  drain) + valve 8 (waste drainage) and let gravity empty.
//
//  Repeat for BACKWASH_CYCLES times, then return to BW_IDLE.
//
//  SUGGESTION: After the final drain cycle, run a brief "rinse" cycle
//  where water is pumped through the normal output path (valve 4) to
//  flush any residual sediment before resuming normal filtration.
//
static void stage_backwash(const SensorData* data)
{
    if (backwashState == BW_IDLE || backwashState == BW_COMPLETE) return;

    switch (backwashState) {

        case BW_FILLING:
            // Pump water from Container 2 into the charcoal filter vessel
            valve_open(VALVE2_PIN);
            pump_start(PUMP1_PIN);
            valve_close(VALVE3_PIN);   // don't route to any output
            valve_close(VALVE4_PIN);

            // Has the filter vessel filled to the target level?
            if (data->levelC3 <= (float)C3_BACKWASH_FULL_CM) {
                // Full enough — switch to draining
                pump_stop(PUMP1_PIN);
                valve_close(VALVE2_PIN);
                valve_open(VALVE5_PIN);    // charcoal filter drain
                valve_open(VALVE8_PIN);    // waste drainage
                backwashState = BW_DRAINING;
                logEvent(LOG_INFO, LOG_CAT_FILTER, F("BW_DRAINING"));

                Serial.print(F("[Backwash] Cycle "));
                Serial.print(backwashCyclesDone + 1);
                Serial.println(F(" -> DRAINING"));
            }
            break;

        case BW_DRAINING:
            // Wait for the filter vessel to drain completely
            if (data->levelC3 >= (float)C3_BACKWASH_EMPTY_CM) {
                // Vessel is empty — close drain valves
                valve_close(VALVE5_PIN);
                valve_close(VALVE8_PIN);
                backwashCyclesDone++;

                if (backwashCyclesDone >= BACKWASH_CYCLES) {
                    // All cycles done
                    backwashState      = BW_IDLE;
                    backwashCyclesDone = 0;
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("Backwash complete"));
                    Serial.println(F("[Backwash] COMPLETE -> normal operation"));
                } else {
                    // More cycles needed — refill
                    backwashState = BW_FILLING;
                    logEvent(LOG_INFO, LOG_CAT_FILTER, F("BW_FILLING"));
                    Serial.print(F("[Backwash] Cycle "));
                    Serial.print(backwashCyclesDone + 1);
                    Serial.println(F(" -> FILLING"));
                }
            }
            break;

        default:
            break;
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  STAGE: Container 4 → Commercial RO Filter
// ═════════════════════════════════════════════════════════════════════════
//
//  Container 4 buffers charcoal-filtered water before the RO membrane.
//  When the level is high enough, run pump 2 to push water through the
//  commercial RO filter.  The RO output flows into Container 5.
//
//  NOTE: In CHARCOAL_ONLY mode this stage is disabled (pump 2 stays off)
//  because water never reaches Container 4.
//
//  SUGGESTION: Commercial RO filters produce a waste/brine stream
//  (typically 3:1 waste-to-product ratio).  Route this to a grey-water
//  tank for garden irrigation rather than sending it to drainage.
//  Also monitor RO inlet/outlet pressure if possible — a rising
//  differential indicates membrane fouling and need for replacement.
//
static void stage_container4(const SensorData* data)
{
    // Only relevant if the RO path is active
    if (currentFilterMode == FILTER_CHARCOAL_ONLY) {
        pump_stop(PUMP2_PIN);
        return;
    }

    // Overflow guard — C4 nearly full, stop ALL inflow paths
    if (data->levelC4 >= (float)C4_LEVEL_OVERFLOW) {
        pump_stop(PUMP1_PIN);      // C2 → charcoal filter pump (feeds V4)
        valve_close(VALVE2_PIN);   // C2 → charcoal filter inlet
        valve_close(VALVE4_PIN);   // charcoal filter → C4
        valve_close(VALVE6_PIN);   // C5 recycle → C4
        pump_stop(PUMP4_PIN);      // C5 recycle booster
        logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("C4 overflow protection: inflow stopped"));
        return;
    }

    bool levelHigh = (data->levelC4 <= (float)C4_LEVEL_HIGH_CM);
    bool levelLow  = (data->levelC4 >= (float)C4_LEVEL_LOW_CM);

    if (levelHigh && !levelLow) {
        pump_start(PUMP2_PIN);
    }

    if (levelLow) {
        pump_stop(PUMP2_PIN);
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  STAGE: Container 5 — Water Quality Check & Routing
// ═════════════════════════════════════════════════════════════════════════
//
//  This is the "decision fork":
//    ● Water PASSES PNSDW 2017 → pump 3 + valve 7  → Container 6 (good!)
//    ● Water FAILS              → pump 3 + valve 6 + pump 4 → Container 4
//                                 (feedback loop for re-processing)
//
//  SUGGESTIONS:
//  ● Don't test immediately after pumping stops.  Add a 30–60 s "settle"
//    timer so turbidity readings stabilise.  Agitation from the pump
//    causes transient spikes that can trigger false failures.
//  ● Implement a "consecutive pass count" — only route to Container 6
//    after N consecutive passes (e.g., 3).  This increases confidence
//    and guards against a single lucky reading slipping through.
//  ● Track a "recycle counter".  If water fails quality > N times in a
//    row, stop the feedback loop and alert the user.  The filter may
//    need cleaning/replacement.
//  ● Log every quality test result (pass/fail with values) to the
//    database.  Gradual pH drift or rising turbidity is an early warning
//    of filter degradation — catch it before it reaches the fail limit.
//
static void stage_container5(const SensorData* data)
{
    // Overflow guard — C5 nearly full, stop RO inflow
    if (data->levelC5 >= (float)C5_LEVEL_OVERFLOW) {
        pump_stop(PUMP2_PIN);   // RO filter → C5
        logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("C5 overflow protection: RO pump stopped"));
        return;
    }

    bool hasWater = (data->levelC5 <= (float)C5_LEVEL_HIGH_CM);
    bool levelLow = (data->levelC5 >= (float)C5_LEVEL_LOW_CM);

    if (!hasWater || levelLow) {
        // Not enough water to test or pump — shut everything off
        pump_stop(PUMP3_PIN);
        valve_close(VALVE6_PIN);
        valve_close(VALVE7_PIN);
        pump_stop(PUMP4_PIN);
        return;
    }

    // We have enough water — evaluate quality against PNSDW 2017
    if (pipeline_isWaterPotable(data)) {
        // ✓ PASS → route to Container 6 (final potable storage)
        pump_start(PUMP3_PIN);
        valve_open(VALVE7_PIN);
        valve_close(VALVE6_PIN);
        pump_stop(PUMP4_PIN);
    } else {
        // ✗ FAIL → recycle back to Container 4 for re-treatment
        pump_start(PUMP3_PIN);
        valve_close(VALVE7_PIN);
        valve_open(VALVE6_PIN);
        pump_start(PUMP4_PIN);

        Serial.println(F("[Quality] FAIL -> water recycled to Container 4"));

        // SUGGESTION: Increment a static failCount here.  If it exceeds
        //   a threshold (e.g., 10), call pipeline_emergencyStop() and
        //   send an alert via comms.  Infinite recycling wastes energy
        //   and indicates a deeper problem.
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  STAGE: Container 6 — Final Storage Monitoring & Emergency Stop
// ═════════════════════════════════════════════════════════════════════════
//
//  Container 6 is the finished-product tank.  Its sensors (temp, pH,
//  turbidity, ultrasonic) are for MONITORING ONLY — the water here has
//  already passed the quality gate in Container 5.
//
//  The critical function is overflow protection:
//    If the water level indicates FULL → EMERGENCY STOP ALL processes.
//    Once the level drops (residents consume water) operations may resume.
//
//  SUGGESTIONS:
//  ● Install a physical float switch as a hardware backup.  Connect it
//    to a digital input and check it independently of the ultrasonic
//    sensor.  Belt AND suspenders.
//  ● Add a "high water warning" threshold (e.g., 20 cm) that sends an
//    app notification BEFORE the emergency stop threshold (15 cm).  This
//    gives the user time to open a tap or take action.
//  ● Monitor Container 6 water quality over time.  pH drift or turbidity
//    increase in storage can indicate biofilm growth or contamination
//    post-treatment.  Alert if values exceed PNSDW limits.
//  ● Consider adding a "last filled" timestamp to detect stagnant water.
//    Water sitting too long (> 3–5 days) should be flagged for manual
//    testing or flushed.
//
static void stage_container6(const SensorData* data)
{
    if (data->levelC6 <= (float)C6_LEVEL_FULL_CM) {
        // ═══ CONTAINER 6 IS FULL — EMERGENCY STOP ═══
        if (!emergencyStopped) {
            Serial.println(F("!!! CONTAINER 6 FULL — EMERGENCY STOP !!!"));
            pipeline_emergencyStop();
        }
    }
    else if (data->levelC6 >= (float)C6_LEVEL_RESUME_CM) {
        // Water has been consumed / dropped enough to resume
        if (emergencyStopped) {
            emergencyStopped = false;
            Serial.println(F("[Container 6] Level safe — operations may resume"));
            // NOTE: We don't auto-start anything.  The pipeline stages
            //       will naturally re-activate on the next update() cycle
            //       if their respective water-level conditions are met.
        }
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  PUBLIC API
// ═════════════════════════════════════════════════════════════════════════

void pipeline_init()
{
    currentFilterMode  = FILTER_CHARCOAL_AND_RO;
    backwashState      = BW_IDLE;
    backwashCyclesDone = 0;
    emergencyStopped   = false;
    // All actuators already OFF from actuators_init()
}

// ─────────────────────────────────────────────────────────────────────────
void pipeline_update(const SensorData* data)
{
    // If emergency-stopped, skip all stages
    if (emergencyStopped) return;

    // Run every stage — each one manages itself
    stage_container2(data);      // Buffer → charcoal filter
    stage_backwash(data);        // Charcoal filter backwash (when active)
    stage_container4(data);      // Pre-RO buffer → commercial filter
    stage_container5(data);      // Quality check + routing
    stage_container6(data);      // Final storage monitoring (must be LAST)
}

// ─────────────────────────────────────────────────────────────────────────
void pipeline_setFilterMode(FilterMode mode)
{
    currentFilterMode = mode;
    const __FlashStringHelper* name = (mode == FILTER_CHARCOAL_ONLY)
                                      ? F("CHARCOAL_ONLY")
                                      : F("CHARCOAL_AND_RO");
    logEvent(LOG_INFO, LOG_CAT_FILTER, name);
    Serial.print(F("[Pipeline] Filter mode -> "));
    Serial.println(name);
}

FilterMode pipeline_getFilterMode()
{
    return currentFilterMode;
}

// ─────────────────────────────────────────────────────────────────────────
void pipeline_startBackwash()
{
    if (backwashState != BW_IDLE) {
        Serial.println(F("[Backwash] Already in progress — ignoring"));
        return;
    }
    backwashCyclesDone = 0;
    backwashState      = BW_FILLING;
    logEvent(LOG_INFO, LOG_CAT_FILTER, F("Backwash started"));
    logEvent(LOG_INFO, LOG_CAT_FILTER, F("BW_FILLING"));
    Serial.println(F("[Backwash] STARTED — Cycle 1 -> FILLING"));
}

void pipeline_stopBackwash()
{
    // Safely shut down any in-progress backwash
    pump_stop(PUMP1_PIN);
    valve_close(VALVE2_PIN);
    valve_close(VALVE5_PIN);
    valve_close(VALVE8_PIN);
    backwashState      = BW_IDLE;
    backwashCyclesDone = 0;
    logEvent(LOG_INFO, LOG_CAT_FILTER, F("BW_IDLE"));
    Serial.println(F("[Backwash] STOPPED"));
}

BackwashState pipeline_getBackwashState()
{
    return backwashState;
}

// ─────────────────────────────────────────────────────────────────────────
bool pipeline_isTankFull(const SensorData* data)
{
    return (data->levelC6 <= (float)C6_LEVEL_FULL_CM);
}

// ─────────────────────────────────────────────────────────────────────────
bool pipeline_isWaterPotable(const SensorData* data)
{
    /*
     *  PNSDW 2017 Water Quality Evaluation
     *
     *  ALL parameters must pass simultaneously for the water to be
     *  considered potable.  This is the legal bar for municipal-level
     *  drinking water quality in the Philippines.
     *
     *  Parameters checked:
     *    pH        : 6.5 – 8.5
     *    Turbidity : ≤ 5 NTU   (WHO recommends < 1 NTU for disinfection)
     *    Temperature: 10 – 40 °C (not mandated, but flags sensor anomalies)
     *
     *  SUGGESTION: Add TDS (Total Dissolved Solids) if you install a
     *  TDS sensor.  PNSDW limit is 600 mg/L.  Also consider a residual
     *  chlorine sensor if you add a dosing stage (target 0.2–0.5 mg/L).
     */

    bool phOK   = (data->phC5 >= WQ_PH_MIN) && (data->phC5 <= WQ_PH_MAX);
    bool turbOK = (data->turbidityC5 <= WQ_TURBIDITY_MAX_NTU);
    bool tempOK = (data->tempC5 >= WQ_TEMP_MIN_C) && (data->tempC5 <= WQ_TEMP_MAX_C);

    if (!phOK) {
        logEvent(LOG_WARNING, LOG_CAT_SENSOR,
                 String("pH out of range: ") + String(data->phC5, 2));
    }
    if (!turbOK) {
        logEvent(LOG_WARNING, LOG_CAT_SENSOR,
                 String("Turbidity out of range: ") + String(data->turbidityC5, 1));
    }

    // Uncomment during development to see every quality evaluation:
    // Serial.print(F("[Quality] pH="));    Serial.print(data->phC5, 2);
    // Serial.print(F(" turb="));           Serial.print(data->turbidityC5, 1);
    // Serial.print(F(" temp="));           Serial.print(data->tempC5, 1);
    // Serial.println(phOK && turbOK && tempOK ? F(" PASS") : F(" FAIL"));

    return (phOK && turbOK && tempOK);
}

// ─────────────────────────────────────────────────────────────────────────
void pipeline_emergencyStop()
{
    emergencyStopped = true;

    // Cancel backwash if it's running
    if (backwashState != BW_IDLE) {
        pipeline_stopBackwash();
    }

    // Kill every actuator via the centralised emergency stop
    actuators_emergencyStop();
}
