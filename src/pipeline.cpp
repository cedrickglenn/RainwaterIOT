#include "pipeline.h"
#include "pins.h"
#include "config.h"
#include "actuators.h"
#include "logger.h"
#include "calibration.h"
#include "first_flush.h"
#include "comms.h"

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

// Overflow latches — log the warning once per overflow event, not every tick.
// Cleared only after level drops back to the *_LEVEL_RESUME threshold (hysteresis).
static bool overflowC2 = false;
static bool overflowC3 = false;
static bool overflowC4 = false;
static bool overflowC5 = false;

// ── C5 quality gate state ────────────────────────────────────────────────
// Require QUALITY_PASS_REQUIRED consecutive passing evaluations before
// routing to C6.  A single noisy pH or turbidity reading cannot flip the
// routing decision — the averaging filter reduces noise but this guard
// protects against residual spikes and transients from pump agitation.
static const uint8_t QUALITY_PASS_REQUIRED = 3;
static const uint8_t QUALITY_FAIL_REQUIRED = 3;   // consecutive fails before recycling to C4
static const uint8_t QUALITY_FAIL_ALERT    = 10;  // log alert after N consecutive failures
static uint8_t       qualityPassCount      = 0;
static uint8_t       qualityFailCount      = 0;

// Tracks the last routing decision applied to P3/P4/V6/V7 in stage_container5().
// -1 = not yet set, 0 = stopped (low level), 1 = calibration pass-through,
//  2 = accumulating passes (holding), 3 = quality PASS routed to C6,
//  4 = quality FAIL recycled to C4.
// Actuator writes are skipped when the decision is the same as last time,
// preventing relay chatter at 1 Hz from unconditional digitalWrite calls.
static int8_t lastC5Routing = -1;

// True while C5 has insufficient water.  Flips to false the first tick C5
// has enough water to begin quality evaluation.  Sensor out-of-range warnings
// are logged exactly once on that transition (once per fill cycle) so that
// recycled water being re-evaluated does not re-fire the same warning.
static bool c5WasEmpty = true;

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

    // Skip entirely if C2 level sensor is not calibrated — a zero/garbage
    // reading gives 0 % which fails levelHigh (0 >= 20) and leaves pumps off.
    if (!cal_isLevelCalibrated(0)) return;

    // Overflow guard — stop all inflow if C2 is nearly full
    if (data->levelC2 >= (float)C2_LEVEL_OVERFLOW) {
        valve_close(VALVE1_PIN);
        if (!overflowC2) {
            overflowC2 = true;
            logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("C2 overflow protection: inlet closed"));
            comms_sendActuatorStatus();
        }
        return;
    }
    if (overflowC2 && data->levelC2 <= (float)C2_LEVEL_RESUME) {
        overflowC2 = false;
    }
    // Still draining toward resume — keep inlet closed but let normal logic run
    // so other actuators (PUMP1 etc.) are managed correctly by the level checks below.
    if (overflowC2) {
        valve_close(VALVE1_PIN);
    }

    // Overflow guard — C3 nearly full, stop feeding it
    if (cal_isLevelCalibrated(1) && data->levelC3 >= (float)C3_LEVEL_OVERFLOW) {
        pump_stop(PUMP1_PIN);
        valve_close(VALVE2_PIN);
        valve_close(VALVE3_PIN);
        valve_close(VALVE4_PIN);
        if (!overflowC3) {
            overflowC3 = true;
            logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("C3 overflow protection: feed stopped"));
            comms_sendActuatorStatus();
        }
        return;
    }
    if (overflowC3 && cal_isLevelCalibrated(1) && data->levelC3 <= (float)C3_LEVEL_RESUME) {
        overflowC3 = false;
    }

    bool levelHigh = (data->levelC2 >= (float)C2_LEVEL_HIGH_CM);
    bool levelLow  = (data->levelC2 <= (float)C2_LEVEL_LOW_CM);

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
            comms_sendActuatorStatus();
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

    // Overflow guard — stop feed if C3 is at overflow
    if (cal_isLevelCalibrated(1) && data->levelC3 >= (float)C3_LEVEL_OVERFLOW) {
        pump_stop(PUMP1_PIN);
        valve_close(VALVE2_PIN);
        logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("C3 overflow protection: feed stopped"));
        return;
    }

    switch (backwashState) {

        case BW_FILLING:
            // Pump water from Container 2 into the charcoal filter vessel
            valve_open(VALVE2_PIN);
            pump_start(PUMP1_PIN);
            valve_close(VALVE3_PIN);   // don't route to any output
            valve_close(VALVE4_PIN);

            // Has the filter vessel filled to the target level?
            if (cal_isLevelCalibrated(1) && data->levelC3 >= (float)C3_LEVEL_OVERFLOW) {
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

    // Skip entirely if C4 level sensor is not calibrated — same startup-glitch
    // risk as C2: a zero reading gives 0 % which fails levelHigh (0 >= 20).
    if (!cal_isLevelCalibrated(2)) return;

    // Overflow guard — C4 nearly full, stop ALL inflow paths.
    // Do NOT return here: PUMP2 (C4 → RO → C5) is outflow and must still run
    // to drain C4 back below the overflow threshold.
    if (data->levelC4 >= (float)C4_LEVEL_OVERFLOW) {
        pump_stop(PUMP1_PIN);      // C2 → charcoal filter pump (feeds V4)
        valve_close(VALVE2_PIN);   // C2 → charcoal filter inlet
        valve_close(VALVE4_PIN);   // charcoal filter → C4
        valve_close(VALVE6_PIN);   // C5 recycle → C4
        pump_stop(PUMP4_PIN);      // C5 recycle booster
        if (!overflowC4) {
            overflowC4 = true;
            logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("C4 overflow protection: inflow stopped"));
            comms_sendActuatorStatus();
        }
    }
    if (overflowC4 && data->levelC4 <= (float)C4_LEVEL_RESUME) {
        overflowC4 = false;
    }

    bool levelHigh = (data->levelC4 >= (float)C4_LEVEL_HIGH_CM);
    bool levelLow  = (data->levelC4 <= (float)C4_LEVEL_LOW_CM);

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
    // Skip entirely if C5 level sensor is not calibrated — a zero reading gives
    // hasWater=false (0 >= 25 is false), so pumps stay off, but we still guard
    // to avoid acting on a garbage percentage from an uncalibrated sensor.
    if (!cal_isLevelCalibrated(3)) return;

    // Overflow guard — C5 nearly full, stop RO inflow
    if (data->levelC5 >= (float)C5_LEVEL_OVERFLOW) {
        pump_stop(PUMP2_PIN);   // RO filter → C5
        if (!overflowC5) {
            overflowC5 = true;
            logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("C5 overflow protection: RO pump stopped"));
            comms_sendActuatorStatus();
        }
        return;
    }
    if (overflowC5 && data->levelC5 <= (float)C5_LEVEL_RESUME) {
        overflowC5 = false;
    }
    // Still draining toward resume — keep RO pump off but let normal logic run
    // so PUMP3/PUMP4/valves are managed correctly by the level checks below.
    if (overflowC5) {
        pump_stop(PUMP2_PIN);
    }

    bool hasWater = (data->levelC5 >= (float)C5_LEVEL_HIGH_CM);
    bool levelLow = (data->levelC5 <= (float)C5_LEVEL_LOW_CM);

    if (!hasWater || levelLow) {
        // Not enough water to test or pump — shut everything off
        if (lastC5Routing != 0) {
            lastC5Routing = 0;
            c5WasEmpty = true;
            pump_stop(PUMP3_PIN);
            valve_close(VALVE6_PIN);
            valve_close(VALVE7_PIN);
            pump_stop(PUMP4_PIN);
        }
        return;
    }

    // C5 just received a new fill — log quality snapshot once so operators
    // can see the incoming water state without spamming on every eval tick.
    if (c5WasEmpty && !firstFlush_isCalMode()) {
        c5WasEmpty = false;
        bool phOK   = (data->phC5 >= WQ_PH_MIN) && (data->phC5 <= WQ_PH_MAX);
        bool turbOK = (data->turbidityC5 <= WQ_TURBIDITY_MAX_NTU);
        if (!phOK) {
            logEvent(LOG_WARNING, LOG_CAT_SENSOR,
                     String("pH out of range on C5 fill: ") + String(data->phC5, 2));
        }
        if (!turbOK) {
            logEvent(LOG_WARNING, LOG_CAT_SENSOR,
                     String("Turbidity out of range on C5 fill: ") + String(data->turbidityC5, 1));
        }
    } else {
        c5WasEmpty = false;
    }

    // We have enough water — evaluate quality against PNSDW 2017.
    // Require QUALITY_PASS_REQUIRED consecutive passes before routing to C6.
    // This guards against residual ADC noise and pump-agitation transients
    // that could slip through even after per-sample averaging.
    if (pipeline_isWaterPotable(data)) {
        qualityFailCount = 0;
        if (qualityPassCount < QUALITY_PASS_REQUIRED) {
            qualityPassCount++;
        }

        if (qualityPassCount >= QUALITY_PASS_REQUIRED) {
            // ✓ CONFIRMED PASS → route to Container 6 (final potable storage)
            if (lastC5Routing != 3) {
                lastC5Routing = 3;
                pump_start(PUMP3_PIN);
                valve_open(VALVE7_PIN);
                valve_close(VALVE6_PIN);
                pump_stop(PUMP4_PIN);
            }
        } else {
            // Accumulating passes — hold in C5, keep pumps off
            if (lastC5Routing != 2) {
                lastC5Routing = 2;
                pump_stop(PUMP3_PIN);
                valve_close(VALVE6_PIN);
                valve_close(VALVE7_PIN);
                pump_stop(PUMP4_PIN);
            }
            Serial.print(F("[Quality] Pending pass "));
            Serial.print(qualityPassCount);
            Serial.print(F("/"));
            Serial.println(QUALITY_PASS_REQUIRED);
        }
    } else {
        // ✗ FAIL — reset pass streak, accumulate fail count
        qualityPassCount = 0;
        if (qualityFailCount < QUALITY_FAIL_REQUIRED) {
            qualityFailCount++;
        }

        if (qualityFailCount >= QUALITY_FAIL_REQUIRED) {
            // CONFIRMED FAIL → recycle to Container 4 for re-treatment.
            // Skip if C4 is at overflow — stage_container4 would immediately
            // kill P4/V6, and pumping into a full tank would worsen the overflow.
            if (cal_isLevelCalibrated(2) && data->levelC4 >= (float)C4_LEVEL_OVERFLOW) {
                // Hold in C5 until C4 drains enough to accept recycled water
                if (lastC5Routing != 2) {
                    lastC5Routing = 2;
                    pump_stop(PUMP3_PIN);
                    valve_close(VALVE6_PIN);
                    valve_close(VALVE7_PIN);
                    pump_stop(PUMP4_PIN);
                }
                Serial.println(F("[Quality] FAIL — recycle blocked, C4 full"));
            } else if (lastC5Routing != 4) {
                lastC5Routing = 4;
                pump_start(PUMP3_PIN);
                valve_close(VALVE7_PIN);
                valve_open(VALVE6_PIN);
                pump_start(PUMP4_PIN);
            }
            Serial.print(F("[Quality] FAIL ("));
            Serial.print(qualityFailCount);
            Serial.println(F(") -> water recycled to Container 4"));
        } else {
            // Accumulating fails — hold in C5, keep pumps off
            if (lastC5Routing != 2) {
                lastC5Routing = 2;
                pump_stop(PUMP3_PIN);
                valve_close(VALVE6_PIN);
                valve_close(VALVE7_PIN);
                pump_stop(PUMP4_PIN);
            }
            Serial.print(F("[Quality] Pending fail "));
            Serial.print(qualityFailCount);
            Serial.print(F("/"));
            Serial.println(QUALITY_FAIL_REQUIRED);
        }

        if (qualityFailCount == QUALITY_FAIL_ALERT) {
            logEvent(LOG_ERROR, LOG_CAT_SENSOR,
                     F("C5 quality: 10 consecutive failures — check sensors/filter"));
            Serial.println(F("[Quality] ALERT: 10 consecutive failures"));
        }
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
    // Skip entirely if C6 level sensor is not yet calibrated — prevents
    // factory-default readings from triggering a spurious emergency stop.
    if (!cal_isLevelCalibrated(4)) return;

    if (data->levelC6 >= (float)C6_LEVEL_OVERFLOW) {
        // ═══ CONTAINER 6 IS FULL — EMERGENCY STOP ═══
        if (!emergencyStopped) {
            Serial.println(F("!!! CONTAINER 6 FULL — EMERGENCY STOP !!!"));
            pipeline_emergencyStop();
            comms_sendActuatorStatus();
        }
    }
    else if (data->levelC6 <= (float)C6_LEVEL_RESUME) {
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
//  STAGE: Calibration Mode — Overflow Protection Only
// ═════════════════════════════════════════════════════════════════════════
//
//  During calibration, the automated pipeline is fully suspended so the
//  operator can freely open valves and run pumps manually.  However, once
//  a container's level sensor is fully calibrated, its overflow limit is
//  enforced on any currently-active manual commands.
//
//  Example: if V4 is opened manually and C4 reaches its overflow limit,
//  we close V4 (and any other inflow actuators for C4) and report back.
//
static void stage_calModeOverflow(const SensorData* data)
{
    // C2 overflow — close V1 (inlet from roof) if open.
    // Latch is set regardless of V1 state so a passive overflow is tracked correctly.
    if (cal_isLevelCalibrated(0) && data->levelC2 >= (float)C2_LEVEL_OVERFLOW) {
        if (actuator_isOn(VALVE1_PIN)) {
            valve_close(VALVE1_PIN);
        }
        if (!overflowC2) {
            overflowC2 = true;
            logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("CAL: C2 overflow — V1 closed"));
            comms_sendActuatorStatus();
        }
    } else if (overflowC2 && cal_isLevelCalibrated(0) && data->levelC2 <= (float)C2_LEVEL_RESUME) {
        overflowC2 = false;
    }

    // C3 overflow — close V2/V3/V4 and stop P1 if any are active.
    // Set the latch regardless of whether actuators were on so that the warning
    // and status push fire exactly once even if overflow was reached passively.
    if (cal_isLevelCalibrated(1) && data->levelC3 >= (float)C3_LEVEL_OVERFLOW) {
        bool anyOn = actuator_isOn(PUMP1_PIN) || actuator_isOn(VALVE2_PIN) ||
                     actuator_isOn(VALVE3_PIN) || actuator_isOn(VALVE4_PIN);
        if (anyOn) {
            pump_stop(PUMP1_PIN);
            valve_close(VALVE2_PIN);
            valve_close(VALVE3_PIN);
            valve_close(VALVE4_PIN);
        }
        if (!overflowC3) {
            overflowC3 = true;
            logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("CAL: C3 overflow — feed stopped"));
            comms_sendActuatorStatus();
        }
    } else if (overflowC3 && cal_isLevelCalibrated(1) && data->levelC3 <= (float)C3_LEVEL_RESUME) {
        overflowC3 = false;
    }

    // C4 overflow — close all C4 inflow paths if any are active.
    // Latch is set regardless so a passive overflow is tracked correctly.
    if (cal_isLevelCalibrated(2) && data->levelC4 >= (float)C4_LEVEL_OVERFLOW) {
        bool anyOn = actuator_isOn(PUMP1_PIN) || actuator_isOn(VALVE2_PIN) ||
                     actuator_isOn(VALVE4_PIN) || actuator_isOn(VALVE6_PIN) ||
                     actuator_isOn(PUMP4_PIN);
        if (anyOn) {
            pump_stop(PUMP1_PIN);
            valve_close(VALVE2_PIN);
            valve_close(VALVE4_PIN);
            valve_close(VALVE6_PIN);
            pump_stop(PUMP4_PIN);
        }
        if (!overflowC4) {
            overflowC4 = true;
            logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("CAL: C4 overflow — inflow stopped"));
            comms_sendActuatorStatus();
        }
    } else if (overflowC4 && cal_isLevelCalibrated(2) && data->levelC4 <= (float)C4_LEVEL_RESUME) {
        overflowC4 = false;
    }

    // C5 overflow — stop P2 (RO pump → C5) if running.
    // Latch is set regardless so a passive overflow is tracked correctly.
    if (cal_isLevelCalibrated(3) && data->levelC5 >= (float)C5_LEVEL_OVERFLOW) {
        if (actuator_isOn(PUMP2_PIN)) {
            pump_stop(PUMP2_PIN);
        }
        if (!overflowC5) {
            overflowC5 = true;
            logEvent(LOG_WARNING, LOG_CAT_SYSTEM, F("CAL: C5 overflow — P2 stopped"));
            comms_sendActuatorStatus();
        }
    } else if (overflowC5 && cal_isLevelCalibrated(3) && data->levelC5 <= (float)C5_LEVEL_RESUME) {
        overflowC5 = false;
    }

    // C6 overflow — emergency stop everything
    if (cal_isLevelCalibrated(4) && data->levelC6 >= (float)C6_LEVEL_OVERFLOW) {
        if (!emergencyStopped) {
            Serial.println(F("!!! CAL MODE: C6 FULL — EMERGENCY STOP !!!"));
            pipeline_emergencyStop();
            comms_sendActuatorStatus();
        }
    } else if (emergencyStopped && cal_isLevelCalibrated(4) && data->levelC6 <= (float)C6_LEVEL_RESUME) {
        emergencyStopped = false;
        Serial.println(F("[Container 6] Level safe — operations may resume"));
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
    overflowC2         = false;
    overflowC3         = false;
    overflowC4         = false;
    overflowC5         = false;
    qualityPassCount   = 0;
    qualityFailCount   = 0;
    // All actuators already OFF from actuators_init()
}

// ─────────────────────────────────────────────────────────────────────────
void pipeline_update(const SensorData* data)
{
    // If emergency-stopped, skip all stages
    if (emergencyStopped) return;

    // In calibration mode, suspend all automated pipeline logic so the
    // operator has unobstructed manual control over valves and pumps.
    // Only overflow protection for fully-calibrated containers still runs —
    // it guards against a manual command accidentally flooding a tank.
    if (firstFlush_isCalMode()) {
        stage_calModeOverflow(data);
        return;
    }

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
    // Reset quality counters — changing the filter path means the water
    // in C5 may no longer reflect the new path's output quality.
    qualityPassCount = 0;
    qualityFailCount = 0;
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
    return cal_isLevelCalibrated(4) && (data->levelC6 >= (float)C6_LEVEL_OVERFLOW);
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
    qualityPassCount = 0;
    qualityFailCount = 0;

    // Cancel backwash if it's running
    if (backwashState != BW_IDLE) {
        pipeline_stopBackwash();
    }

    // Kill every actuator via the centralised emergency stop
    actuators_emergencyStop();
}
