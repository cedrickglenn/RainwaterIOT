#ifndef FIRST_FLUSH_H
#define FIRST_FLUSH_H

#include <Arduino.h>

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  FIRST FLUSH DIVERTER — State Machine
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  The "first flush" is the initial volume of rainwater that washes
 *  accumulated contaminants (dust, bird droppings, leaves, pollen, etc.)
 *  off the roof and guttering.  By diverting this dirty water to drainage
 *  we significantly improve the quality of collected rainwater.
 *
 *  This module implements a software-controlled first flush diverter using
 *  the water flow sensor (between Container 1 and 2) and two solenoid
 *  valves:
 *    ● Valve 1 — pass-through to Container 2 (collection)
 *    ● Valve 8 — divert to drainage (first flush waste)
 *
 *  STATE MACHINE:
 *
 *  ┌──────┐  flow detected  ┌────────────┐  confirmed (3 s)  ┌───────────┐  5 min  ┌────────────┐
 *  │ IDLE │ ─────────────→ │ CONFIRMING │ ────────────────→ │ DIVERTING │ ──────→ │ COLLECTING │
 *  └──────┘                └────────────┘                   └───────────┘         └────────────┘
 *     ↑                         │                                │                      │
 *     │       flow stops        │            flow timeout        │                      │
 *     └─────────────────────────┘                                │                      │
 *     ↑                                                          │                      │
 *     └──────────────────────────────────────────────────────────┘                      │
 *     ↑                                                                                 │
 *     │               flow stops AND collection window expired                          │
 *     └─────────────────────────────────────────────────────────────────────────────────┘
 *
 *  IDLE:
 *    All valves closed.  Flow sensor is monitored passively — no valve wear,
 *    no power draw.
 *    → CONFIRMING when flow first crosses FLOW_MIN_THRESHOLD.
 *
 *  CONFIRMING:
 *    Still all valves closed.  Waits FLOW_CONFIRM_MS of sustained flow
 *    before committing to open V8.  This filters splashes, sensor transients,
 *    and lets water travel the pipe length from the sensor to the valve.
 *    → DIVERTING once confirmed.
 *    → IDLE if flow drops before confirmation.
 *
 *  DIVERTING:
 *    Valve 8 OPEN (to drainage), Valve 1 CLOSED.
 *    Accumulating consistent-flow time.  Brief gaps (< FLOW_TIMEOUT_MS)
 *    are tolerated without resetting the timer — tropical rain often has
 *    short pauses.
 *    → COLLECTING when accumulated consistent flow ≥ FIRST_FLUSH_DURATION_MS.
 *    → IDLE if flow stops for > FLOW_TIMEOUT_MS.
 *
 *  COLLECTING:
 *    Valve 8 CLOSED, Valve 1 OPEN.  Water flows to Container 2.
 *    A protected window of COLLECTION_WINDOW_MS ensures we don't
 *    flip-flop back to IDLE during intermittent rain.  Even if flow
 *    pauses, we stay in COLLECTING for the full window.
 *    → IDLE when window expires AND flow has been absent > FLOW_TIMEOUT_MS.
 *
 *  SUGGESTIONS:
 *  ● Add a tipping-bucket rain gauge as an independent rain confirmation
 *    signal.  Cross-referencing flow sensor + rain gauge gives high
 *    confidence that it's actually raining (vs. a burst pipe or sensor
 *    noise).
 *  ● Track the total first-flush volume diverted (integrate flow rate ×
 *    time).  For a given roof area you can compute the "mm of rainfall
 *    diverted" and compare against published first-flush guidelines
 *    (typically 0.5–2 mm equivalent).
 *  ● Consider a configurable "dry-season multiplier" that extends the
 *    first flush duration after long dry periods (roofs accumulate more
 *    contaminants).  The app could send this multiplier via ESP32.
 *  ● Log every rain event's start time, duration, volume collected, and
 *    volume diverted to the database for historical analysis and water
 *    balance calculations.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * First-flush diverter states.
 */
enum FirstFlushState {
    FF_IDLE,          // No rain — everything closed, flow sensor watching
    FF_CONFIRMING,    // Flow detected — waiting FLOW_CONFIRM_MS before opening V8
    FF_DIVERTING,     // Confirmed rain — diverting first flush to drain
    FF_COLLECTING     // First flush done — collecting into Container 2
};

/**
 * Initialise the first flush module.
 * Sets both valves to closed and enters IDLE.
 * Call once in setup() after actuators_init() and sensors_init().
 */
void firstFlush_init();

/**
 * Run one tick of the first flush state machine.
 * Must be called every iteration of loop() (it is non-blocking).
 *
 * @param flowActive  true if the flow sensor reads above FLOW_MIN_THRESHOLD
 */
void firstFlush_update(bool flowActive);

/**
 * Return the current state (for reporting to ESP32 / debug).
 */
FirstFlushState firstFlush_getState();

/**
 * Force-reset to IDLE and close both valves.
 * Used by emergency stop or a remote reset command.
 */
void firstFlush_reset();

#endif // FIRST_FLUSH_H
