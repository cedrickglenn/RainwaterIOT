#ifndef PIPELINE_H
#define PIPELINE_H

#include <Arduino.h>
#include "sensors.h"

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  WATER TREATMENT PIPELINE — Container Operations (Stages 3–7)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  After rainwater is collected into Container 2 (handled by first_flush),
 *  this module manages the multi-stage treatment pipeline:
 *
 *  Container 2 (buffer)
 *       │ valve 2 + pump 1
 *       ↓
 *  Charcoal Filter (Container 3)
 *       ├── valve 3 → Container 6 DIRECT          (charcoal-only mode)
 *       ├── valve 4 → Container 4 → RO → C5 → C6  (charcoal + RO mode)
 *       └── valve 5 + valve 8 → drainage           (backwash mode)
 *
 *  Container 4 (pre-RO buffer)
 *       │ pump 2
 *       ↓
 *  Commercial RO Filter → Container 5
 *       ├── valve 7 + pump 3 → Container 6         (water PASSES quality)
 *       └── valve 6 + pump 3 + pump 4 → Container 4 (water FAILS → recycle)
 *
 *  Container 6 (final storage)
 *       └── EMERGENCY STOP if full!
 *
 *  KEY DESIGN DECISIONS:
 *  ● Each stage has its own internal function and runs semi-independently.
 *  ● Stages use hysteresis thresholds (HIGH/LOW) to prevent relay chatter.
 *  ● Backwash is a special mode that takes exclusive control of the
 *    Container 2 → Charcoal Filter path.
 *  ● Container 6 full → immediate emergency stop of ALL upstream stages.
 *
 *  SUGGESTIONS:
 *  ● Track per-pump runtime hours for maintenance scheduling.
 *    (e.g., replace RO membrane every 2 000 hours of pump 2 runtime).
 *  ● Add water volume estimation per container:
 *    volume = containerArea × (containerHeight − ultrasonicReading).
 *  ● Implement "stage lockout": if a downstream container is nearly full,
 *    explicitly stop the upstream pump even if the upstream level looks OK.
 *  ● After pumping stops, add a 30–60 s delay before reading turbidity
 *    in Container 5.  Agitation from pumping causes transient spikes.
 *  ● Consider a "max recycle count" for Container 5 failures — after N
 *    consecutive fails, stop recycling and alert the user.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * Filter routing mode — set by commands received from ESP32/app.
 *
 * CHARCOAL_ONLY:
 *   Water from the charcoal filter is routed directly to Container 6 via
 *   Valve 3, bypassing the commercial RO filter entirely.  Use when the
 *   charcoal filter alone produces adequate quality (light rain, clean
 *   roof) or when the RO system is offline for maintenance.
 *
 * CHARCOAL_AND_RO:
 *   Water passes through charcoal → Container 4 → RO → Container 5
 *   (quality check) → Container 6.  Full treatment chain.
 *
 * SUGGESTION: Add a FILTER_OFF mode that stops all treatment (e.g., when
 * you want to manually bypass everything for maintenance).
 */
enum FilterMode {
    FILTER_CHARCOAL_ONLY,
    FILTER_CHARCOAL_AND_RO
};

/**
 * Backwash progress states.  The backwash procedure repeatedly fills the
 * charcoal filter vessel with water and drains it to flush sediment.
 */
enum BackwashState {
    BW_IDLE,       // Not backwashing — normal operation
    BW_FILLING,    // Pumping water from Container 2 into the filter
    BW_DRAINING,   // Filter full → draining through valve 5 + valve 8
    BW_COMPLETE    // All requested cycles finished
};

// ── Public API ──────────────────────────────────────────────────────────

/** Initialise pipeline state. Call once in setup(). */
void pipeline_init();

/**
 * Run one tick of every pipeline stage.
 * Call every iteration of loop().
 * @param data  Current sensor readings from sensors_readAll()
 */
void pipeline_update(const SensorData* data);

/** Set the filter routing mode (typically from an ESP32 command). */
void pipeline_setFilterMode(FilterMode mode);

/** Get the current filter routing mode. */
FilterMode pipeline_getFilterMode();

/** Start a backwash operation (BACKWASH_CYCLES fill-drain cycles). */
void pipeline_startBackwash();

/** Cancel / abort an in-progress backwash. */
void pipeline_stopBackwash();

/** Get the current backwash state. */
BackwashState pipeline_getBackwashState();

/**
 * Check Container 6 for overflow.
 * @return true if Container 6 level indicates FULL
 */
bool pipeline_isTankFull(const SensorData* data);

/**
 * Evaluate Container 5 water against PNSDW 2017 standards.
 * @return true if water is potable
 */
bool pipeline_isWaterPotable(const SensorData* data);

/** Force-stop all pipeline stages immediately (emergency). */
void pipeline_emergencyStop();

#endif // PIPELINE_H
