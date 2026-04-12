/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  RELAY TEST SKETCH
 *  RainwaterIOT — Step 2: Bench Test (Dry, No Water)
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  PURPOSE:
 *    Cycles each relay (valve/pump) one at a time with a 2-second ON window.
 *    Listen for a click from each relay module channel.
 *    Verify the correct physical valve/pump activates for each pin.
 *
 *  HOW TO RUN:
 *    1. In platformio.ini, temporarily change:
 *         src_dir = test
 *       Then upload. Revert src_dir after testing.
 *    OR — simply paste this into src/main.cpp temporarily and upload.
 *
 *  WHAT TO LISTEN FOR:
 *    Each relay should produce a distinct CLICK when it turns ON,
 *    and another CLICK when it turns OFF.
 *    If a relay doesn't click → wiring fault or dead relay channel.
 *    If the wrong physical valve activates → label mismatch, re-check wiring.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include "pins.h"

// All relay pins in test order
struct RelayEntry {
    uint8_t     pin;
    const char* label;
};

static const RelayEntry RELAYS[] = {
    { VALVE1_PIN, "Valve 1 — C1 to C2 (post first flush)"         },
    { VALVE2_PIN, "Valve 2 — C2 to C3 charcoal intake"            },
    { PUMP1_PIN,  "Pump 1  — C2 to C3 transfer"                   },
    { VALVE3_PIN, "Valve 3 — C3 to C6 direct (charcoal only)"     },
    { VALVE4_PIN, "Valve 4 — C3 to C4 (RO path)"                  },
    { PUMP2_PIN,  "Pump 2  — C4 to RO filter"                     },
    { PUMP3_PIN,  "Pump 3  — C5 output"                           },
    { PUMP4_PIN,  "Pump 4  — C5 to C4 recycle"                    },
    { VALVE5_PIN, "Valve 5 — C3 backwash drain"                   },
    { VALVE6_PIN, "Valve 6 — C5 to C4 recycle path"               },
    { VALVE7_PIN, "Valve 7 — C5 to C6 (water pass)"               },
    { VALVE8_PIN, "Valve 8 — First flush drainage"                 },
};

static const uint8_t RELAY_COUNT = sizeof(RELAYS) / sizeof(RELAYS[0]);

// How long each relay stays ON during the test
static const unsigned long ON_DURATION_MS  = 2000;
// Gap between relays (all OFF)
static const unsigned long OFF_DURATION_MS = 1000;

void setup()
{
    Serial.begin(115200);
    Serial.println(F("=========================================================="));
    Serial.println(F("  RainwaterIOT — Relay Test"));
    Serial.println(F("  Each relay will turn ON for 2s then OFF for 1s."));
    Serial.println(F("  Listen for clicks and verify correct valve/pump."));
    Serial.println(F("=========================================================="));
    Serial.println();

    // Initialise all relay pins as OUTPUT and set to HIGH (OFF for active-LOW)
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {
        pinMode(RELAYS[i].pin, OUTPUT);
        digitalWrite(RELAYS[i].pin, HIGH);  // active-LOW: HIGH = OFF
    }

    delay(1000);  // short pause before starting
}

void loop()
{
    for (uint8_t i = 0; i < RELAY_COUNT; i++) {

        // ── Print which relay is about to activate ──────────────────
        Serial.print(F("["));
        Serial.print(i + 1);
        Serial.print(F("/"));
        Serial.print(RELAY_COUNT);
        Serial.print(F("] ON  → Pin "));
        Serial.print(RELAYS[i].pin);
        Serial.print(F(" — "));
        Serial.println(RELAYS[i].label);

        // ── Turn ON (active-LOW) ────────────────────────────────────
        digitalWrite(RELAYS[i].pin, LOW);
        delay(ON_DURATION_MS);

        // ── Turn OFF ────────────────────────────────────────────────
        digitalWrite(RELAYS[i].pin, HIGH);
        Serial.print(F("        OFF ← Pin "));
        Serial.println(RELAYS[i].pin);
        delay(OFF_DURATION_MS);
    }

    // ── Full cycle complete ─────────────────────────────────────────
    Serial.println();
    Serial.println(F("=========================================================="));
    Serial.println(F("  Full cycle complete. Starting again in 3 seconds..."));
    Serial.println(F("  Press reset button to stop."));
    Serial.println(F("=========================================================="));
    Serial.println();
    delay(3000);
}
