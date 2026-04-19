#ifndef PINS_H
#define PINS_H

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  PIN ASSIGNMENT MAP — Arduino Mega 1280
 *  RainwaterIOT — Rainwater Harvesting & Potable Water Treatment System
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  SYSTEM LAYOUT (physical piping):
 *
 *  ┌─────────────┐  valve1   ┌─────────────┐  valve2+pump1  ┌──────────────────┐
 *  │ Container 1 │─────────→│ Container 2 │──────────────→│ Charcoal Filter  │
 *  │ (Catchment) │          │ (Buffer)    │               │   (Container 3)  │
 *  └──────┬──────┘          └─────────────┘               └──┬────┬────┬─────┘
 *         │                                                  │    │    │
 *      valve8                                             valve3 valve4 valve5
 *         │                                                  │    │    │
 *      [drain]                                               │    │  [drain/
 *                                                            │    │  backwash]
 *                                         ┌──────────────────┘    │
 *                                         ↓                       ↓
 *  ┌─────────────┐  valve7  ┌─────────────┐  pump2   ┌─────────────┐
 *  │ Container 6 │←────────│ Container 5 │←────────│ Container 4 │
 *  │ (Final      │         │ (Quality    │ (comm.  │ (Pre-RO     │
 *  │  Storage)   │         │  Check)     │ filter) │  Buffer)    │
 *  └─────────────┘         └──────┬──────┘         └─────────────┘
 *        ↑ (valve3                │                       ↑
 *         bypass)          valve6 + pump4                 │
 *                                 │       (feedback loop) │
 *                                 └───────────────────────┘
 *
 *  PIN GROUPS (chosen to avoid conflicts on the Mega 2560):
 *    22–33   Relay-driven solenoid valves (active-LOW) — V1–V8 on 22/23/25/26/30–33
 *    24/27–29 Relay-driven pumps (active-LOW) — P1–P4
 *    21      Water flow sensor (interrupt-capable — INT2)
 *    35–53   Ultrasonic sensor Trig/Echo pairs
 *    43/37/51 DS18B20 temperature sensors (OneWire, digital)
 *    A0/A1/A5/A6/A10/A11  Analog sensors (pH × 3, turbidity × 3)
 *    18/19   Serial1 TX/RX → ESP32 communication (hardware UART)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

// ── Solenoid Valves (normally-closed, relay active-LOW) ─────────────────
#define VALVE1_PIN   22   // Container 1 → Container 2 (pass-through after first flush)
#define VALVE2_PIN   23   // Container 2 output (to charcoal filter intake)
#define VALVE3_PIN   25   // Charcoal filter → Container 6 DIRECT (bypasses RO)
#define VALVE4_PIN   26   // Charcoal filter → Container 4 (to commercial RO path)
#define VALVE5_PIN   30   // Charcoal filter drainage (backwash drain outlet)
#define VALVE6_PIN   31   // Container 5 feedback path (bad water → Container 4)
#define VALVE7_PIN   32   // Container 5 → Container 6 (good water pass-through)
#define VALVE8_PIN   33   // First flush diverter (routes initial rain to drainage)

// ── Pumps (relay active-LOW) ────────────────────────────────────────────
#define PUMP1_PIN    24   // Pushes water: Container 2 → charcoal filter
#define PUMP2_PIN    27   // Pushes water: Container 4 → commercial RO filter
#define PUMP3_PIN    28   // Pushes water: Container 5 output (to valve 7 or valve 6)
#define PUMP4_PIN    29   // Feedback booster: Container 5 → Container 4 (recycle)

// ── Water Flow Sensor (YF-S201) ─────────────────────────────────────────
//    Pin 21 = INT2 on the Mega (interrupt-capable).
#define FLOW_SENSOR_PIN  21

// ── Ultrasonic Sensors (HC-SR04) — Trigger / Echo pairs ─────────────────
#define US_C2_TRIG   45   // Container 2 — buffer storage water level
#define US_C2_ECHO   44

#define US_C3_TRIG   42   // Container 3 — charcoal filter level (backwash monitoring)
#define US_C3_ECHO   41

#define US_C4_TRIG   39   // Container 4 — pre-commercial RO buffer level
#define US_C4_ECHO   38

#define US_C5_TRIG   36   // Container 5 — post-treatment quality-check tank level
#define US_C5_ECHO   35

#define US_C6_TRIG   53   // Container 6 — final potable water storage level
#define US_C6_ECHO   52

// ── DS18B20 Temperature Sensors (OneWire, digital) ──────────────────────
//    Each sensor lives on its own OneWire bus (separate pins). This avoids
//    having to enumerate ROM addresses — simpler wiring and code.
#define TEMP_C2_PIN  43   // Container 2 (buffer storage) water temperature
#define TEMP_C5_PIN  37   // Container 5 water temperature
#define TEMP_C6_PIN  51   // Container 6 water temperature

// ── pH Sensors — DFRobot SEN0161-V2 (analog) ───────────────────────────
#define PH_C2_PIN    A11  // Container 2 (buffer storage) pH  — post-first-flush rainwater
#define PH_C5_PIN    A6   // Container 5 pH
#define PH_C6_PIN    A1   // Container 6 pH

// ── Turbidity Sensors (analog) ──────────────────────────────────────────
#define TURB_C2_PIN  A10  // Container 2 (buffer storage) turbidity — post-first-flush
#define TURB_C5_PIN  A5   // Container 5 turbidity
#define TURB_C6_PIN  A0   // Container 6 turbidity

// ── ESP32 Communication ─────────────────────────────────────────────────
//    Uses hardware Serial1 on the Mega (TX1 = pin 18, RX1 = pin 19).
//    Wiring: Mega TX1 → ESP32 RX    |    Mega RX1 ← ESP32 TX
//    ⚠️  IMPORTANT: Use a bi-directional logic-level shifter (5 V ↔ 3.3 V)
//        between the Mega and ESP32! The ESP32 is NOT 5 V-tolerant on GPIO.
//        Failure to level-shift WILL damage the ESP32.

#endif // PINS_H
