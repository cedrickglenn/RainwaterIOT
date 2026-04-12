/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  RainwaterIOT — Rainwater Harvesting & Potable Water Treatment System
 *  Main Entry Point
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  OVERVIEW
 *  --------
 *  This firmware controls an automated system that turns raw rainwater into
 *  drinking water compliant with PNSDW 2017 (Philippine National Standards
 *  for Drinking Water).
 *
 *  Treatment chain:
 *
 *    Roof → [First Flush Divert] → Container 2 (buffer)
 *         → Charcoal Filter → (optional) Container 4 → Commercial RO
 *         → Container 5 (quality check) → Container 6 (final storage)
 *         ↻ fail? → recycle back to Container 4
 *
 *  HARDWARE
 *  --------
 *    Controller:    Arduino Mega 1280
 *    Connectivity:  ESP32 module on Serial1 (WiFi/IoT/MQTT → MongoDB)
 *    Actuators:     8× solenoid valves + 4× water pumps (relay-driven)
 *    Sensors:       1× flow, 5× ultrasonic, 2× temp, 2× pH, 2× turbidity
 *
 *  MODULE MAP
 *  ----------
 *    pins.h            All pin assignments (see ASCII wiring diagram)
 *    config.h          Timing constants, level thresholds, PNSDW limits
 *    actuators.h/cpp   Valve & pump control (centralised relay I/O)
 *    sensors.h/cpp     Sensor reading & data aggregation
 *    first_flush.h/cpp First flush diverter state machine
 *    pipeline.h/cpp    Multi-stage water treatment (containers 2–6)
 *    comms.h/cpp       ESP32 serial protocol (telemetry + commands)
 *    main.cpp          ← YOU ARE HERE — system orchestrator
 *
 *  MAIN LOOP ARCHITECTURE
 *  ----------------------
 *  The loop runs continuously (no delay()) and calls each module's
 *  non-blocking update() function.  Sensor reads are throttled to
 *  SENSOR_READ_INTERVAL_MS from config.h.
 *
 *    ┌───────── loop() ─────────────────────────────────────────────┐
 *    │                                                              │
 *    │  1. Read sensors (every SENSOR_READ_INTERVAL_MS)             │
 *    │  2. Update first-flush state machine                         │
 *    │  3. Update water treatment pipeline (containers 2–6)         │
 *    │  4. Send telemetry to ESP32 / receive commands               │
 *    │                                                              │
 *    └──────────────────────────────────────────────────────────────┘
 *
 *  OVERALL SUGGESTIONS & IDEAS
 *  ---------------------------
 *
 *  1. POWER RESILIENCE
 *     ● Add a small UPS (e.g., 12 V lead-acid + charger) for the
 *       controller and relay bank.  A power outage mid-process could
 *       leave valves open and flood the area.
 *     ● Implement a "safe shutdown" ISR on a voltage-divider input:
 *       if the main supply drops, close all valves before the UPS dies.
 *
 *  2. LOGGING & ANALYTICS
 *     ● Every state transition, sensor anomaly, and actuator event
 *       should be timestamped and logged.  Use the ESP32's NTP client
 *       and send events as JSON over MQTT.
 *     ● MongoDB time-series collections are ideal for sensor data.
 *       Build dashboards (Atlas Charts / Grafana) showing:
 *         - daily collected volume
 *         - filter efficiency over time
 *         - water quality trends (pH, turbidity)
 *         - pump duty cycles & runtime hours
 *
 *  3. MAINTENANCE AUTOMATION
 *     ● Track cumulative litres through each filter stage (flow sensor
 *       integration).  Alert when the RO membrane approaches its rated
 *       capacity (manufacturer datasheet — typically 5 000–10 000 L).
 *     ● Auto-schedule charcoal backwash based on volume processed rather
 *       than a calendar timer (volume-based is more reliable).
 *     ● Track pump ON-hours for predictive maintenance.
 *
 *  4. SAFETY
 *     ● Leak detection: add low-cost water-presence sensors at the base
 *       of each container.  A single wet probe → emergency stop + alert.
 *     ● Install a physical emergency stop button (normally-closed in
 *       series with the relay power rail).  No microcontroller dependency.
 *     ● Add ACS712 current sensors on pump power lines to detect
 *       dry-running (near-zero current) and stalls (over-current).
 *
 *  5. SCALABILITY
 *     ● The modular architecture (one .h/.cpp pair per concern) makes it
 *       easy to add new stages (e.g., UV disinfection, chlorine dosing)
 *       without touching existing files.
 *     ● If GPIO count becomes limiting, consider migrating to an
 *       ESP32-S3 as the main controller (built-in WiFi, enough GPIO,
 *       dual-core — eliminates the Mega + ESP32 split).
 *
 *  6. DISINFECTION (POST-FILTRATION)
 *     ● A UV-C lamp stage between Container 5 and Container 6 provides
 *       an additional barrier against pathogens — highly recommended
 *       for any point-of-use potable water system.
 *     ● PNSDW 2017 requires bacteriological compliance (zero E. coli,
 *       < 1 CFU/100 mL total coliform).  Automated coliform sensors are
 *       expensive, so schedule periodic manual lab tests and log
 *       compliance dates in the database.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include "pins.h"
#include "config.h"
#include "actuators.h"
#include "calibration.h"
#include "sensors.h"
#include "first_flush.h"
#include "pipeline.h"
#include "comms.h"
#include "logger.h"

// ── Timing tracker ──────────────────────────────────────────────────────
static unsigned long lastSensorReadMs = 0;

// ── Global sensor data (single instance, re-used every cycle) ───────────
static SensorData sensorData;

// ═════════════════════════════════════════════════════════════════════════
//  SETUP — runs once on power-up / reset
// ═════════════════════════════════════════════════════════════════════════
void setup()
{
    // ── 1. Debug serial (USB) + event log serial (Serial2) ─────────
    Serial.begin(DEBUG_BAUD_RATE);
    logger_init();
    Serial.println();
    Serial.println(F("=========================================================="));
    Serial.println(F("  RainwaterIOT — Rainwater Harvesting & Treatment System"));
    Serial.println(F("  Firmware booting..."));
    Serial.println(F("=========================================================="));

    // ── 2. Actuators FIRST — ensures all relays are OFF before any
    //       sensor initialisation triggers an interrupt or side effect.
    actuators_init();
    Serial.println(F("[Init] Actuators   OK — all relays OFF (safe state)"));

    // ── 3. Calibration — load from EEPROM before sensors read anything
    cal_init();
    Serial.println(F("[Init] Calibration OK — values loaded from EEPROM"));

    // ── 4. Sensors — attaches flow sensor ISR, starts OneWire & pH
    sensors_init();
    Serial.println(F("[Init] Sensors     OK — flow ISR attached, OneWire + pH started"));

    // ── 5. First flush diverter state machine
    firstFlush_init();
    Serial.println(F("[Init] FirstFlush  OK — IDLE, valves 1 & 8 closed"));

    // ── 6. Water treatment pipeline (containers 2–6)
    pipeline_init();
    Serial.println(F("[Init] Pipeline    OK — default mode: CHARCOAL + RO"));

    // ── 7. ESP32 communication link
    comms_init();
    Serial.println(F("[Init] Comms       OK — ESP32 link on Serial1 @ 9600 baud"));

    // ── Boot complete ───────────────────────────────────────────────
    Serial.println(F("=========================================================="));
    Serial.println(F("  System Ready — Monitoring for rain..."));
    Serial.println(F("=========================================================="));
    Serial.println();

    logEvent(LOG_INFO, LOG_CAT_SYSTEM, F("Boot complete"));
}

// ═════════════════════════════════════════════════════════════════════════
//  LOOP — runs continuously, non-blocking
// ═════════════════════════════════════════════════════════════════════════
void loop()
{
    unsigned long now = millis();

    // ── 1. READ SENSORS (throttled) ─────────────────────────────────
    //    All sensor values are packed into the sensorData struct and
    //    shared with every downstream module.  Reading too fast wastes
    //    CPU and doesn't improve accuracy (especially for ultrasonic
    //    and DS18B20 which need physical settling time).
    if ((now - lastSensorReadMs) >= SENSOR_READ_INTERVAL_MS) {
        lastSensorReadMs = now;
        sensors_readAll(&sensorData);
    }

    // ── 2. FIRST FLUSH DIVERTER ─────────────────────────────────────
    //    Controls Valve 1 (to Container 2) and Valve 8 (to drainage).
    //    Must run every tick for responsive state transitions.
    firstFlush_update(sensors_isFlowActive());

    // ── 3. WATER TREATMENT PIPELINE ─────────────────────────────────
    //    Manages all stages from Container 2 through Container 6.
    //    Internally checks for Container 6 overflow (emergency stop).
    pipeline_update(&sensorData);

    // ── 4. ESP32 COMMUNICATION ──────────────────────────────────────
    //    Send sensor telemetry (throttled by COMMS_TX_INTERVAL_MS).
    //    Receive and execute any incoming commands.
    comms_sendData(
        &sensorData,
        firstFlush_getState(),
        pipeline_getFilterMode(),
        pipeline_getBackwashState()
    );
    comms_receiveCommands();

    // ── 5. DEBUG OUTPUT (optional) ──────────────────────────────────
    //    Uncomment the block below to see a periodic sensor snapshot
    //    in the Arduino Serial Monitor.  Disable in production to save
    //    CPU time and serial bandwidth.
    static unsigned long lastDebugMs = 0;
    if ((now - lastDebugMs) >= 5000) {
        lastDebugMs = now;
        Serial.println(F("---- Sensor Snapshot ----"));
        Serial.print(F("Flow:     ")); Serial.print(sensorData.flowRate);       Serial.println(F(" L/min"));
        Serial.print(F("C2 lvl:   ")); Serial.print(sensorData.levelC2, 1);     Serial.println(F(" cm"));
        Serial.print(F("C2 temp:  ")); Serial.print(sensorData.tempC2, 1);      Serial.println(F(" C"));
        Serial.print(F("C2 pH:    ")); Serial.println(sensorData.phC2, 2);
        Serial.print(F("C2 turb:  ")); Serial.print(sensorData.turbidityC2, 1); Serial.println(F(" NTU"));
        Serial.print(F("C3 lvl:   ")); Serial.print(sensorData.levelC3, 1);     Serial.println(F(" cm"));
        Serial.print(F("C4 lvl:   ")); Serial.print(sensorData.levelC4, 1);     Serial.println(F(" cm"));
        Serial.print(F("C5 lvl:   ")); Serial.print(sensorData.levelC5, 1);     Serial.println(F(" cm"));
        Serial.print(F("C5 temp:  ")); Serial.print(sensorData.tempC5, 1);      Serial.println(F(" C"));
        Serial.print(F("C5 pH:    ")); Serial.println(sensorData.phC5, 2);
        Serial.print(F("C5 turb:  ")); Serial.print(sensorData.turbidityC5, 1); Serial.println(F(" NTU"));
        Serial.print(F("C6 lvl:   ")); Serial.print(sensorData.levelC6, 1);     Serial.println(F(" cm"));
        Serial.print(F("C6 temp:  ")); Serial.print(sensorData.tempC6, 1);      Serial.println(F(" C"));
        Serial.print(F("C6 pH:    ")); Serial.println(sensorData.phC6, 2);
        Serial.print(F("C6 turb:  ")); Serial.print(sensorData.turbidityC6, 1); Serial.println(F(" NTU"));
        Serial.print(F("FF state: ")); Serial.println(firstFlush_getState());
        Serial.println(F("-------------------------"));
    }
}
