#ifndef COMMS_H
#define COMMS_H

#include <Arduino.h>
#include "sensors.h"
#include "first_flush.h"
#include "pipeline.h"

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  COMMUNICATIONS MODULE — Arduino Mega ↔ ESP32 Serial Link
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  The Mega handles all real-time sensor reading and actuator control.
 *  The ESP32 handles connectivity:
 *    ● WiFi access point / station
 *    ● MQTT or HTTP to forward sensor data to a backend server
 *    ● MongoDB writes (via REST API, MQTT bridge, or Realm Device Sync)
 *    ● Receives commands from the rainwater monitoring app and relays
 *      them to the Mega over this serial link
 *
 *  PHYSICAL WIRING:
 *    Mega TX1 (pin 18) ──→ ESP32 RX (e.g., GPIO16)
 *    Mega RX1 (pin 19) ←── ESP32 TX (e.g., GPIO17)
 *    GND ──────────────────  GND (common ground is mandatory!)
 *    ⚠️  Use a bi-directional logic-level shifter (5 V ↔ 3.3 V).
 *        The ESP32's GPIO is 3.3 V only — 5 V will damage it.
 *
 *  PROTOCOL (newline-terminated plain text, human-readable):
 *
 *  ── Mega → ESP32 (sensor telemetry, sent every COMMS_TX_INTERVAL_MS) ──
 *
 *    "S,FLOW,<L/min>\n"
 *    "S,LVL_C2,<cm>\n"  through  "S,LVL_C6,<cm>\n"
 *    "S,TEMP_C5,<°C>\n"    "S,PH_C5,<value>\n"    "S,TURB_C5,<NTU>\n"
 *    "S,TEMP_C6,<°C>\n"    "S,PH_C6,<value>\n"    "S,TURB_C6,<NTU>\n"
 *    "S,STATE,<ff>,<filter>,<bw>\n"
 *
 *  ── ESP32 → Mega (commands) ────────────────────────────────────────────
 *
 *    "C,FILTER,CHARCOAL\n"   → set charcoal-only mode
 *    "C,FILTER,BOTH\n"       → set charcoal + RO commercial mode
 *    "C,BACKWASH,START\n"    → begin charcoal filter backwash
 *    "C,BACKWASH,STOP\n"     → cancel backwash
 *    "C,ESTOP,ON\n"          → emergency stop everything
 *    "C,ESTOP,OFF\n"         → clear emergency stop, allow resume
 *
 *  SUGGESTIONS:
 *  ● Add a CRC8 or XOR checksum suffix to each line for data integrity,
 *    especially if the Mega ↔ ESP32 wires are long or near motor/relay
 *    noise sources.
 *  ● Implement ACK/NACK: after the Mega executes a command, echo back
 *    "A,<cmd>,OK\n" or "A,<cmd>,ERR\n" so the app knows it was received.
 *  ● For the ESP32 firmware, recommended libraries:
 *    • WiFi.h / WiFiManager — easy WiFi provisioning
 *    • PubSubClient or AsyncMqttClient — MQTT broker communication
 *    • ArduinoJson — parse/build JSON for REST API calls
 *    • ESPAsyncWebServer — optional local dashboard / OTA updates
 *  ● On the backend:
 *    • MQTT broker (Mosquitto or HiveMQ) as the message bus
 *    • Node.js / Python service subscribed to MQTT that writes to MongoDB
 *    • MongoDB time-series collections for efficient sensor data storage
 *    • MongoDB Atlas Charts or Grafana for live dashboards
 *  ● The ESP32 should buffer incoming sensor lines and batch-insert to
 *    the database at a lower frequency (e.g., every 30 s) to reduce
 *    write pressure and network traffic.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * Initialise the ESP32 serial link (Serial1).
 * Call once in setup().
 */
void comms_init();

/**
 * Transmit sensor data and system state to the ESP32.
 * Internally throttled by COMMS_TX_INTERVAL_MS — safe to call every loop.
 *
 * @param data        Current sensor readings
 * @param ffState     First flush diverter state
 * @param filterMode  Current filter routing mode
 * @param bwState     Current backwash state
 */
void comms_sendData(const SensorData* data,
                    FirstFlushState   ffState,
                    FilterMode        filterMode,
                    BackwashState     bwState);

/**
 * Check for and process incoming commands from the ESP32.
 * Non-blocking — reads whatever bytes are available and processes
 * complete lines.  Call every iteration of loop().
 *
 * @return true if a command was received and executed this tick
 */
bool comms_receiveCommands();

/**
 * Immediately push the current actuator states to the ESP32.
 * Call after any automated actuator change (overflow, dry-run, emergency)
 * so the dashboard reflects the real state without waiting for the next
 * scheduled telemetry frame.
 */
void comms_sendActuatorStatus();

#endif // COMMS_H
