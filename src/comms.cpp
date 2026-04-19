#include "comms.h"
#include "config.h"
#include "pipeline.h"
#include "actuators.h"
#include "first_flush.h"
#include "calibration.h"
#include "sensors.h"
#include "pins.h"
#include "logger.h"

/*
 *  IMPLEMENTATION NOTES
 *
 *  ● Outgoing telemetry is sent as one-line-per-value CSV messages.
 *    The ESP32 can parse these trivially with strtok() or split().
 *  ● Incoming commands are accumulated into a 64-byte line buffer.
 *    When a newline arrives, the complete line is parsed and executed.
 *  ● Serial1 receive buffer on the Mega is 64 bytes by default.  If you
 *    experience dropped bytes, increase SERIAL_RX_BUFFER_SIZE in the
 *    Arduino core — or call comms_receiveCommands() more frequently.
 *
 *  SUGGESTION FOR ESP32 SIDE (starter firmware outline):
 *
 *    #include <WiFi.h>
 *    #include <PubSubClient.h>   // MQTT
 *    #include <ArduinoJson.h>
 *
 *    void setup() {
 *      Serial.begin(9600);       // Link to Mega (Serial1 on the Mega side)
 *      WiFi.begin(SSID, PASS);
 *      mqttClient.setServer(BROKER_IP, 1883);
 *    }
 *
 *    void loop() {
 *      // 1. Read lines from Mega
 *      if (Serial.available()) {
 *        String line = Serial.readStringUntil('\n');
 *        // Parse "S,<key>,<value>" and build a JSON document
 *        // Publish to MQTT topic "rainwater/sensors/<key>"
 *      }
 *
 *      // 2. Check for commands from MQTT (subscribed to "rainwater/cmd")
 *      mqttClient.loop();
 *      // When a command arrives, forward it to the Mega:
 *      //   Serial.println("C,FILTER,BOTH");
 *
 *      // 3. Periodically batch-insert sensor JSON to MongoDB via
 *      //    HTTP POST to a Node.js/Express API endpoint, or use
 *      //    MongoDB Realm / Atlas Device Sync for direct writes.
 *    }
 *
 *  See the project README (or ask!) for a full ESP32 sketch template.
 */

// ── Transmit throttle ───────────────────────────────────────────────────
static unsigned long lastTxMs = 0;

// ── Receive line buffer ─────────────────────────────────────────────────
#define RX_BUF_SIZE 64
static char    rxBuf[RX_BUF_SIZE];
static uint8_t rxIdx = 0;

// ═════════════════════════════════════════════════════════════════════════
void comms_init()
{
    // Serial1 was already started in logger_init() so boot-time log frames
    // reach the ESP32 before this point.  Calling begin() again is harmless.
    Serial1.begin(ESP32_BAUD_RATE);
    Serial.println(F("[Comms] ESP32 link initialised on Serial1"));
}

// ── Helper: send one "S,<key>,<value>\n" line ───────────────────────────
static void sendLine(const char* key, float value, uint8_t decimals = 2)
{
    Serial1.print(F("S,"));
    Serial1.print(key);
    Serial1.print(',');
    Serial1.println(value, decimals);
}

// ═════════════════════════════════════════════════════════════════════════
void comms_sendData(const SensorData* data,
                    FirstFlushState   ffState,
                    FilterMode        filterMode,
                    BackwashState     bwState)
{
    unsigned long now = millis();
    if ((now - lastTxMs) < COMMS_TX_INTERVAL_MS) return;
    lastTxMs = now;

    // ── Calibrated sensor values ─────────────────────────────────────
    sendLine("FLOW",    data->flowRate,    2);
    sendLine("LVL_C2",      data->levelC2,        1);
    sendLine("TEMP_C2",     data->tempC2,         1);
    sendLine("TEMP_DEV_C2", data->tempDevicesC2,  0);
    sendLine("PH_C2",       data->phC2,           2);
    sendLine("TURB_C2",     data->turbidityC2,    1);
    sendLine("LVL_C3",      data->levelC3,        1);
    sendLine("LVL_C4",      data->levelC4,        1);
    sendLine("LVL_C5",      data->levelC5,        1);
    sendLine("TEMP_C5",     data->tempC5,         1);
    sendLine("TEMP_DEV_C5", data->tempDevicesC5,  0);
    sendLine("PH_C5",       data->phC5,           2);
    sendLine("TURB_C5",     data->turbidityC5,    1);
    sendLine("LVL_C6",      data->levelC6,        1);
    sendLine("TEMP_C6",     data->tempC6,         1);
    sendLine("TEMP_DEV_C6", data->tempDevicesC6,  0);
    sendLine("PH_C6",       data->phC6,           2);
    sendLine("TURB_C6",     data->turbidityC6,    1);

    // ── Raw (pre-calibration) values — used by the calibration dashboard
    sendLine("RAW_DIST_C2",   data->rawDistC2,   1);
    sendLine("RAW_DIST_C3",   data->rawDistC3,   1);
    sendLine("RAW_DIST_C4",   data->rawDistC4,   1);
    sendLine("RAW_DIST_C5",   data->rawDistC5,   1);
    sendLine("RAW_DIST_C6",   data->rawDistC6,   1);
    sendLine("RAW_MV_C2",     data->rawMvC2,     0);
    sendLine("RAW_MV_C5",     data->rawMvC5,     0);
    sendLine("RAW_MV_C6",     data->rawMvC6,     0);
    sendLine("RAW_TURB_V_C2", data->rawTurbVC2,  3);
    sendLine("RAW_TURB_V_C5", data->rawTurbVC5,  3);
    sendLine("RAW_TURB_V_C6", data->rawTurbVC6,  3);
    sendLine("RAW_TEMP_C2",   data->rawTempC2,   1);
    sendLine("RAW_TEMP_C5",   data->rawTempC5,   1);
    sendLine("RAW_TEMP_C6",   data->rawTempC6,   1);
    // Flow pulse count — shows actual pulses this interval so you can verify
    // the sensor is quiet at idle and responding correctly during real flow.
    Serial1.print(F("S,RAW_FLOW_PULSES,"));
    Serial1.println(data->rawFlowPulses);

    // ── Aggregated system state ─────────────────────────────────────
    //    Format: "S,STATE,<ff_state>,<filter_mode>,<backwash_state>\n"
    //    Values are integer enum ordinals (0, 1, 2...)
    Serial1.print(F("S,STATE,"));
    Serial1.print((int)ffState);
    Serial1.print(',');
    Serial1.print((int)filterMode);
    Serial1.print(',');
    Serial1.println((int)bwState);

    // SUGGESTION: Append a message sequence number so the ESP32 can
    //             detect missed frames:
    //   static uint32_t seqNum = 0;
    //   Serial1.print(F("S,SEQ,")); Serial1.println(seqNum++);
}

// ── Map container string ("C2","C5","C6") to sensor array index ─────────
//    pH / turbidity / temp arrays:  C2→0, C5→1, C6→2
//    Level array:                   C2→0, C3→1, C4→2, C5→3, C6→4
static int8_t containerToQualIdx(const char* s)
{
    if (strncmp(s, "C2", 2) == 0) return 0;
    if (strncmp(s, "C5", 2) == 0) return 1;
    if (strncmp(s, "C6", 2) == 0) return 2;
    return -1;
}

static int8_t containerToLevelIdx(const char* s)
{
    if (strncmp(s, "C2", 2) == 0) return 0;
    if (strncmp(s, "C3", 2) == 0) return 1;
    if (strncmp(s, "C4", 2) == 0) return 2;
    if (strncmp(s, "C5", 2) == 0) return 3;
    if (strncmp(s, "C6", 2) == 0) return 4;
    return -1;
}

// Map quality sensor index to its pH / turbidity analog pin
static uint8_t phPins[CAL_QTY_SENSORS]   = { PH_C2_PIN,   PH_C5_PIN,   PH_C6_PIN   };
static uint8_t turbPins[CAL_QTY_SENSORS] = { TURB_C2_PIN, TURB_C5_PIN, TURB_C6_PIN };

// ── Send ACK frame back to ESP32 ─────────────────────────────────────────
static void sendAck(const char* fields)
{
    Serial1.print(F("A,"));
    Serial1.println(fields);
    // Also echo to USB Serial for debugging
    Serial.print(F("[ACK] A,"));
    Serial.println(fields);
}

// ── Actuator lookup tables ────────────────────────────────────────────────
// Maps label strings ("V1"–"V8", "P1"–"P4") to their pin numbers.
// Add or reorder entries here if pin assignments ever change — no other
// code needs to change.

struct ActuatorEntry {
    const char* label;
    uint8_t     pin;
};

static const ActuatorEntry VALVE_MAP[] = {
    { "V1", VALVE1_PIN },
    { "V2", VALVE2_PIN },
    { "V3", VALVE3_PIN },
    { "V4", VALVE4_PIN },
    { "V5", VALVE5_PIN },
    { "V6", VALVE6_PIN },
    { "V7", VALVE7_PIN },
    { "V8", VALVE8_PIN },
};
static const uint8_t VALVE_COUNT = sizeof(VALVE_MAP) / sizeof(VALVE_MAP[0]);

static const ActuatorEntry PUMP_MAP[] = {
    { "P1", PUMP1_PIN },
    { "P2", PUMP2_PIN },
    { "P3", PUMP3_PIN },
    { "P4", PUMP4_PIN },
};
static const uint8_t PUMP_COUNT = sizeof(PUMP_MAP) / sizeof(PUMP_MAP[0]);

// Returns pin for a given label, or 255 if not found.
static uint8_t lookupActuator(const ActuatorEntry* table, uint8_t count, const char* label)
{
    for (uint8_t i = 0; i < count; i++) {
        if (strncmp(table[i].label, label, 2) == 0) return table[i].pin;
    }
    return 255;  // not found
}

// ── Parse and execute a received command ────────────────────────────────
static void processCommand(const char* cmd)
{
    // Commands must start with "C,"
    if (cmd[0] != 'C' || cmd[1] != ',') return;

    const char* payload = cmd + 2;  // skip "C,"

    // ── Filter mode ─────────────────────────────────────────────────
    if (strncmp(payload, "FILTER,CHARCOAL", 15) == 0) {
        pipeline_setFilterMode(FILTER_CHARCOAL_ONLY);
        Serial.println(F("[Comms] CMD: Filter -> CHARCOAL only"));
    }
    else if (strncmp(payload, "FILTER,BOTH", 11) == 0) {
        pipeline_setFilterMode(FILTER_CHARCOAL_AND_RO);
        Serial.println(F("[Comms] CMD: Filter -> CHARCOAL + RO"));
    }
    // ── Backwash ────────────────────────────────────────────────────
    else if (strncmp(payload, "BACKWASH,START", 14) == 0) {
        pipeline_startBackwash();
        Serial.println(F("[Comms] CMD: Backwash START"));
    }
    else if (strncmp(payload, "BACKWASH,STOP", 13) == 0) {
        pipeline_stopBackwash();
        Serial.println(F("[Comms] CMD: Backwash STOP"));
    }
    // ── Emergency stop ──────────────────────────────────────────────
    else if (strncmp(payload, "ESTOP,ON", 8) == 0) {
        // ACK first — pipeline_emergencyStop() triggers multiple logEvent()
        // calls (one per actuator closed) which fill the TX buffer before the
        // ACK can be sent if we don't send it here first.
        sendAck("ESTOP,OK");
        pipeline_emergencyStop();
        firstFlush_reset();
        Serial.println(F("[Comms] CMD: EMERGENCY STOP"));
        logEvent(LOG_WARN, LOG_CAT_SYSTEM, F("E-STOP activated — all actuators off"));
    }
    else if (strncmp(payload, "ESTOP,OFF", 9) == 0) {
        sendAck("ESTOP,OK");
        pipeline_init();
        firstFlush_init();
        Serial.println(F("[Comms] CMD: Emergency cleared — system reset"));
        logEvent(LOG_INFO, LOG_CAT_SYSTEM, F("E-STOP cleared — system ready"));
    }
    // ── Manual valve control ─────────────────────────────────────────
    //    Format: C,VALVE,Vn,ON|OFF   e.g. C,VALVE,V3,ON
    //    ACK:    A,VALVE,Vn,OK       or   A,VALVE,Vn,ERR
    else if (strncmp(payload, "VALVE,", 6) == 0)
    {
        char buf[16];
        strncpy(buf, payload + 6, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char* label  = strtok(buf, ",");   // e.g. "V3"
        char* action = strtok(nullptr, ","); // "ON" or "OFF"

        uint8_t pin = lookupActuator(VALVE_MAP, VALVE_COUNT, label ? label : "");

        char ack[20];
        if (pin == 255 || !action) {
            snprintf(ack, sizeof(ack), "VALVE,%s,ERR", label ? label : "?");
            Serial.print(F("[Comms] Unknown valve: "));
            Serial.println(label ? label : "?");
            sendAck(ack);
            logEvent(LOG_ERROR, LOG_CAT_ACTUATOR,
                     String(label ? label : "?") + " unknown — command rejected");
        } else {
            bool opening = (strncmp(action, "ON", 2) == 0);
            if (opening) {
                valve_open(pin);
            } else {
                valve_close(pin);
            }
            snprintf(ack, sizeof(ack), "VALVE,%s,OK", label);
            Serial.print(F("[Comms] Valve "));
            Serial.print(label);
            Serial.print(' ');
            Serial.println(action);
            // ACK is sent BEFORE logEvent so it reaches the ESP32 drain window
            // ahead of the log frames — log frames in the TX buffer ahead of the
            // ACK were the root cause of the "No ACK" timeout warnings.
            sendAck(ack);
            logEvent(LOG_INFO, LOG_CAT_ACTUATOR,
                     String(label) + (opening ? " opened" : " closed"));
        }
    }
    // ── Manual pump control ──────────────────────────────────────────
    //    Format: C,PUMP,Pn,ON|OFF   e.g. C,PUMP,P2,ON
    //    ACK:    A,PUMP,Pn,OK       or   A,PUMP,Pn,ERR
    //    A 100 ms delay is inserted before pump_start() to allow any
    //    upstream valve that was just opened to reach full flow area
    //    before the pump draws water. No delay on stop.
    else if (strncmp(payload, "PUMP,", 5) == 0)
    {
        char buf[16];
        strncpy(buf, payload + 5, sizeof(buf) - 1);
        buf[sizeof(buf) - 1] = '\0';

        char* label  = strtok(buf, ",");
        char* action = strtok(nullptr, ",");

        uint8_t pin = lookupActuator(PUMP_MAP, PUMP_COUNT, label ? label : "");

        char ack[20];
        if (pin == 255 || !action) {
            snprintf(ack, sizeof(ack), "PUMP,%s,ERR", label ? label : "?");
            Serial.print(F("[Comms] Unknown pump: "));
            Serial.println(label ? label : "?");
            sendAck(ack);
            logEvent(LOG_ERROR, LOG_CAT_PUMP,
                     String(label ? label : "?") + " command failed");
        } else {
            bool starting = (strncmp(action, "ON", 2) == 0);
            if (starting) {
                delay(100);   // let valve pressure equalise before starting pump
                pump_start(pin);
            } else {
                pump_stop(pin);
            }
            snprintf(ack, sizeof(ack), "PUMP,%s,OK", label);
            Serial.print(F("[Comms] Pump "));
            Serial.print(label);
            Serial.print(' ');
            Serial.println(action);
            // ACK before logEvent — same reason as valve handler above
            sendAck(ack);
            logEvent(LOG_INFO, LOG_CAT_PUMP,
                     String(label) + (starting ? " started" : " stopped"));
        }
    }
    // ── Calibration commands ────────────────────────────────────────
    //    Format: C,CAL_<TYPE>,<CONTAINER>[,<PARAM>[,<VALUE>]]
    //
    //    pH:         C,CAL_PH,C2,MID       — capture neutral (pH 7.00) point
    //                C,CAL_PH,C2,LOW       — capture acid (pH 4.01) point
    //    Turbidity:  C,CAL_TURB,C2,ZERO    — capture 0 NTU zero point
    //                C,CAL_TURB,C2,SPAN,50 — capture span point at known NTU
    //    Level:      C,CAL_LVL,C2,EMPTY    — capture empty distance
    //                C,CAL_LVL,C2,FULL     — capture full distance
    //    Temp:       C,CAL_TEMP,C2,OFFSET,-0.5  — set offset directly
    //    Flow:       C,CAL_FLOW,PPL,450    — set pulses per litre
    //    Reset:      C,CAL_RESET,ALL       — restore factory defaults
    else if (strncmp(payload, "CAL_PH,", 7) == 0)
    {
        // Tokenise: CAL_PH , <container> , <point>
        char buf[32];
        strncpy(buf, payload + 7, sizeof(buf) - 1);
        char* container = strtok(buf, ",");
        char* point     = strtok(nullptr, ",");
        if (!container || !point) return;

        int8_t idx = containerToQualIdx(container);
        if (idx < 0) { sendAck("CAL_ERROR,PH,BAD_CONTAINER"); return; }

        float mV = analogRead(phPins[idx]) / 1024.0f * 5000.0f;

        if (strncmp(point, "MID", 3) == 0) {
            calData.ph[idx].neutralV = mV;
            cal_save();
            char ack[48];
            snprintf(ack, sizeof(ack), "CAL_PH,%s,MID,OK,%.2f", container, mV);
            sendAck(ack);
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION,
                     String("CAL_PH,") + container + ",MID accepted");
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION, F("pH saved to EEPROM"));
        }
        else if (strncmp(point, "LOW", 3) == 0) {
            calData.ph[idx].acidV = mV;
            cal_save();
            char ack[48];
            snprintf(ack, sizeof(ack), "CAL_PH,%s,LOW,OK,%.2f", container, mV);
            sendAck(ack);
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION,
                     String("CAL_PH,") + container + ",LOW accepted");
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION, F("pH saved to EEPROM"));
        }
        else {
            sendAck("CAL_ERROR,PH,BAD_POINT");
        }
    }
    else if (strncmp(payload, "CAL_TURB,", 9) == 0)
    {
        char buf[40];
        strncpy(buf, payload + 9, sizeof(buf) - 1);
        char* container = strtok(buf, ",");
        char* point     = strtok(nullptr, ",");
        char* ntuStr    = strtok(nullptr, ",");   // only present for SPAN
        if (!container || !point) return;

        int8_t idx = containerToQualIdx(container);
        if (idx < 0) { sendAck("CAL_ERROR,TURB,BAD_CONTAINER"); return; }

        float volt = sensors_readTurbVoltage(turbPins[idx]);

        if (strncmp(point, "ZERO", 4) == 0) {
            calData.turb[idx].zeroV = volt;
            cal_save();
            char ack[48];
            snprintf(ack, sizeof(ack), "CAL_TURB,%s,ZERO,OK,%.3f", container, volt);
            sendAck(ack);
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION,
                     String("CAL_TURB,") + container + ",ZERO accepted");
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION, F("Turbidity saved to EEPROM"));
        }
        else if (strncmp(point, "SPAN", 4) == 0 && ntuStr) {
            float ntuRef = atof(ntuStr);
            float span   = calData.turb[idx].zeroV - volt;
            if (span < 0.01f) {
                sendAck("CAL_ERROR,TURB,BAD_VOLTAGE");
            } else {
                calData.turb[idx].slopeNTUperV = ntuRef / span;
                cal_save();
                char ack[56];
                snprintf(ack, sizeof(ack), "CAL_TURB,%s,SPAN,OK,%.3f", container, volt);
                sendAck(ack);
                logEvent(LOG_INFO, LOG_CAT_CALIBRATION,
                         String("CAL_TURB,") + container + ",SPAN accepted");
                logEvent(LOG_INFO, LOG_CAT_CALIBRATION, F("Turbidity saved to EEPROM"));
            }
        }
        else {
            sendAck("CAL_ERROR,TURB,BAD_POINT");
        }
    }
    else if (strncmp(payload, "CAL_LVL,", 8) == 0)
    {
        char buf[32];
        strncpy(buf, payload + 8, sizeof(buf) - 1);
        char* container = strtok(buf, ",");
        char* point     = strtok(nullptr, ",");
        if (!container || !point) return;

        int8_t idx = containerToLevelIdx(container);
        if (idx < 0) { sendAck("CAL_ERROR,LVL,BAD_CONTAINER"); return; }

        // Read the appropriate ultrasonic sensor for this container.
        // We re-read directly here rather than using the last cached value
        // so the calibration captures the current actual distance.
        const uint8_t trigPins[5] = { US_C2_TRIG, US_C3_TRIG, US_C4_TRIG, US_C5_TRIG, US_C6_TRIG };
        const uint8_t echoPins[5] = { US_C2_ECHO, US_C3_ECHO, US_C4_ECHO, US_C5_ECHO, US_C6_ECHO };

        // Simple single-shot read (not median) for calibration capture.
        // Timeout: 25 000 µs ≈ 4.3 m max range — ensures the call returns
        // well within the ESP32's 200 ms ACK drain window.  Without a timeout
        // pulseIn() blocks up to 1 s, causing the ESP32 to log a false
        // "no ACK (200ms timeout)" warning even though the Mega did respond.
        digitalWrite(trigPins[idx], LOW);  delayMicroseconds(2);
        digitalWrite(trigPins[idx], HIGH); delayMicroseconds(10);
        digitalWrite(trigPins[idx], LOW);
        float distCm = pulseIn(echoPins[idx], HIGH, 25000UL) / 58.0f;

        if (strncmp(point, "EMPTY", 5) == 0) {
            calData.level[idx].emptyCm = distCm;
            cal_save();
            char ack[48];
            snprintf(ack, sizeof(ack), "CAL_LVL,%s,EMPTY,OK,%.1f", container, distCm);
            sendAck(ack);
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION,
                     String("CAL_LVL,") + container + ",EMPTY accepted");
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION, F("Level saved to EEPROM"));
        }
        else if (strncmp(point, "FULL", 4) == 0) {
            calData.level[idx].fullCm = distCm;
            cal_save();
            char ack[48];
            snprintf(ack, sizeof(ack), "CAL_LVL,%s,FULL,OK,%.1f", container, distCm);
            sendAck(ack);
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION,
                     String("CAL_LVL,") + container + ",FULL accepted");
            logEvent(LOG_INFO, LOG_CAT_CALIBRATION, F("Level saved to EEPROM"));
        }
        else {
            sendAck("CAL_ERROR,LVL,BAD_POINT");
        }
    }
    else if (strncmp(payload, "CAL_TEMP,", 9) == 0)
    {
        char buf[40];
        strncpy(buf, payload + 9, sizeof(buf) - 1);
        char* container = strtok(buf, ",");
        char* keyword   = strtok(nullptr, ",");   // "OFFSET"
        char* valStr    = strtok(nullptr, ",");
        if (!container || !keyword || !valStr) return;

        int8_t idx = containerToQualIdx(container);
        if (idx < 0) { sendAck("CAL_ERROR,TEMP,BAD_CONTAINER"); return; }

        calData.tempOffset[idx] = atof(valStr);
        cal_save();
        char ack[48];
        snprintf(ack, sizeof(ack), "CAL_TEMP,%s,OK,%.2f", container, calData.tempOffset[idx]);
        sendAck(ack);
        logEvent(LOG_INFO, LOG_CAT_CALIBRATION,
                 String("CAL_TEMP,") + container + ",OFFSET accepted");
        logEvent(LOG_INFO, LOG_CAT_CALIBRATION, F("Temperature saved to EEPROM"));
    }
    else if (strncmp(payload, "CAL_FLOW,PPL,", 13) == 0)
    {
        calData.flowPPL = atof(payload + 13);
        cal_save();
        char ack[32];
        snprintf(ack, sizeof(ack), "CAL_FLOW,OK,%.1f", calData.flowPPL);
        sendAck(ack);
        logEvent(LOG_INFO, LOG_CAT_CALIBRATION, F("CAL_FLOW,PPL accepted"));
        logEvent(LOG_INFO, LOG_CAT_CALIBRATION, F("Flow saved to EEPROM"));
    }
    else if (strncmp(payload, "CAL_RESET,ALL", 13) == 0)
    {
        cal_reset();
        sendAck("CAL_RESET,OK");
    }
    // ── Unknown ─────────────────────────────────────────────────────
    else {
        Serial.print(F("[Comms] Unknown command: "));
        Serial.println(cmd);
    }

    // SUGGESTION: Echo an ACK back to the ESP32 so the app knows the
    //             command was received and executed:
    //   Serial1.print(F("A,")); Serial1.print(payload); Serial1.println(F(",OK"));
}

// ═════════════════════════════════════════════════════════════════════════
bool comms_receiveCommands()
{
    bool received = false;

    while (Serial1.available()) {
        char c = Serial1.read();

        if (c == '\n' || c == '\r') {
            if (rxIdx > 0) {
                rxBuf[rxIdx] = '\0';   // null-terminate
                processCommand(rxBuf);
                rxIdx    = 0;
                received = true;
            }
        } else {
            if (rxIdx < RX_BUF_SIZE - 1) {
                rxBuf[rxIdx++] = c;
            } else {
                // Buffer overflow — discard malformed line
                rxIdx = 0;
                Serial.println(F("[Comms] RX buffer overflow — line discarded"));
            }
        }
    }

    return received;
}
