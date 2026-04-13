#include "sensors.h"
#include "pins.h"
#include "config.h"
#include "calibration.h"

// ── Library includes (confined to this file) ────────────────────────────
#include <Ultrasonic.h>
#include <FlowSensor.h>
#include <FlowSensor_Type.h>
#include <OneWire.h>
#include <DallasTemperature.h>

/*
 *  IMPLEMENTATION NOTES
 *
 *  All hardware objects are file-static so no other translation unit
 *  depends on the concrete sensor libraries.  This makes it trivial to
 *  swap a library (e.g., NewPing instead of Ultrasonic) without touching
 *  the rest of the firmware.
 *
 *  NOTE — DFRobot_PH library removed:
 *  The DFRobot_PH library stored calibration at a hardcoded EEPROM address
 *  (0x00), which caused all three pH sensor instances to share and corrupt
 *  each other's calibration data.
 *
 *  pH readings are now handled entirely by our calibration module
 *  (calibration.h / cal_applyPH) using a two-point linear formula with
 *  per-sensor values stored at known EEPROM addresses. The DFRobot library
 *  is no longer needed or used.
 */

// ── Flow Sensor ─────────────────────────────────────────────────────────
static FlowSensor flowSensor(YFS201, FLOW_SENSOR_PIN);
static float      lastFlowRate = 0.0f;

// The ISR must be a plain free function (no captures / member pointers).
void sensors_flowISR()
{
    flowSensor.count();
}

// ── Ultrasonic Sensors ──────────────────────────────────────────────────
static Ultrasonic usC2(US_C2_TRIG, US_C2_ECHO);
static Ultrasonic usC3(US_C3_TRIG, US_C3_ECHO);
static Ultrasonic usC4(US_C4_TRIG, US_C4_ECHO);
static Ultrasonic usC5(US_C5_TRIG, US_C5_ECHO);
static Ultrasonic usC6(US_C6_TRIG, US_C6_ECHO);

// ── Temperature Sensors (DS18B20 on separate OneWire buses) ─────────────
static OneWire            owC2(TEMP_C2_PIN);
static OneWire            owC5(TEMP_C5_PIN);
static OneWire            owC6(TEMP_C6_PIN);
static DallasTemperature  tempC2(&owC2);
static DallasTemperature  tempC5(&owC5);
static DallasTemperature  tempC6(&owC6);

// ── pH — raw voltage accumulators ───────────────────────────────────────
//    We read raw ADC voltage and pass it to cal_applyPH().
//    No DFRobot_PH library needed — our calibration module handles
//    the two-point linear formula with per-sensor EEPROM storage.
static float lastVoltageC2 = 0.0f;
static float lastVoltageC5 = 0.0f;
static float lastVoltageC6 = 0.0f;

// ═════════════════════════════════════════════════════════════════════════
//  Helpers
// ═════════════════════════════════════════════════════════════════════════

/**
 * Take a single ultrasonic reading and return the distance in cm.
 *
 * The previous median-of-3 approach (3 reads + 2×delay(10)) blocked
 * ~110ms per sensor — 550ms for all five — which caused the Mega's
 * command-receive loop to miss the ESP32's ACK drain window.
 *
 * A single read per 1-second interval is sufficient: the HC-SR04 echo
 * has fully dissipated long before the next trigger, so no inter-read
 * settling delay is needed.  Each call now blocks only ~5ms (typical
 * echo for a 50–100cm water surface) instead of ~110ms.
 *
 * SUGGESTION: If you see occasional wild readings due to surface ripples,
 * implement a ring-buffer median across the last 3 *intervals* instead of
 * 3 rapid reads within one interval — this keeps each call fast while
 * still rejecting outliers.
 */
static float readUltrasonic(Ultrasonic& sensor)
{
    return (float)sensor.read();
}

/**
 * Read turbidity sensor and return NTU via calibration module.
 * Raw voltage is passed to cal_applyTurb() which uses the stored
 * zero-point and slope set during calibration.
 *
 * @param analogPin  Pin to read
 * @param calIdx     Calibration index: 0=C2, 1=C5, 2=C6
 */
static float readTurbidityNTU(uint8_t analogPin, uint8_t calIdx)
{
    float volt = analogRead(analogPin) * (5.0f / 1024.0f);
    return cal_applyTurb(calIdx, volt);
}

/**
 * Read a raw turbidity voltage — used by comms.cpp during SPAN calibration
 * to capture the current voltage without applying calibration.
 */
float sensors_readTurbVoltage(uint8_t analogPin)
{
    return analogRead(analogPin) * (5.0f / 1024.0f);
}

// ═════════════════════════════════════════════════════════════════════════
//  Public API
// ═════════════════════════════════════════════════════════════════════════

void sensors_init()
{
    // Flow sensor — enable internal pull-up on the signal pin.
    // The YF-S201 has an open-collector output and needs the line held HIGH
    // between pulses. Without this (or an external 10kΩ pull-up to 5V),
    // the floating pin picks up EMI noise and generates phantom pulse counts.
    // The internal pull-up (~50kΩ) is a software fallback — an external 10kΩ
    // resistor from pin 2 to ANY 5V source (common ground required) is preferred
    // for long wire runs (> 1m) or noisy environments near pumps/relays.
    pinMode(FLOW_SENSOR_PIN, INPUT_PULLUP);

    // Attach interrupt on the rising edge of each pulse.
    flowSensor.begin(sensors_flowISR);

    // Temperature buses — disable blocking wait so requestTemperatures() returns
    // immediately (conversion happens in the sensor's internal circuit over ~750ms).
    // We fire a request at the END of each sensors_readAll() and read the result
    // at the START of the next call — by then SENSOR_READ_INTERVAL_MS (1 s) has
    // elapsed, well past the 750ms conversion window.
    tempC2.begin(); tempC2.setWaitForConversion(false);
    tempC5.begin(); tempC5.setWaitForConversion(false);
    tempC6.begin(); tempC6.setWaitForConversion(false);

    // Fire the first conversion so the very first sensors_readAll() call (1+ s
    // after setup) returns valid temperature values rather than DEVICE_DISCONNECTED.
    tempC2.requestTemperatures();
    tempC5.requestTemperatures();
    tempC6.requestTemperatures();

    // SUGGESTION: Read each ultrasonic sensor once here and print the
    //             result to Serial.  If a sensor returns 0 or > 400 cm,
    //             print a "SENSOR FAULT" warning so you catch wiring
    //             problems during setup rather than during a rain event.
}

// ─────────────────────────────────────────────────────────────────────────
void sensors_readAll(SensorData* data)
{
    // ── Temperature — read results of the PREVIOUS non-blocking request ─
    // requestTemperatures() was called at the end of the last cycle (or in
    // sensors_init() for the very first call).  The 750ms conversion is
    // complete well within the 1000ms SENSOR_READ_INTERVAL_MS, so these
    // values are always fresh.  Reading first removes 2250ms of blocking
    // that previously locked out the Mega's command-receive loop.
    { float t = tempC2.getTempCByIndex(0); data->rawTempC2 = t; data->tempC2 = cal_applyTemp(0, t); }
    { float t = tempC5.getTempCByIndex(0); data->rawTempC5 = t; data->tempC5 = cal_applyTemp(1, t); }
    { float t = tempC6.getTempCByIndex(0); data->rawTempC6 = t; data->tempC6 = cal_applyTemp(2, t); }

    // ── Flow rate ───────────────────────────────────────────────────
    flowSensor.read();
    lastFlowRate   = flowSensor.getFlowRate_m();
    data->flowRate = lastFlowRate;

    // ── Ultrasonic levels — capture raw cm, then calibrate to % ────
    //    Raw distances are stored for the calibration dashboard so the
    //    operator can see exactly what the sensor reads when setting the
    //    EMPTY / FULL reference distances.
    { float d = readUltrasonic(usC2); data->rawDistC2 = d; data->levelC2 = cal_applyLevel(0, d); }
    { float d = readUltrasonic(usC3); data->rawDistC3 = d; data->levelC3 = cal_applyLevel(1, d); }
    { float d = readUltrasonic(usC4); data->rawDistC4 = d; data->levelC4 = cal_applyLevel(2, d); }
    { float d = readUltrasonic(usC5); data->rawDistC5 = d; data->levelC5 = cal_applyLevel(3, d); }
    { float d = readUltrasonic(usC6); data->rawDistC6 = d; data->levelC6 = cal_applyLevel(4, d); }

    // ── pH & turbidity — capture raw voltages, then calibrate ───────
    { float v = sensors_readTurbVoltage(TURB_C2_PIN); data->rawTurbVC2 = v; data->turbidityC2 = cal_applyTurb(0, v); }
    lastVoltageC2  = analogRead(PH_C2_PIN) / 1024.0f * 5000.0f;
    data->rawMvC2  = lastVoltageC2;
    data->phC2     = cal_applyPH(0, lastVoltageC2);

    { float v = sensors_readTurbVoltage(TURB_C5_PIN); data->rawTurbVC5 = v; data->turbidityC5 = cal_applyTurb(1, v); }
    lastVoltageC5  = analogRead(PH_C5_PIN) / 1024.0f * 5000.0f;
    data->rawMvC5  = lastVoltageC5;
    data->phC5     = cal_applyPH(1, lastVoltageC5);

    { float v = sensors_readTurbVoltage(TURB_C6_PIN); data->rawTurbVC6 = v; data->turbidityC6 = cal_applyTurb(2, v); }
    lastVoltageC6  = analogRead(PH_C6_PIN) / 1024.0f * 5000.0f;
    data->rawMvC6  = lastVoltageC6;
    data->phC6     = cal_applyPH(2, lastVoltageC6);

    // ── Fire next temperature conversion (non-blocking, ~1ms) ───────
    // Results will be ready in 750ms — well before the next call at
    // SENSOR_READ_INTERVAL_MS (1000ms).
    tempC2.requestTemperatures();
    tempC5.requestTemperatures();
    tempC6.requestTemperatures();

    // SUGGESTION: Add sanity checks here.  For example:
    //   if (data->levelC2 == 0 || data->levelC2 > 400)
    //       Serial.println(F("[WARN] C2 ultrasonic fault"));
}

// ─────────────────────────────────────────────────────────────────────────
bool sensors_isFlowActive()
{
    return (lastFlowRate >= FLOW_MIN_THRESHOLD);
}
