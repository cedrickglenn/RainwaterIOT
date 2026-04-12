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
 * Take three rapid ultrasonic readings and return the median.
 * Rejects the occasional wild echo caused by ripples, bubbles, or
 * acoustic reflections inside the container.
 *
 * SUGGESTION: If you still get noisy readings, try increasing to
 * median-of-5 or add a thin PVC cone ("acoustic funnel") below the
 * sensor to stabilise the beam path.
 */
static float readUltrasonicMedian(Ultrasonic& sensor)
{
    float a = (float)sensor.read();
    delay(10);
    float b = (float)sensor.read();
    delay(10);
    float c = (float)sensor.read();

    // Sort three values — the middle one is the median
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    return b;  // median
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

    // Temperature buses
    tempC2.begin();
    tempC5.begin();
    tempC6.begin();

    // SUGGESTION: Read each ultrasonic sensor once here and print the
    //             result to Serial.  If a sensor returns 0 or > 400 cm,
    //             print a "SENSOR FAULT" warning so you catch wiring
    //             problems during setup rather than during a rain event.
}

// ─────────────────────────────────────────────────────────────────────────
void sensors_readAll(SensorData* data)
{
    // ── Flow rate ───────────────────────────────────────────────────
    flowSensor.read();
    lastFlowRate   = flowSensor.getFlowRate_m();
    data->flowRate = lastFlowRate;

    // ── Ultrasonic levels → calibrated fill percentage (0–100%) ────
    //    cal_applyLevel() converts raw cm distance to % using stored
    //    empty/full distances set during level calibration.
    data->levelC2 = cal_applyLevel(0, readUltrasonicMedian(usC2));
    data->levelC3 = cal_applyLevel(1, readUltrasonicMedian(usC3));
    data->levelC4 = cal_applyLevel(2, readUltrasonicMedian(usC4));
    data->levelC5 = cal_applyLevel(3, readUltrasonicMedian(usC5));
    data->levelC6 = cal_applyLevel(4, readUltrasonicMedian(usC6));

    // ── Container 2 water quality (raw rainwater, post-first-flush) ────
    tempC2.requestTemperatures();
    data->tempC2      = cal_applyTemp(0, tempC2.getTempCByIndex(0));
    data->turbidityC2 = readTurbidityNTU(TURB_C2_PIN, 0);

    lastVoltageC2     = analogRead(PH_C2_PIN) / 1024.0f * 5000.0f;
    data->phC2        = cal_applyPH(0, lastVoltageC2);

    // ── Container 5 water quality ───────────────────────────────────
    tempC5.requestTemperatures();
    data->tempC5      = cal_applyTemp(1, tempC5.getTempCByIndex(0));
    data->turbidityC5 = readTurbidityNTU(TURB_C5_PIN, 1);

    lastVoltageC5     = analogRead(PH_C5_PIN) / 1024.0f * 5000.0f;
    data->phC5        = cal_applyPH(1, lastVoltageC5);

    // ── Container 6 water quality ───────────────────────────────────
    tempC6.requestTemperatures();
    data->tempC6      = cal_applyTemp(2, tempC6.getTempCByIndex(0));
    data->turbidityC6 = readTurbidityNTU(TURB_C6_PIN, 2);

    lastVoltageC6     = analogRead(PH_C6_PIN) / 1024.0f * 5000.0f;
    data->phC6        = cal_applyPH(2, lastVoltageC6);

    // SUGGESTION: Add sanity checks here.  For example:
    //   if (data->levelC2 == 0 || data->levelC2 > 400)
    //       Serial.println(F("[WARN] C2 ultrasonic fault"));
}

// ─────────────────────────────────────────────────────────────────────────
bool sensors_isFlowActive()
{
    return (lastFlowRate >= FLOW_MIN_THRESHOLD);
}
