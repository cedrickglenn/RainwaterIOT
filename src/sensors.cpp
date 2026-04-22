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
static FlowSensor     flowSensor(YFS201, FLOW_SENSOR_PIN);
static float          lastFlowRate      = 0.0f;
static unsigned long  lastTotalPulses   = 0UL;  // for computing per-interval delta

// Flow threshold — settable at runtime via sensors_setFlowThreshold().
// Defaults to the compile-time constant so existing behaviour is unchanged.
static float          flowThreshold     = FLOW_MIN_THRESHOLD;

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

// Last accepted distance per sensor (cm). -1.0 = no valid reading yet.
// Used by the outlier-rejection filter to discard physically impossible pings.
static float usLastGood[5] = { -1.0f, -1.0f, -1.0f, -1.0f, -1.0f };

// Maximum plausible level change between 1-second intervals.
// Tanks fill/drain slowly — 30 cm/s would require an extraordinary flow rate.
static const float US_MAX_DELTA_CM = 30.0f;

// 3-slot ring buffer for median smoothing across successive 1-second intervals.
// Zero blocking time — one read per call, median computed from the last 3 results.
static float   usRingBuf[5][3]  = {};
static uint8_t usRingIdx[5]     = {};
static uint8_t usRingCount[5]   = {};  // saturates at 3 once buffer is full

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
 * Take a single ultrasonic reading and return the distance in cm,
 * with outlier rejection to protect pipeline control logic.
 *
 * A reading is rejected (last good value returned instead) if:
 *   - It is 0 cm      — missed echo / HC-SR04 blind zone
 *   - It is > 400 cm  — beyond sensor range, no echo returned
 *   - It jumps > US_MAX_DELTA_CM from the last accepted value —
 *     physically impossible for a slow-filling/draining tank
 *
 * No blocking delay is added — each call still takes ~5ms.
 * The last-good store is per-sensor via the idx parameter.
 */
static float readUltrasonic(Ultrasonic& sensor, uint8_t idx)
{
    float raw = (float)sensor.read();

    // Hard validity bounds — HC-SR04 reliable range is ~2–400 cm
    if (raw <= 0.0f || raw > 400.0f) {
        Serial.print(F("[US] Sensor "));
        Serial.print(idx);
        Serial.print(F(" rejected (out of range): "));
        Serial.println(raw);
        return (usLastGood[idx] >= 0.0f) ? usLastGood[idx] : raw;
    }

    // Delta check — reject implausible jumps vs last accepted reading
    if (usLastGood[idx] >= 0.0f) {
        float delta = fabsf(raw - usLastGood[idx]);
        if (delta > US_MAX_DELTA_CM) {
            Serial.print(F("[US] Sensor "));
            Serial.print(idx);
            Serial.print(F(" rejected (delta "));
            Serial.print(delta, 1);
            Serial.println(F(" cm)"));
            return usLastGood[idx];
        }
    }

    usLastGood[idx] = raw;

    // Store accepted reading into the ring buffer and return the median.
    // On the first two calls the buffer isn't full yet — return raw directly
    // so pipeline logic has a valid value from the very first interval.
    usRingBuf[idx][usRingIdx[idx]] = raw;
    usRingIdx[idx] = (usRingIdx[idx] + 1) % 3;
    if (usRingCount[idx] < 3) {
        usRingCount[idx]++;
        return raw;
    }

    // Median of 3 via two-swap sort (no library needed, runs in ~6 comparisons).
    float a = usRingBuf[idx][0], b = usRingBuf[idx][1], c = usRingBuf[idx][2];
    if (a > b) { float t = a; a = b; b = t; }
    if (b > c) { float t = b; b = c; c = t; }
    if (a > b) { float t = a; a = b; b = t; }
    (void)a; (void)c;  // only the middle value is used
    return b;
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
    // Flow sensor — external 4.7kΩ pull-up resistor is fitted on the signal
    // line, so pass pullup=true to begin().  The library's default (pullup=false)
    // would enable INPUT_PULLUP, layering a ~50kΩ internal resistor in parallel
    // with the external one — harmless but unnecessary.
    // No separate pinMode() call needed; begin() configures the pin itself.
    flowSensor.begin(sensors_flowISR, true);  // true = external pull-up fitted

    // Temperature buses — disable blocking wait so requestTemperatures() returns
    // immediately (conversion happens in the sensor's internal circuit over ~750ms).
    // We fire a request at the END of each sensors_readAll() and read the result
    // at the START of the next call — by then SENSOR_READ_INTERVAL_MS (1 s) has
    // elapsed, well past the 750ms conversion window.
    tempC2.begin(); tempC2.setWaitForConversion(false);
    tempC5.begin(); tempC5.setWaitForConversion(false);
    tempC6.begin(); tempC6.setWaitForConversion(false);
    Serial.print(F("[Sensors] Temp devices — C2:"));
    Serial.print(tempC2.getDeviceCount());
    Serial.print(F(" C5:"));
    Serial.print(tempC5.getDeviceCount());
    Serial.print(F(" C6:"));
    Serial.println(tempC6.getDeviceCount());

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
    data->tempDevicesC2 = tempC2.getDeviceCount();
    data->tempDevicesC5 = tempC5.getDeviceCount();
    data->tempDevicesC6 = tempC6.getDeviceCount();
    { float t = tempC2.getTempCByIndex(0); data->rawTempC2 = t; data->tempC2 = cal_applyTemp(0, t); }
    { float t = tempC5.getTempCByIndex(0); data->rawTempC5 = t; data->tempC5 = cal_applyTemp(1, t); }
    { float t = tempC6.getTempCByIndex(0); data->rawTempC6 = t; data->tempC6 = cal_applyTemp(2, t); }

    // ── Flow rate ───────────────────────────────────────────────────
    flowSensor.read();

    // Compute per-interval pulse delta from the cumulative total.
    // getPulse() returns _totalpulse (cumulative); read() adds the just-cleared
    // _pulse to it before zeroing _pulse — so the delta is valid post-read().
    {
        unsigned long total = flowSensor.getPulse();
        data->rawFlowPulses = total - lastTotalPulses;
        lastTotalPulses     = total;
    }

    // getFlowRate_m() can return inf on the first call (elapsed time ≈ 0 due
    // to _timebefore initialising to 0) or spurious values from noise pulses.
    // isfinite() rejects inf/nan; FLOW_MIN_THRESHOLD rejects the noise floor.
    {
        float raw      = flowSensor.getFlowRate_m();
        lastFlowRate   = (isfinite(raw) && raw >= flowThreshold) ? raw : 0.0f;
        data->flowRate = lastFlowRate;
    }

    // ── Ultrasonic levels — capture raw cm, then calibrate to % ────
    //    Raw distances are stored for the calibration dashboard so the
    //    operator can see exactly what the sensor reads when setting the
    //    EMPTY / FULL reference distances.
    { float d = readUltrasonic(usC2, 0); data->rawDistC2 = d; data->levelC2 = cal_applyLevel(0, d); }
    { float d = readUltrasonic(usC3, 1); data->rawDistC3 = d; data->levelC3 = cal_applyLevel(1, d); }
    { float d = readUltrasonic(usC4, 2); data->rawDistC4 = d; data->levelC4 = cal_applyLevel(2, d); }
    { float d = readUltrasonic(usC5, 3); data->rawDistC5 = d; data->levelC5 = cal_applyLevel(3, d); }
    { float d = readUltrasonic(usC6, 4); data->rawDistC6 = d; data->levelC6 = cal_applyLevel(4, d); }

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
    return (lastFlowRate >= flowThreshold);
}

// ─────────────────────────────────────────────────────────────────────────
void sensors_setFlowThreshold(float lpm)
{
    flowThreshold = lpm;
    Serial.print(F("[Sensors] Flow threshold set to "));
    Serial.print(lpm, 2);
    Serial.println(F(" L/min"));
}
