#ifndef SENSORS_H
#define SENSORS_H

#include <Arduino.h>

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  SENSORS MODULE — All Sensor Initialisation and Reading
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  Encapsulates every sensor in the system:
 *    1× YF-S201  water flow sensor       (pulse / interrupt-based)
 *    5× HC-SR04  ultrasonic distance     (containers 2–6 water levels)
 *    3× DS18B20  temperature             (containers 2, 5 & 6)
 *    3× DFRobot  analog pH               (containers 2, 5 & 6)
 *    3× analog   turbidity               (containers 2, 5 & 6)
 *
 *  All library-specific includes (#include <Ultrasonic.h>, etc.) are
 *  confined to sensors.cpp.  The rest of the codebase only sees the clean
 *  SensorData struct and the public API below.
 *
 *  SUGGESTIONS:
 *  ● Add a moving-average / ring-buffer filter on analog readings (pH,
 *    turbidity).  A simple buffer of 10 samples smooths ADC noise nicely
 *    and costs very little RAM.
 *  ● For ultrasonic readings, a median-of-3 filter rejects the occasional
 *    wild echo caused by a choppy water surface.  (Implemented in .cpp.)
 *  ● Monitor for sensor faults: if an ultrasonic sensor returns 0 cm or
 *    max range (> 400 cm) repeatedly, flag it as faulty and alert via the
 *    ESP32.  Operating blind on a failed sensor is dangerous.
 *  ● For pH accuracy, recalibrate monthly with standard buffer solutions
 *    (pH 4.0 and pH 7.0).  The DFRobot library supports interactive
 *    calibration over Serial.
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * Consolidated sensor data shared across the entire system.
 * A single instance is populated by sensors_readAll() and passed
 * to every other module that needs sensor values.
 */
struct SensorData {
    // ── Flow ────────────────────────────────────────────────────────
    float         flowRate;       // L/min — instantaneous (thresholded; 0 when idle)
    unsigned long rawFlowPulses;  // raw pulse count accumulated this interval

    // ── Ultrasonic water levels (cm from sensor face to water surface)
    //    LOWER value = HIGHER water level
    float levelC2;            // Container 2  — calibrated fill %
    float levelC3;            // Container 3  — calibrated fill %
    float levelC4;            // Container 4  — calibrated fill %
    float levelC5;            // Container 5  — calibrated fill %
    float levelC6;            // Container 6  — calibrated fill %

    // ── Raw ultrasonic distances (cm) — before cal_applyLevel() ────
    //    Exposed for the calibration dashboard so the operator can see
    //    the exact sensor reading when setting EMPTY / FULL references.
    float rawDistC2;          // Container 2  — raw distance (cm)
    float rawDistC3;          // Container 3  — raw distance (cm)
    float rawDistC4;          // Container 4  — raw distance (cm)
    float rawDistC5;          // Container 5  — raw distance (cm)
    float rawDistC6;          // Container 6  — raw distance (cm)

    // ── Container 2 water quality (raw rainwater, post-first-flush) ───
    float tempC2;             // Temperature (°C) — offset-corrected
    float phC2;               // pH (0–14 scale)  — calibrated
    float turbidityC2;        // Turbidity (NTU)  — calibrated

    // ── Container 5 water quality ───────────────────────────────────
    float tempC5;             // Temperature (°C) — offset-corrected
    float phC5;               // pH (0–14 scale)  — calibrated
    float turbidityC5;        // Turbidity (NTU)  — calibrated

    // ── Container 6 water quality (monitoring / read-only) ──────────
    float tempC6;             // Temperature (°C) — offset-corrected
    float phC6;               // pH (0–14 scale)  — calibrated
    float turbidityC6;        // Turbidity (NTU)  — calibrated

    // ── Raw quality sensor values — before calibration is applied ───
    //    pH:       ADC voltage in mV (neutral ≈ 2530 mV, acid ≈ 2030 mV)
    //    Turbidity: ADC voltage in V  (clear ≈ 4.2 V, murky lower)
    //    Temp:     DS18B20 °C before per-sensor offset correction
    float rawMvC2;    float rawMvC5;    float rawMvC6;    // pH probe mV
    float rawTurbVC2; float rawTurbVC5; float rawTurbVC6; // turbidity V
    float rawTempC2;  float rawTempC5;  float rawTempC6;  // temp °C (no offset)
};

/**
 * Initialise all sensor peripherals (flow ISR, OneWire buses, pH boards).
 * Call once in setup() after actuators_init().
 */
void sensors_init();

/**
 * Read EVERY sensor and fill the provided SensorData struct.
 * This is the main polling function — call it at SENSOR_READ_INTERVAL_MS.
 *
 * @param data  Pointer to a SensorData struct to populate
 */
void sensors_readAll(SensorData* data);

/**
 * Quick check: is there active water flow right now?
 * Based on the most recent reading from sensors_readAll().
 *
 * @return true if flow rate ≥ FLOW_MIN_THRESHOLD
 */
bool sensors_isFlowActive();

/**
 * ISR callback for the flow sensor.
 * Registered internally by sensors_init() — do NOT call directly.
 */
void sensors_flowISR();

/**
 * Read raw turbidity voltage (V) without applying calibration.
 * Used by the calibration command handler to capture span/zero points.
 *
 * @param analogPin  The analog pin to read (TURB_C2_PIN, etc.)
 */
float sensors_readTurbVoltage(uint8_t analogPin);

#endif // SENSORS_H
