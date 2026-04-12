#ifndef CALIBRATION_H
#define CALIBRATION_H

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  CALIBRATION MODULE
 *  Stores and applies per-sensor calibration data, persisted to EEPROM.
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  All calibration values live in a single CalibrationData struct that is
 *  loaded from EEPROM on boot and written back whenever a CAL_* command
 *  is received.  A magic number (0xCAFE) at address 0x0000 tells us whether
 *  the EEPROM has ever been written — if not, safe defaults are used.
 *
 *  EEPROM LAYOUT (106 bytes total, well within Mega's 4 KB):
 *
 *    Addr    Size  Contents
 *    0x0000    2   Magic number (0xCAFE)
 *    0x0002    8   pH C2:  neutralVoltage (float), acidVoltage (float)
 *    0x000A    8   pH C5:  neutralVoltage, acidVoltage
 *    0x0012    8   pH C6:  neutralVoltage, acidVoltage
 *    0x001A    8   Turbidity C2: zeroVoltage (float), slopeNTUperV (float)
 *    0x0022    8   Turbidity C5
 *    0x002A    8   Turbidity C6
 *    0x0032    8   Level C2: emptyDist_cm (float), fullDist_cm (float)
 *    0x003A    8   Level C3
 *    0x0042    8   Level C4
 *    0x004A    8   Level C5
 *    0x0052    8   Level C6
 *    0x005A    4   Temp offset C2 (float)
 *    0x005E    4   Temp offset C5
 *    0x0062    4   Temp offset C6
 *    0x0066    4   Flow pulsesPerLiter (float)
 *    ─────────────────────────────────────────
 *              106 bytes total
 *
 *  CONTAINER INDEX CONVENTION (used throughout this module):
 *    pH / Turbidity / Temp arrays:  0 = C2,  1 = C5,  2 = C6
 *    Level array:                   0 = C2,  1 = C3,  2 = C4,  3 = C5,  4 = C6
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

#include <Arduino.h>
#include <EEPROM.h>

// ── EEPROM magic ─────────────────────────────────────────────────────────
#define EEPROM_MAGIC        0xCAFE
#define EEPROM_MAGIC_ADDR   0x0000

// ── Sensor count constants ────────────────────────────────────────────────
#define CAL_QTY_SENSORS     3   // pH / turbidity / temp: C2, C5, C6
#define CAL_LVL_SENSORS     5   // level: C2, C3, C4, C5, C6

// ── Per-sensor calibration structs ───────────────────────────────────────

struct PhCalData {
    float neutralV;   // Voltage when probe is in pH 7.00 buffer
    float acidV;      // Voltage when probe is in pH 4.01 buffer
};

struct TurbCalData {
    float zeroV;        // Voltage in 0 NTU (distilled water)
    float slopeNTUperV; // NTU per volt — derived from span calibration
};

struct LevelCalData {
    float emptyCm;  // Ultrasonic reading (cm) when container is empty
    float fullCm;   // Ultrasonic reading (cm) when container is full
};

// ── Master calibration struct ─────────────────────────────────────────────
struct CalibrationData {
    PhCalData    ph[CAL_QTY_SENSORS];    // [0]=C2  [1]=C5  [2]=C6
    TurbCalData  turb[CAL_QTY_SENSORS];  // [0]=C2  [1]=C5  [2]=C6
    LevelCalData level[CAL_LVL_SENSORS]; // [0]=C2  [1]=C3  [2]=C4  [3]=C5  [4]=C6
    float        tempOffset[CAL_QTY_SENSORS]; // °C offset: [0]=C2 [1]=C5 [2]=C6
    float        flowPPL;                // Flow sensor pulses-per-litre
};

// ── Global calibration data (defined in calibration.cpp) ─────────────────
extern CalibrationData calData;

// ── Public API ────────────────────────────────────────────────────────────

/**
 * Load calibration from EEPROM on boot.
 * If EEPROM magic is missing, loads safe defaults and saves them.
 * Call in setup() BEFORE sensors_init().
 */
void cal_init();

/**
 * Write current calData to EEPROM.
 * Called automatically after every successful CAL_* command.
 */
void cal_save();

/**
 * Reset all calibration to firmware defaults and save to EEPROM.
 * Triggered by C,CAL_RESET,ALL command.
 */
void cal_reset();

/**
 * Apply pH calibration to a raw ADC voltage (mV).
 * Uses two-point linear interpolation between acid and neutral points.
 *
 * @param idx  Sensor index: 0=C2, 1=C5, 2=C6
 * @param mV   Raw voltage in millivolts (analogRead / 1024.0 * 5000.0)
 */
float cal_applyPH(uint8_t idx, float mV);

/**
 * Apply turbidity calibration to a raw ADC voltage (V).
 * Linear model: NTU = (zeroV - voltage) * slopeNTUperV
 *
 * @param idx   Sensor index: 0=C2, 1=C5, 2=C6
 * @param volt  Raw voltage in volts (analogRead / 1024.0 * 5.0)
 */
float cal_applyTurb(uint8_t idx, float volt);

/**
 * Convert raw ultrasonic distance (cm) to fill percentage (0–100%).
 * Lower distance = higher water level (sensor mounted at top).
 *
 * @param idx    Level sensor index: 0=C2, 1=C3, 2=C4, 3=C5, 4=C6
 * @param rawCm  Raw distance reading in cm
 */
float cal_applyLevel(uint8_t idx, float rawCm);

/**
 * Apply temperature offset correction.
 *
 * @param idx    Sensor index: 0=C2, 1=C5, 2=C6
 * @param rawC   Raw DS18B20 reading in °C
 */
float cal_applyTemp(uint8_t idx, float rawC);

#endif // CALIBRATION_H
