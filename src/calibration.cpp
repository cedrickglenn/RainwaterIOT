#include "calibration.h"

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  CALIBRATION — Implementation
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  DEFAULT VALUES — edit these to match your hardware before first flash.
 *
 *  pH defaults:
 *    neutralV = 2530 mV  (typical DFRobot SEN0161 in pH 7.00 buffer)
 *    acidV    = 2030 mV  (typical in pH 4.01 buffer)
 *    These will be overwritten once you run the pH calibration procedure.
 *
 *  Turbidity defaults:
 *    zeroV        = 4.2 V  (typical reading in distilled water for TSD-10)
 *    slopeNTUperV = 320.0  (rough value — run span calibration to correct)
 *
 *  Level defaults:
 *    emptyCm = 100.0 cm  (sensor-to-bottom when container is empty)
 *    fullCm  =  10.0 cm  (sensor-to-water when container is full)
 *    Measure your containers and update before calibrating.
 *
 *  Temperature default offset: 0.0°C (DS18B20 is ±0.5°C factory accurate)
 *
 *  Flow default: 450.0 pulses/litre (YF-S201 typical — measure yours)
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

// ── Global calibration instance ──────────────────────────────────────────
CalibrationData calData;

// ── Factory defaults ──────────────────────────────────────────────────────
static const CalibrationData DEFAULTS = {
    // ph[3]: {neutralV mV, acidV mV}  — [0]=C2, [1]=C5, [2]=C6
    .ph = {
        { 2530.0f, 2030.0f },   // C2
        { 2530.0f, 2030.0f },   // C5
        { 2530.0f, 2030.0f },   // C6
    },

    // turb[3]: {zeroV V, slopeNTUperV}
    .turb = {
        { 4.2f, 320.0f },   // C2
        { 4.2f, 320.0f },   // C5
        { 4.2f, 320.0f },   // C6
    },

    // level[5]: {emptyCm, fullCm, calibrated} — [0]=C2,[1]=C3,[2]=C4,[3]=C5,[4]=C6
    .level = {
        { 100.0f, 10.0f, false },  // C2
        { 100.0f, 10.0f, false },  // C3
        { 100.0f, 10.0f, false },  // C4
        { 100.0f, 10.0f, false },  // C5
        { 100.0f, 10.0f, false },  // C6
    },

    // tempOffset[3] — °C correction
    .tempOffset = { 0.0f, 0.0f, 0.0f },

    // flowPPL — pulses per litre (YF-S201 typical)
    .flowPPL = 450.0f,
};

// ═════════════════════════════════════════════════════════════════════════
//  EEPROM helpers
// ═════════════════════════════════════════════════════════════════════════

void cal_save()
{
    EEPROM.put(EEPROM_MAGIC_ADDR,     (uint16_t)EEPROM_MAGIC);
    EEPROM.put(EEPROM_MAGIC_ADDR + 2, calData);
    Serial.println(F("[Cal] Calibration saved to EEPROM"));
}

void cal_reset()
{
    calData = DEFAULTS;
    cal_save();
    Serial.println(F("[Cal] Calibration reset to defaults"));
}

void cal_init()
{
    uint16_t magic = 0;
    EEPROM.get(EEPROM_MAGIC_ADDR, magic);

    if (magic == EEPROM_MAGIC) {
        EEPROM.get(EEPROM_MAGIC_ADDR + 2, calData);
        Serial.println(F("[Cal] Loaded calibration from EEPROM"));
    } else {
        // First boot — EEPROM has never been written
        Serial.println(F("[Cal] No EEPROM data found — loading defaults"));
        cal_reset();
    }
}

// ═════════════════════════════════════════════════════════════════════════
//  Apply functions
// ═════════════════════════════════════════════════════════════════════════

float cal_applyPH(uint8_t idx, float mV, float tempC)
{
    if (idx >= CAL_QTY_SENSORS) return 7.0f;

    const PhCalData& p = calData.ph[idx];

    // Two-point linear calibration:
    //   At neutralV → pH 7.00
    //   At acidV    → pH 4.01
    //   slope = (7.00 - 4.01) / (neutralV - acidV) = 2.99 / (neutralV - acidV)
    float dV = p.neutralV - p.acidV;
    if (fabsf(dV) < 1.0f) return 7.0f;   // guard against uncalibrated / zero slope

    // Nernst temperature correction: the electrode's mV/pH slope is proportional
    // to absolute temperature.  Calibration buffers are captured at ambient temp
    // but the formula below scales the slope continuously with the live reading.
    //   tempFactor = (273.15 + T) / 298.15   (298.15 K = 25 °C reference)
    // Clamp input to a plausible range (0–60 °C) to guard against disconnected
    // DS18B20 returning DEVICE_DISCONNECTED (-127 °C) or other fault values.
    float tC = tempC;
    if (tC < 0.0f)  tC = 25.0f;
    if (tC > 60.0f) tC = 25.0f;
    float tempFactor = (273.15f + tC) / 298.15f;

    float slope = (2.99f / dV) * tempFactor;
    float ph    = 7.0f + (p.neutralV - mV) * slope;

    // Clamp to physically meaningful range
    if (ph < 0.0f)  ph = 0.0f;
    if (ph > 14.0f) ph = 14.0f;
    return ph;
}

float cal_applyTurb(uint8_t idx, float volt)
{
    if (idx >= CAL_QTY_SENSORS) return 0.0f;

    const TurbCalData& t = calData.turb[idx];

    // Linear model: NTU = (zeroV - voltage) * slopeNTUperV
    // When voltage == zeroV (distilled water), NTU = 0.
    // As voltage drops (more turbid), NTU increases.
    float ntu = (t.zeroV - volt) * t.slopeNTUperV;

    if (ntu < 0.0f) ntu = 0.0f;
    return ntu;
}

float cal_applyLevel(uint8_t idx, float rawCm)
{
    if (idx >= CAL_LVL_SENSORS) return 0.0f;

    const LevelCalData& l = calData.level[idx];

    // Distance decreases as water level rises (sensor at top).
    // pct = (emptyCm - rawCm) / (emptyCm - fullCm) * 100
    float range = l.emptyCm - l.fullCm;
    if (fabsf(range) < 1.0f) return 0.0f;   // guard against uncalibrated

    float pct = (l.emptyCm - rawCm) / range * 100.0f;

    if (pct < 0.0f)   pct = 0.0f;
    if (pct > 100.0f) pct = 100.0f;
    return pct;
}

float cal_applyTemp(uint8_t idx, float rawC)
{
    if (idx >= CAL_QTY_SENSORS) return rawC;
    return rawC + calData.tempOffset[idx];
}

bool cal_isLevelCalibrated(uint8_t idx)
{
    if (idx >= CAL_LVL_SENSORS) return false;
    return calData.level[idx].calibrated;
}
