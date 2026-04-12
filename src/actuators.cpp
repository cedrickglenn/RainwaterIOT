#include "actuators.h"
#include "pins.h"

/*
 *  IMPLEMENTATION NOTES:
 *
 *  The allPins[] array is the single source of truth for every relay-
 *  controlled output.  If you add new valves or pumps, append them here
 *  so that actuators_init() and actuators_emergencyStop() automatically
 *  cover them.
 */

// Master list of every actuator pin — keep in sync with pins.h!
static const uint8_t allPins[] = {
    VALVE1_PIN, VALVE2_PIN, VALVE3_PIN, VALVE4_PIN,
    VALVE5_PIN, VALVE6_PIN, VALVE7_PIN, VALVE8_PIN,
    PUMP1_PIN,  PUMP2_PIN,  PUMP3_PIN,  PUMP4_PIN
};
static const uint8_t NUM_ACTUATORS = sizeof(allPins) / sizeof(allPins[0]);

// ─────────────────────────────────────────────────────────────────────────
void actuators_init()
{
    for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
        digitalWrite(allPins[i], HIGH);   // Pre-load HIGH *before* becoming output
        pinMode(allPins[i], OUTPUT);      // Pin transitions to output already at HIGH → relay never glitches
    }

    // SUGGESTION: Briefly pulse each relay in sequence during a "self-test"
    //             mode (enabled by a jumper or serial command) so you can
    //             hear/see each one click.  Useful during installation.
}

// ─────────────────────────────────────────────────────────────────────────
void valve_open(uint8_t pin)
{
    digitalWrite(pin, LOW);    // Active-LOW → LOW = energised = open
}

void valve_close(uint8_t pin)
{
    digitalWrite(pin, HIGH);   // HIGH = de-energised = closed
}

// ─────────────────────────────────────────────────────────────────────────
void pump_start(uint8_t pin)
{
    // SUGGESTION: Before starting a pump, verify the upstream valve is
    //             open (actuator_isOn) and the upstream water level is OK.
    //             This can be asserted here or left to the caller (pipeline).
    digitalWrite(pin, LOW);
}

void pump_stop(uint8_t pin)
{
    digitalWrite(pin, HIGH);
}

// ─────────────────────────────────────────────────────────────────────────
void actuators_emergencyStop()
{
    // Kill everything.  No conditions, no delays.
    for (uint8_t i = 0; i < NUM_ACTUATORS; i++) {
        digitalWrite(allPins[i], HIGH);
    }

    // SUGGESTION: Log the emergency-stop event with a timestamp to the
    //             ESP32/database so it can be reviewed in the app.
    //             Also consider driving a buzzer or status LED here.
}

// ─────────────────────────────────────────────────────────────────────────
bool actuator_isOn(uint8_t pin)
{
    return (digitalRead(pin) == LOW);
}
