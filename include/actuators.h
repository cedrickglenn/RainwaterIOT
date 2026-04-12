#ifndef ACTUATORS_H
#define ACTUATORS_H

#include <Arduino.h>

/*
 * ═══════════════════════════════════════════════════════════════════════════
 *  ACTUATORS MODULE — Solenoid Valve & Pump Control
 * ═══════════════════════════════════════════════════════════════════════════
 *
 *  ALL relay I/O for valves and pumps is funnelled through this module.
 *  No other file should call digitalWrite() on actuator pins directly.
 *
 *  The relay boards are ACTIVE-LOW:
 *    LOW  on the pin → relay coil energised → valve/pump ON
 *    HIGH on the pin → relay coil off       → valve/pump OFF (safe state)
 *
 *  SAFETY NOTES:
 *  ● On power-up, actuators_init() forces every relay HIGH (OFF) before
 *    anything else runs.  This is the first call in setup().
 *  ● actuators_emergencyStop() is the "big red button" — kills everything
 *    instantly.  It can be triggered by Container 6 overflow detection,
 *    remote ESTOP from the app, or a sensor fault.
 *  ● Pumps must NEVER run dry.  Always check the upstream water level
 *    in the pipeline module before enabling a pump.
 *
 *  SUGGESTIONS:
 *  ● Add a small delay (50–100 ms) between opening a valve and starting
 *    the downstream pump.  This lets pressure equalise and prevents
 *    water hammer in PVC piping.
 *  ● If budget allows, add ACS712 current sensors on each pump line.
 *    A current reading near zero while the relay is ON indicates the
 *    pump stalled or lost prime — trigger an immediate shutdown.
 *  ● Consider a hardware watchdog timer (e.g., TPL5010 IC) that resets
 *    the Mega if it hangs.  In the reset handler, all relays go HIGH
 *    automatically (safe state).
 *
 * ═══════════════════════════════════════════════════════════════════════════
 */

/**
 * Initialise all valve and pump pins as OUTPUT and drive them HIGH (OFF).
 * MUST be the first module initialised in setup().
 */
void actuators_init();

/**
 * Open a solenoid valve (energise relay → pin LOW).
 * @param pin  Use VALVE*_PIN constants from pins.h
 */
void valve_open(uint8_t pin);

/**
 * Close a solenoid valve (de-energise relay → pin HIGH).
 * @param pin  Use VALVE*_PIN constants from pins.h
 */
void valve_close(uint8_t pin);

/**
 * Start a pump (energise relay → pin LOW).
 * @param pin  Use PUMP*_PIN constants from pins.h
 */
void pump_start(uint8_t pin);

/**
 * Stop a pump (de-energise relay → pin HIGH).
 * @param pin  Use PUMP*_PIN constants from pins.h
 */
void pump_stop(uint8_t pin);

/**
 * EMERGENCY STOP — immediately de-energise every valve and pump relay.
 * Called when Container 6 is full, a critical fault occurs, or a remote
 * ESTOP command arrives from the app.
 */
void actuators_emergencyStop();

/**
 * Check whether a specific actuator relay is currently energised.
 * @param pin  The pin number to query
 * @return     true if the relay is ON (pin reads LOW)
 */
bool actuator_isOn(uint8_t pin);

#endif // ACTUATORS_H
