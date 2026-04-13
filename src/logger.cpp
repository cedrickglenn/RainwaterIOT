#include "logger.h"
#include "config.h"

void logger_init()
{
    // Serial1 is the ESP32 link — init here so boot-time log frames
    // reach the ESP32 before comms_init() is called later in setup().
    // comms_init() calling Serial1.begin() again is harmless.
    Serial1.begin(ESP32_BAUD_RATE);
    Serial2.begin(LOG_BAUD_RATE);
}

void logEvent(const char* level, const char* category, const String& message)
{
    // ── Build the frame once ─────────────────────────────────────────
    // Local copy (Serial2 / USB): plain format
    //   L,INFO,SYSTEM,Boot complete
    // ESP32 / MQTT copy: [mega] prefix so the dashboard can tell the
    // two controllers apart
    //   L,INFO,SYSTEM,[mega] Boot complete
    String frame = String("L,") + level + "," + category + ",";

    Serial.println(frame + message);
    Serial2.println(frame + message);
    Serial1.println(frame + "[MEGA] " + message);
}
