#include "settings.h"

#include <Arduino.h>
#include <Preferences.h>

#include "config.h"

static const settings_t kDefaults = {
    100,
    DEFAULT_BRIGHTNESS,
    false,
    0,
    {(float)X_ROTATION_CORRECTION, (float)Y_ROTATION_CORRECTION, (float)Z_ROTATION_CORRECTION},
    1,
};

static const char *const kCorrectionKeys[] = {"corr_x", "corr_y", "corr_z"};

settings_t settings = kDefaults;

void settings_load()
{
    Preferences prefs;
    prefs.begin("settings", true);
    settings.speed_percent = prefs.getUChar("speed", kDefaults.speed_percent);
    settings.brightness = prefs.getUChar("brightness", kDefaults.brightness);
    settings.flip_pages = prefs.getBool("flip", kDefaults.flip_pages);
    settings.dim_minutes = prefs.getUChar("dim", kDefaults.dim_minutes);
    settings.machine_number = prefs.getUChar("machine", kDefaults.machine_number);
    for (int i = 0; i < 3; ++i)
    {
        settings.correction[i] = prefs.getFloat(kCorrectionKeys[i], kDefaults.correction[i]);
        if (settings.correction[i] < MIN_ROTATION_CORRECTION || settings.correction[i] > MAX_ROTATION_CORRECTION)
        {
            settings.correction[i] = kDefaults.correction[i];
        }
    }
    prefs.end();

    settings.speed_percent = constrain(settings.speed_percent, MIN_SPEED_PERCENT, 100);
    settings.brightness = constrain(settings.brightness, MIN_BRIGHTNESS, 255);
    settings.machine_number = constrain(settings.machine_number, 1, MAX_MACHINE_NUMBER);
}

// Flash writes stall both cores, so never call this while the motors are moving.
void settings_save()
{
    Preferences prefs;
    prefs.begin("settings", false);
    prefs.putUChar("speed", settings.speed_percent);
    prefs.putUChar("brightness", settings.brightness);
    prefs.putBool("flip", settings.flip_pages);
    prefs.putUChar("dim", settings.dim_minutes);
    prefs.putUChar("machine", settings.machine_number);
    for (int i = 0; i < 3; ++i)
    {
        prefs.putFloat(kCorrectionKeys[i], settings.correction[i]);
    }
    prefs.end();
}

void settings_restore_defaults()
{
    settings = kDefaults;
}
