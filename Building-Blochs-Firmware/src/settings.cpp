#include "settings.h"

#include <Arduino.h>
#include <Preferences.h>

#include "config.h"

static const settings_t kDefaults = {100, DEFAULT_BRIGHTNESS, false, 0};

settings_t settings = kDefaults;

void settings_load()
{
    Preferences prefs;
    prefs.begin("settings", true);
    settings.speed_percent = prefs.getUChar("speed", kDefaults.speed_percent);
    settings.brightness = prefs.getUChar("brightness", kDefaults.brightness);
    settings.flip_pages = prefs.getBool("flip", kDefaults.flip_pages);
    settings.dim_minutes = prefs.getUChar("dim", kDefaults.dim_minutes);
    prefs.end();

    settings.speed_percent = constrain(settings.speed_percent, MIN_SPEED_PERCENT, 100);
    settings.brightness = constrain(settings.brightness, MIN_BRIGHTNESS, 255);
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
    prefs.end();
}

void settings_restore_defaults()
{
    settings = kDefaults;
}
