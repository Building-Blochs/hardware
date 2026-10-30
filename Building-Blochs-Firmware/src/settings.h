#ifndef __SETTINGS_H
#define __SETTINGS_H

#include <stdint.h>

// User settings, edited on the GUI settings screen and kept across reboots.
typedef struct
{
    uint8_t speed_percent; // of MOTORS_MAX_SPEED / MOTORS_MAX_ACCEL
    uint8_t brightness;    // display backlight, 0-255
    bool flip_pages;       // false = swipe between pages, true = side bars
    uint8_t dim_minutes;   // dim the display when idle this long, 0 = never
} settings_t;

extern settings_t settings;

void settings_load();
void settings_save();
void settings_restore_defaults();

#endif // __SETTINGS_H
