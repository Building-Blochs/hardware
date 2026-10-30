#ifndef __CONFIG_H
#define __CONFIG_H

// Firmware version (MAJOR.MINOR.PATCH), shown on the RESET screen.
// Bump for every release and tag the commit as v<version>.
#define VERSIONSTR "1.1.0"

#define MOTOR_ANGLE_DEG         30
#define BASE_Z_ROTATION_DEG     30

// Globe Diam : 160mm
// Wheel Diam : 58mm
// Motor steps: 200 * microstepping = 200*16 = 3200
// Steps per revolution: 3200 steps / 58mm * 160mm = 8827
#define STEPS_PER_ROTATION         		(3200*160/58)

// Calibration: an X / Y gate (180 deg about that axis) was measured to fall
// short by this many degrees on the physical sphere.  The matching component
// of every motor command is scaled up to compensate (the software Bloch state
// is not affected).
#define X_ROTATION_SHORTFALL_DEG    3.396
#define Y_ROTATION_SHORTFALL_DEG    1.83673469388
#define X_ROTATION_CORRECTION       (180.0 / (180.0 - X_ROTATION_SHORTFALL_DEG))
#define Y_ROTATION_CORRECTION       (180.0 / (180.0 - Y_ROTATION_SHORTFALL_DEG))

// #define MOTORS_MAX_SPEED        20000
// #define MOTORS_MAX_ACCEL		5000
#define MOTORS_MAX_SPEED        15000
#define MOTORS_MAX_ACCEL		4000
#define MIN_SPEED_PERCENT		20					// Lowest speed on the settings screen

#define DISPLAY_MAXFPS			30					// LVGL Update rate (timer_event)
#define DISPLAY_WIDTH			320
#define DISPLAY_HEIGHT			240
#define DEFAULT_BRIGHTNESS		127
#define MIN_BRIGHTNESS			10					// Keeps the display readable
#define DIM_BRIGHTNESS			20					// Backlight while idle-dimmed

// PINS

#endif // __CONFIG_H
