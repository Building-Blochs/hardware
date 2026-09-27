#ifndef __CONFIG_H
#define __CONFIG_H

#define VERSIONSTR "20241018"

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

#define DISPLAY_MAXFPS			30					// LVGL Update rate (timer_event)
#define DISPLAY_WIDTH			320
#define DISPLAY_HEIGHT			240

// PINS

#endif // __CONFIG_H
