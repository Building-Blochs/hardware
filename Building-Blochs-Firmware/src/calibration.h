#ifndef __CALIBRATION_H
#define __CALIBRATION_H

// Interactive calibration of the rotation correction per axis, started from the
// settings screen. There are no position sensors, so the user is the sensor:
// a mark on the sphere is rotated through 8 half turns, which multiplies the
// error of one gate by 8, and the user nudges it back to where it started.
// The nudge needed is what the correction should have added.

void calibration_create();
void calibration_start();
void calibration_loop();
bool calibration_active();

#endif // __CALIBRATION_H
