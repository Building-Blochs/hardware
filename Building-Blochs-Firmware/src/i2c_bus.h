#ifndef __I2C_BUS_H
#define __I2C_BUS_H

// The touch controller and backlight (through LovyanGFX) and the stepper driver
// (through Wire1) share one I2C peripheral. Neither knows about the other, and
// LovyanGFX reprograms the peripheral directly, so a transfer from one aborts a
// transfer in progress from the other. Hold this lock around every I2C access
// that can run after setup(), when the two cores run concurrently.
void i2c_bus_lock();
void i2c_bus_unlock();

#endif // __I2C_BUS_H
