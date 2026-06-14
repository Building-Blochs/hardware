#include <Arduino.h>
#include <AccelStepper.h>
#include <ArduinoEigenDense.h>

#include "config.h"
#include "log.h"
#include "bloch.h"
#include "gui.h"
#include "receiver.h"

BlochSphere bloch;

void setup()
{
    Serial.begin(115200);

    receiver_setup();
    gui_begin();
    bloch.begin();

    // Home the sphere to |0⟩ = (0, 0, 1) on startup.
    bloch.rotate(Eigen::Vector3d(0, 0, 1), STEPS_PER_ROTATION);

    DBG("Setup done.");
}

void loop()
{
    gui_loop();
    receiver_loop();
    bloch.loop();
}
