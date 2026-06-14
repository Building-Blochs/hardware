#include <Arduino.h>
#include <AccelStepper.h>
#include <ArduinoEigenDense.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#include "config.h"
#include "log.h"
#include "bloch.h"
#include "gui.h"
#include "receiver.h"

BlochSphere bloch;

static void stepper_task(void *)
{
    for (;;)
    {
        bloch.loop();
        if (!bloch.isBusy())
            vTaskDelay(1);  // feed watchdog when idle
    }
}

// ---------------------------------------------------------------------------
// Loop timing: measures the gap between consecutive bloch.loop() calls and
// the duration of each interleaved call.  At MOTORS_MAX_SPEED steps/sec the
// stepper needs a call every (1e6 / MOTORS_MAX_SPEED) µs; gaps longer than
// that will cause missed steps.
// ---------------------------------------------------------------------------
static constexpr uint32_t STEP_PERIOD_US = 1000000UL / MOTORS_MAX_SPEED;
static constexpr uint32_t REPORT_MS      = 2000;

struct LoopStats {
    uint32_t receiver_max_us = 0;
    uint32_t gui_max_us      = 0;
};

static LoopStats stats;
static uint32_t  last_report_ms = 0;

void setup()
{
    Serial.begin(115200);

    receiver_setup();
    gui_begin();
    bloch.begin();

    // Home the sphere to |0⟩ = (0, 0, 1) on startup.
    bloch.rotate(Eigen::Vector3d(0, 0, 1), STEPS_PER_ROTATION);

    // Stepper runs on core 0; GUI + receiver run on core 1 (Arduino loop).
    xTaskCreatePinnedToCore(stepper_task, "stepper", 4096, nullptr, 5, nullptr, 0);

    DBG("Setup done.");
    DBG("Step period at max speed: %u µs", STEP_PERIOD_US);
}

void loop()
{
    // Timing stays here to measure how long GUI/receiver block core 1.
    // The stepper itself is now immune to these delays (runs on core 0).
    uint32_t t0 = micros();
    receiver_loop();
    uint32_t receiver_us = micros() - t0;
    if (receiver_us > stats.receiver_max_us) stats.receiver_max_us = receiver_us;

    t0 = micros();
    gui_loop();
    uint32_t gui_us = micros() - t0;
    if (gui_us > stats.gui_max_us) stats.gui_max_us = gui_us;

    // Print summary every REPORT_MS ms.
    uint32_t now_ms = millis();
    if (now_ms - last_report_ms >= REPORT_MS) {
        last_report_ms = now_ms;
        Serial.printf("[timing] gui_max=%uµs  receiver_max=%uµs  (stepper on core 0, unaffected)\n",
                      stats.gui_max_us, stats.receiver_max_us);
        stats = {};
    }
}
