#include <Arduino.h>
#include "ble_update.h"
#include "wifi_update.h"
#include <esp_ota_ops.h>
#include <esp_timer.h>
#include <atomic>

// Defer Arduino's default early confirmation until our application is running.
extern "C" bool verifyRollbackLater() { return true; }
static esp_timer_handle_t bootDeadline;
static bool pendingBoot = false;
static bool motorReady = false;
static uint32_t bootStarted;
static std::atomic<uint32_t> stepperHeartbeat{0};
static void bootExpired(void *) { ESP.restart(); }
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
#include "settings.h"

BlochSphere bloch;

// Diagnostic service gaps, not physical step measurements. No serial I/O here.
static constexpr uint32_t STEP_PERIOD_US = 1000000UL / MOTORS_MAX_SPEED;
static std::atomic<uint32_t> motorGapMax{0}, motorOverBudget{0}, motorOver1ms{0};
static std::atomic<uint32_t> motorSamples{0};

static void stepper_task(void *)
{
    uint32_t previous = 0;
    bool previousBusy = false;
    for (;;)
    {
        const bool busy = bloch.isBusy();
        const uint32_t now = micros();
        // Exclude idle sleeps and the first call after idle.
        if (busy && previousBusy) {
            const uint32_t gap = now - previous;
            motorSamples.fetch_add(1, std::memory_order_relaxed);
            uint32_t peak = motorGapMax.load(std::memory_order_relaxed);
            while (gap > peak && !motorGapMax.compare_exchange_weak(
                peak, gap, std::memory_order_relaxed)) {}
            if (gap > STEP_PERIOD_US) motorOverBudget.fetch_add(1, std::memory_order_relaxed);
            if (gap > 1000) motorOver1ms.fetch_add(1, std::memory_order_relaxed);
        }
        previous = now;
        previousBusy = busy;
        bloch.loop();
        stepperHeartbeat = millis();
        if (!bloch.isBusy()) {
            previousBusy = false;
            vTaskDelay(1);  // feed watchdog when idle
        }
    }
}

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

    esp_ota_img_states_t state;
    pendingBoot = esp_ota_get_state_partition(esp_ota_get_running_partition(), &state) == ESP_OK
        && state == ESP_OTA_IMG_PENDING_VERIFY;
    bootStarted = millis();
    if (pendingBoot) {
        esp_timer_create_args_t args{};
        args.callback = bootExpired;
        args.name = "ota_boot_deadline";
        if (esp_timer_create(&args, &bootDeadline) != ESP_OK ||
            esp_timer_start_once(bootDeadline, 30000000) != ESP_OK)
            esp_ota_mark_app_invalid_rollback_and_reboot();
    }
    settings_load();
    wifi_update_setup();
    receiver_setup();
    gui_begin();
    motorReady = bloch.begin();

    // Home the sphere to |0⟩ = (0, 0, 1) on startup.
    bloch.rotate(Eigen::Vector3d(0, 0, 1), STEPS_PER_ROTATION);

    // Stepper runs on core 0; GUI + receiver run on core 1 (Arduino loop).
    xTaskCreatePinnedToCore(stepper_task, "stepper", 4096, nullptr, 5, nullptr, 0);

    DBG("Setup done.");
    DBG("Step period at max speed: %u µs", STEP_PERIOD_US);
}

void loop()
{
    if (pendingBoot && millis()-bootStarted >= 10000) {
        if (!motorReady || millis()-stepperHeartbeat.load() > 1000)
            esp_ota_mark_app_invalid_rollback_and_reboot();
        if (esp_ota_mark_app_valid_cancel_rollback() == ESP_OK) {
            pendingBoot = false;
            esp_timer_stop(bootDeadline);
            esp_timer_delete(bootDeadline);
        }
    }
    ble_update_loop();
    if (ble_update_active()) {
        receiver_loop(); // discard any queued commands; keep UI/motion controls suspended
        gui_loop(); // the update overlay blocks touch controls while status stays live
        delay(1);
        return;
    }
    // Timing stays here to measure how long GUI/receiver block core 1.
    // Core separation alone does not guarantee motor deadlines.
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
        // Independent snapshots can place boundary events in adjacent reports.
        Serial.printf("[timing] ms=%lu gui_max_us=%u receiver_max_us=%u motor_samples=%u motor_gap_max_us=%u motor_over_budget=%u motor_over_1ms=%u budget_us=%u\n",
                      (unsigned long)now_ms, stats.gui_max_us, stats.receiver_max_us,
                      motorSamples.exchange(0), motorGapMax.exchange(0),
                      motorOverBudget.exchange(0), motorOver1ms.exchange(0), STEP_PERIOD_US);
        stats = {};
    }
}
