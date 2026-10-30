#include "i2c_bus.h"

#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>

static SemaphoreHandle_t bus_mutex = xSemaphoreCreateMutex();

void i2c_bus_lock()
{
    xSemaphoreTake(bus_mutex, portMAX_DELAY);
}

void i2c_bus_unlock()
{
    xSemaphoreGive(bus_mutex);
}
