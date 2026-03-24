#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/event_groups.h"

#include "esp_system.h"
#include "esp_log.h"
#include "esp_err.h" 

#include "driver/i2c.h"
#include "driver/gpio.h"

void max30102_init();
void max30102_start_continuous_detection();
void max30102_stop_continuous_detection();
void max30102_start_idle_detection();
void max30102_stop_idle_detection();

extern bool sensor_active;
extern uint8_t xinlv;