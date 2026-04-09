#include "adxl345.h"
#include "driver/i2c.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "gui_guider.h"
#include "lvgl.h"
#include "esp_lvgl_port.h"
#include "gatts_table_creat_demo.h"
#include <math.h>
#include <inttypes.h>

static const char *TAG = "ADXL345";

#define ADXL345_I2C_SCL 16
#define ADXL345_I2C_SDA 17
#define ADXL345_I2C_NUM 1
#define ADXL345_I2C_FREQ_HZ 100000
#define ADXL345_I2C_TX_BUF_DISABLE 0
#define ADXL345_I2C_RX_BUF_DISABLE 0
#define ADXL345_I2C_TIMEOUT_MS 1000

#define ADXL345_ADDR 0x53
#define ADXL345_DEVID_REG 0x00
#define ADXL345_DEVID 0xE5
#define ADXL345_POWER_CTL_REG 0x2D
#define ADXL345_DATA_FORMAT_REG 0x31
#define ADXL345_DATAX0_REG 0x32

static bool adxl345_present = false;

#define STEP_THRESHOLD 0.2f
#define STEP_INTERVAL_MS 300
#define WINDOW_SIZE 5

static uint32_t step_count = 0;
static bool step_detected = false;
static uint32_t last_step_time = 0;
static float accel_history[WINDOW_SIZE];
static int history_index = 0;

static float current_x = 0.0f;
static float current_y = 0.0f;
static float current_z = 0.0f;
static lv_ui *ui_ptr = NULL;

void adxl345_set_ui_ptr(lv_ui *ui)
{
    ui_ptr = ui;
}

static esp_err_t adxl345_register_write_byte(uint8_t reg_addr, uint8_t data);
static esp_err_t adxl345_register_read(uint8_t reg_addr, uint8_t *data, size_t len);
static esp_err_t adxl345_register_read_with_retry(uint8_t reg_addr, uint8_t *data, size_t len, int max_retries);

static esp_err_t adxl345_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command link for write");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADXL345_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ADXL345_I2C_NUM, cmd, ADXL345_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t adxl345_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command link for read phase 1");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADXL345_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(ADXL345_I2C_NUM, cmd, ADXL345_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);

    if (ret != ESP_OK) {
        return ret;
    }

    cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command link for read phase 2");
        return ESP_ERR_NO_MEM;
    }

    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, ADXL345_ADDR << 1 | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(ADXL345_I2C_NUM, cmd, ADXL345_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

static esp_err_t adxl345_register_read_with_retry(uint8_t reg_addr, uint8_t *data, size_t len, int max_retries)
{
    esp_err_t err;
    for (int i = 0; i < max_retries; i++) {
        err = adxl345_register_read(reg_addr, data, len);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Read from register 0x%02x failed (attempt %d/%d): %s", reg_addr, i+1, max_retries, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return err;
}

static int detect_step(float x, float y, float z)
{
    float magnitude = sqrtf(x*x + y*y + z*z) - 1.0f;

    accel_history[history_index] = magnitude;
    history_index = (history_index + 1) % WINDOW_SIZE;

    float avg_magnitude = 0.0f;
    for (int i = 0; i < WINDOW_SIZE; i++) {
        avg_magnitude += accel_history[i];
    }
    avg_magnitude /= WINDOW_SIZE;

    uint32_t current_time = xTaskGetTickCount() * portTICK_PERIOD_MS;

    if (avg_magnitude > STEP_THRESHOLD && !step_detected) {
        step_detected = true;
    } else if (avg_magnitude < STEP_THRESHOLD * 0.5f && step_detected) {
        if (current_time - last_step_time > STEP_INTERVAL_MS) {
            step_count++;
            last_step_time = current_time;
            step_detected = false;
            ble_send_steps(step_count);
            return 1;
        }
        step_detected = false;
    }

    return 0;
}

uint32_t adxl345_get_step_count(void)
{
    return step_count;
}

static void adxl345_read_task(void *pvParam)
{
    int16_t x, y, z;
    uint8_t data[6];
    float x_g, y_g, z_g;
    int error_count = 0;
    int last_log_step_count = 0;

    for (int i = 0; i < WINDOW_SIZE; i++) {
        accel_history[i] = 0.0f;
    }

    ESP_LOGI(TAG, "ADXL345 reading task started, step detection enabled");

    while (1) {
        if (adxl345_present) {
            esp_err_t err = adxl345_register_read(ADXL345_DATAX0_REG, data, 6);
            if (err == ESP_OK) {
                error_count = 0;
                x = (int16_t)((data[1] << 8) | data[0]);
                y = (int16_t)((data[3] << 8) | data[2]);
                z = (int16_t)((data[5] << 8) | data[4]);

                x_g = (float)x / 256.0f;
                y_g = (float)y / 256.0f;
                z_g = (float)z / 256.0f;

                current_x = x_g;
                current_y = y_g;
                current_z = z_g;

                int step_detected = detect_step(x_g, y_g, z_g);
                if (step_detected) {
                    ESP_LOGI(TAG, "Step detected! Total steps: %" PRIu32, step_count);
                }

                if (step_count != last_log_step_count) {
                    ESP_LOGI(TAG, "X: %.2f g, Y: %.2f g, Z: %.2f g | Steps: %" PRIu32, x_g, y_g, z_g, step_count);
                    last_log_step_count = step_count;
                }

                if (ui_ptr != NULL && ui_ptr->step != NULL) {
                    lvgl_port_lock(0);
                    char buf[32];
                    snprintf(buf, sizeof(buf), "X: %.2f", current_x);
                    lv_label_set_text(ui_ptr->step_x_g, buf);
                    snprintf(buf, sizeof(buf), "Y: %.2f", current_y);
                    lv_label_set_text(ui_ptr->step_y_g, buf);
                    snprintf(buf, sizeof(buf), "Z: %.2f", current_z);
                    lv_label_set_text(ui_ptr->step_z_g, buf);
                    snprintf(buf, sizeof(buf), "%" PRIu32 " Step", step_count);
                    lv_label_set_text(ui_ptr->step_step_count, buf);
                    lvgl_port_unlock();
                }
            } else {
                error_count++;
                if (error_count % 10 == 0) {
                    ESP_LOGE(TAG, "Failed to read sensor data: %s (error count: %d)", esp_err_to_name(err), error_count);
                }
                if (error_count > 20) {
                    ESP_LOGE(TAG, "Too many consecutive errors, pausing for 5 seconds");
                    vTaskDelay(pdMS_TO_TICKS(5000));
                    error_count = 0;
                }
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        } else {
            ESP_LOGI(TAG, "ADXL345 sensor not present, reducing polling frequency");
            vTaskDelay(pdMS_TO_TICKS(5000));
        }
    }
}

void adxl345_init(void)
{
    uint8_t dev_id;
    esp_err_t err;

    ESP_LOGI(TAG, "Initializing ADXL345 sensor...");
    ESP_LOGI(TAG, "I2C configuration: SCL:%d SDA:%d Port:%d", ADXL345_I2C_SCL, ADXL345_I2C_SDA, ADXL345_I2C_NUM);
    ESP_LOGI(TAG, "CS connected to VSS, I2C address: 0x%02x", ADXL345_ADDR);

    err = adxl345_register_read_with_retry(ADXL345_DEVID_REG, &dev_id, 1, 3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read ADXL345 device ID: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "ADXL345 sensor will not be available");
        adxl345_present = false;
        return;
    }

    if (dev_id != ADXL345_DEVID) {
        ESP_LOGE(TAG, "ADXL345 device ID mismatch: expected 0x%02x, got 0x%02x", ADXL345_DEVID, dev_id);
        adxl345_present = false;
        return;
    }

    ESP_LOGI(TAG, "ADXL345 device ID matched: 0x%02x", dev_id);
    adxl345_present = true;

    err = adxl345_register_write_byte(ADXL345_POWER_CTL_REG, 0x08);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ADXL345 to measurement mode: %s", esp_err_to_name(err));
        adxl345_present = false;
        return;
    }
    ESP_LOGI(TAG, "ADXL345 entered measurement mode");

    err = adxl345_register_write_byte(ADXL345_DATA_FORMAT_REG, 0x08);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set ADXL345 data format: %s", esp_err_to_name(err));
        adxl345_present = false;
        return;
    }
    ESP_LOGI(TAG, "ADXL345 data format set to +/-2g range, 4mg/LSB");

    xTaskCreate(adxl345_read_task, "adxl345_read", 4096, NULL, 5, NULL);
    ESP_LOGI(TAG, "ADXL345 reading task started");
    ESP_LOGI(TAG, "ADXL345 initialization complete");
}
