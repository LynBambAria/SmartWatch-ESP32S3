#include "max30102.h"

static const char *TAG = "max30102";

#define MAX30102_I2C_SCL 16           // GPIO number used for I2C master clock
#define MAX30102_I2C_SDA 17            // GPIO number used for I2C master data
#define MAX30102_I2C_NUM 1            // I2C master i2c port number, the number of i2c peripheral interfaces available will depend on the chip
#define MAX30102_I2C_FREQ_HZ 10000    // I2C master clock frequency (降低频率提高稳定性) (降低频率提高稳定性)
#define MAX30102_I2C_TX_BUF_DISABLE 0 // I2C master doesn't need buffer
#define MAX30102_I2C_RX_BUF_DISABLE 0
#define MAX30102_I2C_TIMEOUT_MS 1000

#define MAX30102_GPIO_INT 18 // GPIO number used for MAX30102 int

// | B7 | B6 | B5 | B4 | B3 | B2 | B1 | B0 | WRITE ADDRESS | READ ADDRESS |
// | 1  | 0  | 1  | 0  | 1  | 1  | 1  | R/W| 0xAE          | 0xAF         |
#define MAX30102_ADDR 0x57 //  I2C device MAX30102's 7-bit address
#define MAX30102_PART_ID_REG_ADDR 0xff

// 传感器状态
static bool sensor_active = false;

/**
 * @brief init the i2c port for MAX30102
 */
static esp_err_t max30102_i2c_init()
{
    int i2c_master_port = MAX30102_I2C_NUM;

    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = MAX30102_I2C_SDA,
        .scl_io_num = MAX30102_I2C_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = MAX30102_I2C_FREQ_HZ,
    };

    ESP_LOGI(TAG, "Configuring I2C port %d with SDA:%d SCL:%d freq:%dHz", 
             i2c_master_port, MAX30102_I2C_SDA, MAX30102_I2C_SCL, MAX30102_I2C_FREQ_HZ);

    esp_err_t err = i2c_param_config(i2c_master_port, &conf);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to configure I2C parameters: %s", esp_err_to_name(err));
        return err;
    }

    err = i2c_driver_install(i2c_master_port, conf.mode, MAX30102_I2C_RX_BUF_DISABLE, MAX30102_I2C_TX_BUF_DISABLE, 0);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install I2C driver: %s", esp_err_to_name(err));
        return err;
    }

    ESP_LOGI(TAG, "I2C driver installed successfully");
    return ESP_OK;
}

/**
 * @brief write a byte to a register of MAX30102
 */
static esp_err_t max30102_register_write_byte(uint8_t reg_addr, uint8_t data)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command link for write");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MAX30102_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_write_byte(cmd, data, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(MAX30102_I2C_NUM, cmd, MAX30102_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief read a byte from a register of MAX30102
 */
static esp_err_t max30102_register_read(uint8_t reg_addr, uint8_t *data, size_t len)
{
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    if (cmd == NULL) {
        ESP_LOGE(TAG, "Failed to create I2C command link for read phase 1");
        return ESP_ERR_NO_MEM;
    }
    
    i2c_master_start(cmd);
    i2c_master_write_byte(cmd, MAX30102_ADDR << 1 | I2C_MASTER_WRITE, true);
    i2c_master_write_byte(cmd, reg_addr, true);
    i2c_master_stop(cmd);
    esp_err_t ret = i2c_master_cmd_begin(MAX30102_I2C_NUM, cmd, MAX30102_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
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
    i2c_master_write_byte(cmd, MAX30102_ADDR << 1 | I2C_MASTER_READ, true);
    if (len > 1) {
        i2c_master_read(cmd, data, len - 1, I2C_MASTER_ACK);
    }
    i2c_master_read_byte(cmd, data + len - 1, I2C_MASTER_NACK);
    i2c_master_stop(cmd);
    ret = i2c_master_cmd_begin(MAX30102_I2C_NUM, cmd, MAX30102_I2C_TIMEOUT_MS / portTICK_PERIOD_MS);
    i2c_cmd_link_delete(cmd);
    return ret;
}

/**
 * @brief write a byte to a register of MAX30102 with retry
 */
static esp_err_t max30102_register_write_with_retry(uint8_t reg_addr, uint8_t data, int max_retries)
{
    esp_err_t err;
    for (int i = 0; i < max_retries; i++) {
        err = max30102_register_write_byte(reg_addr, data);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Write to register 0x%02x failed (attempt %d/%d): %s", reg_addr, i+1, max_retries, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return err;
}

/**
 * @brief read a byte from a register of MAX30102 with retry
 */
static esp_err_t max30102_register_read_with_retry(uint8_t reg_addr, uint8_t *data, size_t len, int max_retries)
{
    esp_err_t err;
    for (int i = 0; i < max_retries; i++) {
        err = max30102_register_read(reg_addr, data, len);
        if (err == ESP_OK) {
            return ESP_OK;
        }
        ESP_LOGE(TAG, "Read from register 0x%02x failed (attempt %d/%d): %s", reg_addr, i+1, max_retries, esp_err_to_name(err));
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    return err;
}

/**
 * @brief init the gpio for MAX30102
 */
static esp_err_t max30102_gpio_init()
{
    // 配置INT引脚为输入模式，用于将来可能的中断使用
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;
    io_conf.mode = GPIO_MODE_INPUT;
    io_conf.pin_bit_mask = (1ULL << MAX30102_GPIO_INT);
    io_conf.pull_down_en = 0;
    io_conf.pull_up_en = 1;
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    // 注意：不再使用中断，而是使用轮询方式检测
    ESP_LOGI(TAG, "MAX30102 GPIO configured (INT pin: %d)", MAX30102_GPIO_INT);
    
    return ESP_OK;
}

void get_temp()
{
    uint8_t byte[2];
    float temp;
    ESP_ERROR_CHECK(max30102_register_write_byte(0x21, 0x01));
    vTaskDelay(pdMS_TO_TICKS(100));
    ESP_ERROR_CHECK(max30102_register_read(0x1f, &byte[0], 1));
    ESP_ERROR_CHECK(max30102_register_read(0x20, &byte[1], 1));
    temp = (int8_t)(byte[0]) + byte[1] * 0.0625;
    printf("Temp: %f\n", temp);

    // FIFO
    ESP_ERROR_CHECK(max30102_register_write_byte(0x04, 0x00)); // clear FIFO Write Pointer
    ESP_ERROR_CHECK(max30102_register_write_byte(0x05, 0x00)); // clear FIFO Overflow Counter
    ESP_ERROR_CHECK(max30102_register_write_byte(0x06, 0x00)); // clear FIFO Read Pointer

    // clear PPG_RDY ! Cannot receive the first interrupt without clearing !
    uint8_t status_data;
    ESP_ERROR_CHECK(max30102_register_read(0x00, &status_data, 1));
    ESP_ERROR_CHECK(max30102_register_read(0x01, &status_data, 1));
}

void gpio_intr_task()
{
    uint8_t buffer[6];
    int data[2];
    float xueyang;
    uint8_t xinlv;
    int error_count = 0;
    
    // 定期检查是否有物体靠近传感器
    while (1) {
        // 读取传感器数据
        esp_err_t err = max30102_register_read(0x07, &buffer[0], 6);
        if (err == ESP_OK) {
            error_count = 0; // 重置错误计数
            data[0] = ((buffer[0] << 16 | buffer[1] << 8 | buffer[2]) & 0x03ffff);
            data[1] = ((buffer[3] << 16 | buffer[4] << 8 | buffer[5]) & 0x03ffff);

            // 检测是否有物体靠近（光被挡住）
            if (data[0] >= 50000) {  // 阈值可以根据实际情况调整
                if (!sensor_active) {
                    // 检测到物体靠近，开始正常检测
                    ESP_LOGI(TAG, "检测到物体靠近，开始血氧和心率检测");
                    sensor_active = true;
                    
                    // 增加LED亮度以获得更好的信号
                    esp_err_t led_err = max30102_register_write_with_retry(0x0c, 0x7f, 3); // LED1_PA(red) = 0x7f, 最大亮度
                    if (led_err == ESP_OK) {
                        max30102_register_write_with_retry(0x0d, 0x7f, 3); // LED2_PA(IR) = 0x7f, 最大亮度
                    }
                }

                // 只有当传感器激活时才计算和输出数据
                if (sensor_active && data[0] >= 100000) {
                    xueyang = (float)(data[1]) / (float)(data[0]);
                    xinlv = 30.354 * xueyang + 94.845 - 45.060 * xueyang * xueyang;
                    printf("血氧:%f,心率:%d\n", xueyang * 100, xinlv);
                }
            } else {
                if (sensor_active) {
                    // 物体移开，停止检测
                    ESP_LOGI(TAG, "物体移开，停止血氧和心率检测");
                    sensor_active = false;
                    
                    // 降低LED亮度以节省功耗
                    esp_err_t led_err = max30102_register_write_with_retry(0x0c, 0x10, 3); // LED1_PA(red) = 0x10, 低亮度
                    if (led_err == ESP_OK) {
                        max30102_register_write_with_retry(0x0d, 0x10, 3); // LED2_PA(IR) = 0x10, 低亮度
                    }
                    printf("没有手指检测\n");
                }
            }

            // clear PPG_RDY ! Cannot receive the first interrupt without clearing !
            uint8_t status_data;
            max30102_register_read(0x00, &status_data, 1);
            max30102_register_read(0x01, &status_data, 1);
        } else {
            error_count++;
            if (error_count % 10 == 0) { // 每10次错误才打印一次，避免日志刷屏
                ESP_LOGE(TAG, "Failed to read sensor data: %s (error count: %d)", esp_err_to_name(err), error_count);
            }
            // 如果连续错误超过20次，可能是硬件问题，暂停一段时间
            if (error_count > 20) {
                ESP_LOGE(TAG, "Too many consecutive errors, pausing for 5 seconds");
                vTaskDelay(pdMS_TO_TICKS(5000));
                error_count = 0;
            }
        }
        
        // 控制检测频率，避免过于频繁的读取
        vTaskDelay(pdMS_TO_TICKS(200)); // 增加延迟到200毫秒，减少I2C操作频率
    }
}

void max30102_init()
{
    esp_err_t err = max30102_i2c_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 I2C initialization failed: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "MAX30102 sensor will not be available");
        return;
    }
    ESP_LOGI(TAG, "MAX30102 I2C initialized successfully (SDA:%d SCL:%d)", MAX30102_I2C_SDA, MAX30102_I2C_SCL);
    
    err = max30102_gpio_init();
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 GPIO initialization failed: %s", esp_err_to_name(err));
        // 继续执行，不因为GPIO初始化失败而停止
    } else {
        ESP_LOGI(TAG, "MAX30102 GPIO initialized successfully");
    }

    // 启动传感器轮询任务
    xTaskCreate(gpio_intr_task, "max30102_poll", 2048, NULL, 5, NULL);
    ESP_LOGI(TAG, "MAX30102 sensor polling task started");

    // 先尝试读取设备ID，确认I2C连接是否正常
    uint8_t part_id;
    err = max30102_register_read_with_retry(MAX30102_PART_ID_REG_ADDR, &part_id, 1, 3);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "MAX30102 device ID: 0x%02x", part_id);
    } else {
        ESP_LOGE(TAG, "Failed to read MAX30102 device ID: %s", esp_err_to_name(err));
        ESP_LOGE(TAG, "This indicates a hardware connection issue or incorrect I2C configuration");
        // 继续执行，不因为读取ID失败而停止
    }

    // reset
    err = max30102_register_write_with_retry(0x09, 0x40, 3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 reset failed: %s", esp_err_to_name(err));
        // 继续执行，不因为重置失败而停止
    } else {
        ESP_LOGI(TAG, "MAX30102 reset successful");
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    // mode configuration
    err = max30102_register_write_with_retry(0x09, 0x03, 3); // 0x03 for SpO2 mode
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 mode configuration failed: %s", esp_err_to_name(err));
        // 继续执行，不因为模式配置失败而停止
    } else {
        ESP_LOGI(TAG, "MAX30102 mode configured for SpO2");
    }

    // 设置LED亮度
    err = max30102_register_write_with_retry(0x0c, 0x10, 3); // LED1_PA(red) = 0x10, 低亮度
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 LED1 brightness configuration failed: %s", esp_err_to_name(err));
        // 继续执行，不因为亮度配置失败而停止
    }

    err = max30102_register_write_with_retry(0x0d, 0x10, 3); // LED2_PA(IR) = 0x10, 低亮度
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 LED2 brightness configuration failed: %s", esp_err_to_name(err));
        // 继续执行，不因为亮度配置失败而停止
    }

    // 设置采样率
    err = max30102_register_write_with_retry(0x0a, 0x00, 3); // 100Hz
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 sample rate configuration failed: %s", esp_err_to_name(err));
        // 继续执行，不因为采样率配置失败而停止
    } else {
        ESP_LOGI(TAG, "MAX30102 sample rate set to 100Hz");
    }

    // 设置脉冲宽度
    err = max30102_register_write_with_retry(0x0b, 0x02, 3); // 1600us
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 pulse width configuration failed: %s", esp_err_to_name(err));
        // 继续执行，不因为脉冲宽度配置失败而停止
    } else {
        ESP_LOGI(TAG, "MAX30102 pulse width set to 1600us");
    }

    // 设置ADC范围
    err = max30102_register_write_with_retry(0x0f, 0x03, 3); // 8192
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 ADC range configuration failed: %s", esp_err_to_name(err));
        // 继续执行，不因为ADC范围配置失败而停止
    } else {
        ESP_LOGI(TAG, "MAX30102 ADC range set to 8192");
    }

    // FIFO
    err = max30102_register_write_with_retry(0x04, 0x00, 3); // clear FIFO Write Pointer
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 FIFO Write Pointer configuration failed: %s", esp_err_to_name(err));
        // 继续执行，不因为FIFO配置失败而停止
    }

    err = max30102_register_write_with_retry(0x05, 0x00, 3); // clear FIFO Overflow Counter
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 FIFO Overflow Counter configuration failed: %s", esp_err_to_name(err));
        // 继续执行，不因为FIFO配置失败而停止
    }

    err = max30102_register_write_with_retry(0x06, 0x00, 3); // clear FIFO Read Pointer
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "MAX30102 FIFO Read Pointer configuration failed: %s", esp_err_to_name(err));
        // 继续执行，不因为FIFO配置失败而停止
    } else {
        ESP_LOGI(TAG, "MAX30102 FIFO configuration completed");
    }

    // clear PPG_RDY ! Cannot receive the first interrupt without clearing !
    uint8_t status_data;
    max30102_register_read(0x00, &status_data, 1);
    max30102_register_read(0x01, &status_data, 1);

    ESP_LOGI(TAG, "MAX30102 initialization completed");
}
