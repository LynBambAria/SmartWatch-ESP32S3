// touch_cst816s.c
#include "touch_cst816s.h"
#include "config.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "driver/i2c.h"
#include "esp_lcd_touch_cst816s.h"
#include "esp_log.h"
#include "esp_check.h"

#define TAG "TOUCH_CST816S"

/* 引脚定义 - 从config.h导入或使用默认值 */
#ifndef CST816S_SDA
#define CST816S_SDA  5
#define CST816S_SCL  4
#define CST816S_RST  6
#define CST816S_INT  7
#endif

/* I2C 配置 */
#define CST816S_I2C_NUM     I2C_NUM_0
#define CST816S_I2C_FREQ    400000  // 降低频率到400kHz提高稳定性

/* 触摸芯片句柄 */
static esp_lcd_touch_handle_t tp_handle = NULL;
static esp_lcd_panel_io_handle_t touch_io_handle = NULL;  // 单独的触摸IO句柄

/* 触摸状态 */
static uint16_t s_last_x = 0;
static uint16_t s_last_y = 0;
static bool s_is_pressed = false;

/* 触摸事件标志和互斥锁 */
static bool s_touch_event_flag = false;
static portMUX_TYPE s_touch_mutex = portMUX_INITIALIZER_UNLOCKED;

/* 简化版本：移除防抖计数器，使用ESP-IDF的API直接获取状态 */
// static uint8_t s_release_cnt = 0;  // 注释掉这行
// #define RELEASE_THRESHOLD  3  // 注释掉这行

/**
 * @brief 触摸中断回调函数
 */
static void touch_interrupt_callback(void *arg)
{
    portENTER_CRITICAL_ISR(&s_touch_mutex);
    s_touch_event_flag = true;
    portEXIT_CRITICAL_ISR(&s_touch_mutex);
}

/**
 * @brief 硬件复位触摸芯片
 */
static void touch_hw_reset(void)
{
    ESP_LOGI(TAG, "Hardware reset touch chip");
    
    // 配置RST引脚为输出
    gpio_config_t rst_conf = {
        .pin_bit_mask = (1ULL << CST816S_RST),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&rst_conf));
    
    // 执行复位序列
    gpio_set_level(CST816S_RST, 0);
    vTaskDelay(pdMS_TO_TICKS(10));
    gpio_set_level(CST816S_RST, 1);
    vTaskDelay(pdMS_TO_TICKS(50));
    
    ESP_LOGI(TAG, "Touch hardware reset done");
}

/**
 * @brief 配置INT引脚
 */
static void touch_int_init(void)
{
    gpio_config_t int_conf = {
        .pin_bit_mask = (1ULL << CST816S_INT),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&int_conf));
    
    ESP_LOGD(TAG, "INT pin configured");
}

esp_err_t touch_cst816s_init(uint16_t x_max, uint16_t y_max)
{
    ESP_LOGI(TAG, "Initializing CST816S touch with resolution %dx%d", x_max, y_max);
    
    /* 1. 硬件复位和引脚初始化 */
    touch_hw_reset();
    touch_int_init();
    
    /* 2. I2C初始化 - 总是重新初始化以确保正确 */
    i2c_port_t i2c_num = CST816S_I2C_NUM;
    
    // 先尝试卸载（如果已安装）
    i2c_driver_delete(i2c_num);
    vTaskDelay(pdMS_TO_TICKS(10));  // 等待一小段时间
    
    ESP_LOGI(TAG, "Initializing I2C port %d...", i2c_num);
    
    i2c_config_t i2c_conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = CST816S_SDA,
        .scl_io_num = CST816S_SCL,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = CST816S_I2C_FREQ,
    };
    
    ESP_RETURN_ON_ERROR(i2c_param_config(i2c_num, &i2c_conf), 
                       TAG, "I2C parameter config failed");
    
    ESP_RETURN_ON_ERROR(i2c_driver_install(i2c_num, I2C_MODE_MASTER, 
                                         0, 0, 0), 
                       TAG, "I2C driver install failed");
    
    ESP_LOGI(TAG, "I2C initialized, SDA:%d SCL:%d Freq:%dHz", 
             CST816S_SDA, CST816S_SCL, CST816S_I2C_FREQ);
    
    /* 3. 创建触摸面板IO（单独的句柄） */
    esp_lcd_panel_io_i2c_config_t io_conf = {
        .dev_addr = ESP_LCD_TOUCH_IO_I2C_CST816S_ADDRESS,  // 通常是0x15
        .control_phase_bytes = 1,
        .dc_bit_offset = 0,
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .flags = {
            .disable_control_phase = 1,
        },
    };
    
    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_io_i2c(
        (esp_lcd_i2c_bus_handle_t)i2c_num,
        &io_conf, 
        &touch_io_handle
    ), TAG, "Create touch panel IO failed");
    
    /* 4. 创建触摸驱动 */
    esp_lcd_touch_config_t tp_cfg = {
        .x_max = x_max,
        .y_max = y_max,
        .rst_gpio_num = CST816S_RST,
        .int_gpio_num = CST816S_INT,
        .levels = {
            .reset = 0,
            .interrupt = 0,
        },
        .flags = {
            .swap_xy = 0,
            .mirror_x = 0,
            .mirror_y = 0,
        },
        .interrupt_callback = touch_interrupt_callback,
    };
    
    ESP_RETURN_ON_ERROR(esp_lcd_touch_new_i2c_cst816s(touch_io_handle, &tp_cfg, &tp_handle), 
                       TAG, "Create CST816S touch failed");
    
    ESP_LOGI(TAG, "CST816S touch initialized successfully");
    return ESP_OK;
}

bool touch_cst816s_poll(touch_point_t *point)
{
    if (point == NULL || tp_handle == NULL) {
        return false;
    }
    
    uint8_t point_cnt = 0;
    esp_lcd_touch_point_data_t touch_data[1] = {0};
    
    // 检查是否有触摸事件
    bool has_touch_event = false;
    portENTER_CRITICAL(&s_touch_mutex);
    if (s_touch_event_flag) {
        has_touch_event = true;
        s_touch_event_flag = false;
    }
    portEXIT_CRITICAL(&s_touch_mutex);
    
    // 只有在有触摸事件时才读取数据
    esp_err_t ret = ESP_OK;
    if (has_touch_event) {
        // 读取触摸数据
        esp_lcd_touch_read_data(tp_handle);
        
        // 获取触摸点数据
        ret = esp_lcd_touch_get_data(tp_handle, touch_data, &point_cnt, 1);
    } else {
        // 无触摸事件，直接返回
        if (s_is_pressed) {
            // 之前有触摸，现在释放了
            point->x = s_last_x;
            point->y = s_last_y;
            point->event = TOUCH_EVENT_RELEASE;
            point->valid = true;
            s_is_pressed = false;
            
            return true;  // 返回一次释放事件
        } else {
            // 无触摸
            point->x = 0;
            point->y = 0;
            point->event = TOUCH_EVENT_NONE;
            point->valid = false;
            
            return false;
        }
    }
    
    if (ret == ESP_OK && point_cnt > 0) {
        // 在CST816S中，如果坐标不为0，通常表示有触摸
        uint16_t x = touch_data[0].x;
        uint16_t y = touch_data[0].y;
        
        // 检查坐标是否在有效范围内
        if (x > 0 && x < EXAMPLE_LCD_H_RES && y > 0 && y < EXAMPLE_LCD_V_RES) {
            // 有触摸按下
            point->x = x;
            point->y = y;
            point->valid = true;
            
            // 事件检测
            if (!s_is_pressed) {
                point->event = TOUCH_EVENT_PRESS;
            } else if (x != s_last_x || y != s_last_y) {
                point->event = TOUCH_EVENT_MOVE;
            } else {
                point->event = TOUCH_EVENT_PRESS;  // 保持按下状态
            }
            
            s_is_pressed = true;
            s_last_x = x;
            s_last_y = y;
            
            return true;
        }
    }
    
    // 无触摸或触摸释放
    if (s_is_pressed) {
        // 之前有触摸，现在释放了
        point->x = s_last_x;
        point->y = s_last_y;
        point->event = TOUCH_EVENT_RELEASE;
        point->valid = true;
        s_is_pressed = false;
        
        return true;  // 返回一次释放事件
    } else {
        // 无触摸
        point->x = 0;
        point->y = 0;
        point->event = TOUCH_EVENT_NONE;
        point->valid = false;
        
        return false;
    }
}

bool touch_cst816s_get_xy(uint16_t *x, uint16_t *y)
{
    if (x && y) {
        *x = s_last_x;
        *y = s_last_y;
    }
    return s_is_pressed;
}

bool touch_cst816s_is_pressed(void)
{
    return s_is_pressed;
}