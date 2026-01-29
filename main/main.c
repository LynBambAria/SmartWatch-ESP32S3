#include "lvgl_init.h"
#include "touch_cst816s.h"
#include "max30102.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_lvgl_port.h"
#include "lv_demos.h"

static const char *TAG = "MAIN";

static void app_main_display(void)
{
    lvgl_port_lock(0);
    
    /* 选择demo：keypad_encoder支持触摸+编码器 */
    // lv_demo_music();
    lv_demo_widgets();       // 小组件（支持触摸）
    // lv_demo_benchmark();     // 性能测试（支持触摸）
    lvgl_port_unlock();
}

/* 可选：调试任务 */
static void touch_debug_task(void *pvParam)
{
    touch_point_t point;
    uint32_t last_print_time = 0;
    
    while (1) {
        if (touch_cst816s_poll(&point)) {
            uint32_t now = esp_log_timestamp();
            
            // 只在有触摸事件且距离上次打印超过200ms时才打印
            if (point.valid && (now - last_print_time > 200)) {
                ESP_LOGI("TOUCH_DEBUG", "Event:%d X:%d Y:%d Valid:%d", 
                         point.event, point.x, point.y, point.valid);
                last_print_time = now;
            }
        }
        
        // 增加延时，减少轮询频率
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

static void max30102_task(void *pvParam)
{
    ESP_LOGI(TAG, "MAX30102 task starting...");
    
    // 延时一段时间，让系统其他部分先初始化完成
    vTaskDelay(pdMS_TO_TICKS(2000));
    
    // 初始化MAX30102传感器
    max30102_init();
    
    ESP_LOGI(TAG, "MAX30102 task running");
    
    // 任务主循环
    while (1) {
        // 这里可以添加定期检查传感器状态的代码
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Starting...");
    
    /* 初始化顺序很重要 */
    ESP_ERROR_CHECK(app_lcd_init());      // 1. LCD硬件
    // ESP_ERROR_CHECK(app_encoder_init());  // 2. 编码器硬件

    // 注意：触摸初始化现在在 app_lvgl_init() 中完成
    ESP_ERROR_CHECK(app_lvgl_init());     // 2. LVGL（包含显示+触摸）
    
    app_main_display();
    
    /* 启动任务 */
    xTaskCreate(touch_debug_task, "touch_dbg", 4096, NULL, 2, NULL);
    xTaskCreate(max30102_task, "max30102_task", 4096, NULL, 1, NULL);
    
    ESP_LOGI(TAG, "Running");
}