/*
 * custom.c
 */

#include "lvgl.h"
#include "gui_guider.h"
#include "custom.h"
#include "max30102.h"
#include "lvgl_init.h"
#include "gatts_table_creat_demo.h"
#include <stdio.h>

// ========== 心率页面相关 ==========
extern bool sensor_active;   // 来自 max30102.c
extern uint8_t xinlv;        // 来自 max30102.c
extern bool screen_on;       // 来自 lvgl_init.c
extern TimerHandle_t screen_timer;  // 来自 lvgl_init.c
extern void screen_timer_callback(TimerHandle_t timer);  // 来自 lvgl_init.c

typedef enum {
    HEART_IDLE = 0,
    HEART_DETECTING,
    HEART_SHOW_RESULT
} heart_state_t;

static heart_state_t heart_state = HEART_IDLE;
static uint32_t detect_start_tick = 0;
static lv_timer_t *heart_timer = NULL;
static lv_ui *g_ui = NULL;  // 保存ui指针
static bool in_heart_page = false;  // 是否在心率页面

/**
 * @brief 心率页面定时器回调
 */
static void heart_page_timer_cb(lv_timer_t *timer)
{
    if(g_ui == NULL) return;
    
    char text_buf[32];
    
    switch(heart_state) {
        case HEART_IDLE:
            if(sensor_active) {
                heart_state = HEART_DETECTING;
                detect_start_tick = lv_tick_get();
                lv_label_set_text(g_ui->heart_label_2, "Please stay still");
                lv_obj_set_style_img_opa(g_ui->heart_img_1, 255, LV_PART_MAIN);
            }
            break;
            
        case HEART_DETECTING:
            if(!sensor_active) {
                heart_state = HEART_IDLE;
                lv_label_set_text(g_ui->heart_label_2, "Please place your finger on the sensor");
                lv_bar_set_value(g_ui->heart_bar_1, 0, LV_ANIM_OFF);
                lv_obj_set_style_img_opa(g_ui->heart_img_1, 80, LV_PART_MAIN);
            } else {
                uint32_t elapsed = lv_tick_get() - detect_start_tick;
                uint8_t progress = (elapsed * 100) / 5000;
                
                if(progress >= 100) {
                    progress = 100;
                    heart_state = HEART_SHOW_RESULT;
                    snprintf(text_buf, sizeof(text_buf), "Heart Rate: %d BPM", xinlv);
                    lv_label_set_text(g_ui->heart_label_2, text_buf);
                    ble_send_heart_rate(xinlv);
                }
                lv_bar_set_value(g_ui->heart_bar_1, progress, LV_ANIM_OFF);
            }
            break;
            
        case HEART_SHOW_RESULT:
            if(!sensor_active) {
                heart_state = HEART_IDLE;
                lv_label_set_text(g_ui->heart_label_2, "Please place your finger on the sensor");
                lv_bar_set_value(g_ui->heart_bar_1, 0, LV_ANIM_OFF);
                lv_obj_set_style_img_opa(g_ui->heart_img_1, 80, LV_PART_MAIN);
            } else {
                snprintf(text_buf, sizeof(text_buf), "Heart Rate: %d BPM", xinlv);
                lv_label_set_text(g_ui->heart_label_2, text_buf);
                ble_send_heart_rate(xinlv);
            }
            break;
    }
}

/**
 * @brief 暂停心率定时器
 */
void heart_timer_pause(void)
{
    if(heart_timer) {
        lv_timer_pause(heart_timer);
    }
}

/**
 * @brief 恢复心率定时器
 */
void heart_timer_resume(void)
{
    if(heart_timer) {
        lv_timer_resume(heart_timer);
    }
}

/**
 * @brief 重置屏幕定时器，根据当前页面设置不同的超时时间
 */
void reset_heart_screen_timer(void)
{
    // 先重新检测当前是否在心率页面
    if (g_ui != NULL) {
        lv_obj_t *active_screen = lv_screen_active();
        bool prev_in_heart = in_heart_page;
        
        if (active_screen == g_ui->heart) {
            in_heart_page = true;
            if (!prev_in_heart) {
                ESP_LOGI("CUSTOM", "Entered heart page, starting continuous detection");
                max30102_start_continuous_detection();
                // 立即检查一次，如果已经有手指就开始检测
                if (sensor_active) {
                    heart_state = HEART_DETECTING;
                    detect_start_tick = lv_tick_get();
                    lv_label_set_text(g_ui->heart_label_2, "Please stay still");
                    lv_obj_set_style_img_opa(g_ui->heart_img_1, 255, LV_PART_MAIN);
                    ESP_LOGI("CUSTOM", "Finger already detected, start detecting immediately");
                } else {
                    heart_state = HEART_IDLE;
                    lv_label_set_text(g_ui->heart_label_2, "Please place your finger on the sensor");
                    lv_bar_set_value(g_ui->heart_bar_1, 0, LV_ANIM_OFF);
                    lv_obj_set_style_img_opa(g_ui->heart_img_1, 80, LV_PART_MAIN);
                }
            }
            // 不管之前是什么状态，确保定时器在运行，提高更新频率到50ms让响应更快
            if (heart_timer == NULL) {
                heart_timer = lv_timer_create(heart_page_timer_cb, 50, NULL);
                ESP_LOGI("CUSTOM", "Heart timer created");
            } else {
                lv_timer_set_period(heart_timer, 50);
                lv_timer_resume(heart_timer);
                ESP_LOGI("CUSTOM", "Heart timer resumed");
            }
        } else {
            in_heart_page = false;
            if (prev_in_heart) {
                ESP_LOGI("CUSTOM", "Left heart page, stopping continuous detection");
                heart_timer_pause();
                max30102_stop_continuous_detection();
            }
        }
    }
    
    // 心率页面20秒超时，其他页面5秒超时
    const TickType_t heart_timeout = pdMS_TO_TICKS(20000);
    const TickType_t normal_timeout = pdMS_TO_TICKS(5000);
    
    ESP_LOGI("CUSTOM", "Resetting screen timer, in_heart_page=%d", in_heart_page);
    
    if (screen_timer != NULL) {
        // 停止当前定时器
        xTimerStop(screen_timer, 0);
        
        // 修改定时器周期 - 使用xTimerChangePeriod直接修改
        if (in_heart_page) {
            // 在心率页面，设置20秒超时
            xTimerChangePeriod(screen_timer, heart_timeout, 0);
            ESP_LOGI("CUSTOM", "Heart page screen timer changed to 20s timeout");
        } else {
            // 在其他页面，设置5秒超时
            xTimerChangePeriod(screen_timer, normal_timeout, 0);
            ESP_LOGI("CUSTOM", "Normal page screen timer changed to 5s timeout");
        }
        
        // 重新启动定时器
        xTimerStart(screen_timer, 0);
    } else {
        ESP_LOGW("CUSTOM", "screen_timer is NULL, cannot reset");
    }
}



/**
 * @brief 触摸事件回调 - 用于重置屏幕定时器
 */
static void touch_event_cb(lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    
    // 只处理按下、按住和点击事件
    if (code == LV_EVENT_PRESSED || code == LV_EVENT_PRESSING || code == LV_EVENT_CLICKED) {
        // 检查当前活动屏幕是否是心率页面
        lv_obj_t *active_screen = lv_screen_active();
        bool prev_in_heart = in_heart_page;
        
        if (g_ui && active_screen == g_ui->heart) {
            in_heart_page = true;
            
            // 如果刚进入心率页面，启动持续检测和恢复定时器
            if (!prev_in_heart) {
                ESP_LOGI("CUSTOM", "Entered heart page, starting continuous detection");
                // 重置状态
                heart_state = HEART_IDLE;
                lv_label_set_text(g_ui->heart_label_2, "Please place your finger on the sensor");
                lv_bar_set_value(g_ui->heart_bar_1, 0, LV_ANIM_OFF);
                lv_obj_set_style_img_opa(g_ui->heart_img_1, 80, LV_PART_MAIN);
                
                // 确保定时器运行
                if(heart_timer == NULL) {
                    heart_timer = lv_timer_create(heart_page_timer_cb, 100, NULL);
                    ESP_LOGI("CUSTOM", "Heart timer created");
                } else {
                    lv_timer_resume(heart_timer);
                    ESP_LOGI("CUSTOM", "Heart timer resumed");
                }
                
                // 启动持续检测模式
                max30102_start_continuous_detection();
            }
        } else {
            in_heart_page = false;
            
            // 如果刚离开心率页面，停止持续检测和暂停定时器
            if (prev_in_heart) {
                ESP_LOGI("CUSTOM", "Left heart page, stopping continuous detection");
                // 暂停心率定时器
                heart_timer_pause();
                // 停止持续检测模式
                max30102_stop_continuous_detection();
            }
        }
        
        // 根据当前页面重置屏幕定时器
        reset_heart_screen_timer();
        
        // 如果屏幕是关闭的，唤醒屏幕
        if (!screen_on) {
            screen_turn_on();
        }
    }
}

// ========== 原有 custom.c 内容保留 ==========

void custom_init(lv_ui *ui)
{
    // 保存ui指针供定时器使用
    g_ui = ui;
    
    // 为heart页面添加事件监听
    if(ui->heart) {
        // 所有触摸事件都处理，每次触摸都检查当前页面
        lv_obj_add_event_cb(ui->heart, touch_event_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(ui->heart, touch_event_cb, LV_EVENT_PRESSING, NULL);
        lv_obj_add_event_cb(ui->heart, touch_event_cb, LV_EVENT_CLICKED, NULL);
        
        // 设置初始状态（如果heart是初始页面）
        lv_label_set_text(ui->heart_label_2, "Please place your finger on the sensor");
        lv_bar_set_value(ui->heart_bar_1, 0, LV_ANIM_OFF);
        lv_obj_set_style_img_opa(ui->heart_img_1, 80, LV_PART_MAIN);
        
        ESP_LOGI("CUSTOM", "Heart page event listeners added");
    }
    
    // 为主屏幕添加触摸事件监听
    if(ui->screen) {
        // 触摸事件
        lv_obj_add_event_cb(ui->screen, touch_event_cb, LV_EVENT_PRESSED, NULL);
        lv_obj_add_event_cb(ui->screen, touch_event_cb, LV_EVENT_PRESSING, NULL);
    }
    
    // 启动定时器（延迟启动，确保max30102已初始化）
    if(heart_timer == NULL) {
        heart_timer = lv_timer_create(heart_page_timer_cb, 100, NULL);
        ESP_LOGI("CUSTOM", "Heart rate timer created with 100ms interval");
    }
    
    // 检查初始屏幕是否是心率页面
    lv_obj_t *active_screen = lv_screen_active();
    if(g_ui && active_screen == g_ui->heart) {
        in_heart_page = true;
        max30102_start_continuous_detection();
        ESP_LOGI("CUSTOM", "Initial screen is heart page, started continuous detection");
    } else {
        in_heart_page = false;
        ESP_LOGI("CUSTOM", "Initial screen is not heart page");
    }
    
    // 初始化屏幕定时器状态
    reset_heart_screen_timer();
}

void enter_heart_page_immediately(void)
{
    if (g_ui == NULL) return;
    
    lv_obj_t *active_screen = lv_screen_active();
    if (active_screen == g_ui->heart) {
        in_heart_page = true;
        // 立即启动持续检测，不能延迟
        max30102_start_continuous_detection();
        // 确保定时器运行，提高更新频率到50ms让响应更快
        if (heart_timer == NULL) {
            heart_timer = lv_timer_create(heart_page_timer_cb, 50, NULL);
            ESP_LOGI("CUSTOM", "Heart timer created");
        } else {
            lv_timer_set_period(heart_timer, 50);
            lv_timer_resume(heart_timer);
            ESP_LOGI("CUSTOM", "Heart timer resumed");
        }
        // 立即检查一次，如果已经有手指就开始检测
        if (sensor_active) {
            heart_state = HEART_DETECTING;
            detect_start_tick = lv_tick_get();
            lv_label_set_text(g_ui->heart_label_2, "Please stay still");
            lv_obj_set_style_img_opa(g_ui->heart_img_1, 255, LV_PART_MAIN);
            ESP_LOGI("CUSTOM", "Finger already detected, start detecting immediately");
        } else {
            heart_state = HEART_IDLE;
            lv_label_set_text(g_ui->heart_label_2, "Please place your finger on the sensor");
            lv_bar_set_value(g_ui->heart_bar_1, 0, LV_ANIM_OFF);
            lv_obj_set_style_img_opa(g_ui->heart_img_1, 80, LV_PART_MAIN);
            ESP_LOGI("CUSTOM", "No finger detected, waiting for placement");
        }
        ESP_LOGI("CUSTOM", "Enter heart page immediately, detection started");
    }
}