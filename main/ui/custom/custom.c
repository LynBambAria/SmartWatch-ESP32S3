/*
 * custom.c
 */

#include "lvgl.h"
#include "gui_guider.h"
#include "custom.h"
#include <stdio.h>

// ========== 心率页面相关 ==========
extern bool sensor_active;   // 来自 max30102.c
extern uint8_t xinlv;        // 来自 max30102.c

typedef enum {
    HEART_IDLE = 0,
    HEART_DETECTING,
    HEART_SHOW_RESULT
} heart_state_t;

static heart_state_t heart_state = HEART_IDLE;
static uint32_t detect_start_tick = 0;
static lv_timer_t *heart_timer = NULL;
static lv_ui *g_ui = NULL;  // 保存ui指针

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
            }
            break;
    }
}

/**
 * @brief 屏幕加载事件回调 - 检测是否进入心率页面
 */
static void screen_load_event_cb(lv_event_t *e)
{
    lv_obj_t *screen = lv_event_get_target(e);
    
    // 检查是否是心率页面被加载
    if(g_ui && screen == g_ui->heart) {
        // 重置状态
        heart_state = HEART_IDLE;
        lv_label_set_text(g_ui->heart_label_2, "Please place your finger on the sensor");
        lv_bar_set_value(g_ui->heart_bar_1, 0, LV_ANIM_OFF);
        lv_obj_set_style_img_opa(g_ui->heart_img_1, 80, LV_PART_MAIN);
        
        // 确保定时器运行
        if(heart_timer == NULL) {
            heart_timer = lv_timer_create(heart_page_timer_cb, 100, NULL);
        } else {
            lv_timer_resume(heart_timer);
        }
    }
}

// ========== 原有 custom.c 内容保留 ==========

void custom_init(lv_ui *ui)
{
    // 保存ui指针供定时器使用
    g_ui = ui;
    
    // 为所有屏幕添加加载事件监听（或者只给heart页面加）
    // 方法1：给heart页面单独添加（推荐）
    if(ui->heart) {
        lv_obj_add_event_cb(ui->heart, screen_load_event_cb, LV_EVENT_SCREEN_LOADED, NULL);
        
        // 设置初始状态（如果heart是初始页面）
        lv_label_set_text(ui->heart_label_2, "Please place your finger on the sensor");
        lv_bar_set_value(ui->heart_bar_1, 0, LV_ANIM_OFF);
        lv_obj_set_style_img_opa(ui->heart_img_1, 80, LV_PART_MAIN);
    }
    
    // 启动定时器（延迟启动，确保max30102已初始化）
    if(heart_timer == NULL) {
        heart_timer = lv_timer_create(heart_page_timer_cb, 100, NULL);
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