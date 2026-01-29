#ifndef TOUCH_CST816S_H
#define TOUCH_CST816S_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

/**
 * @brief 触摸事件类型
 */
typedef enum {
    TOUCH_EVENT_NONE = 0,
    TOUCH_EVENT_PRESS,      // 按下
    TOUCH_EVENT_RELEASE,    // 释放
    TOUCH_EVENT_MOVE,       // 移动
} touch_event_t;

/**
 * @brief 触摸点数据结构
 */
typedef struct {
    uint16_t x;
    uint16_t y;
    touch_event_t event;
    bool valid;
} touch_point_t;

/**
 * @brief 初始化触摸芯片 CST816S
 * 
 * @param x_max 屏幕最大X坐标
 * @param y_max 屏幕最大Y坐标
 * @return esp_err_t 
 */
esp_err_t touch_cst816s_init(uint16_t x_max, uint16_t y_max);

/**
 * @brief 轮询读取触摸数据（非阻塞，可在LVGL任务中调用）
 * 
 * @param point 输出触摸点数据
 * @return true 有有效触摸数据
 * @return false 无触摸
 */
bool touch_cst816s_poll(touch_point_t *point);

/**
 * @brief 获取最后一次触摸状态（用于LVGL indev）
 * 
 * @param x 输出X坐标
 * @param y 输出Y坐标
 * @return true 当前被触摸
 * @return false 未触摸
 */
bool touch_cst816s_get_xy(uint16_t *x, uint16_t *y);

/**
 * @brief 检查触摸是否被按下（供LVGL使用）
 * 
 * @return true 按下状态
 * @return false 释放状态
 */
bool touch_cst816s_is_pressed(void);

#endif // TOUCH_CST816S_H