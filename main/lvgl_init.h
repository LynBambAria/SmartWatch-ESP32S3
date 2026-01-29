#ifndef LVGL_INIT_H
#define LVGL_INIT_H

#include "freertos/FreeRTOS.h"
#include "esp_lcd_panel_io.h"
#include "lvgl.h"

/* 外部声明：在 config.h 中定义的变量 */
extern esp_lcd_panel_handle_t panel_handle;
extern esp_lcd_panel_io_handle_t io_handle;
extern lv_display_t *lvgl_disp;

/**
 * @brief 初始化LCD显示
 */
esp_err_t app_lcd_init(void);

/**
 * @brief 初始化LVGL（显示 + 触摸）
 */
esp_err_t app_lvgl_init(void);

#endif // LVGL_INIT_H