#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"
#include "esp_lcd_panel_io.h"
#include "esp_lcd_panel_vendor.h"
#include "esp_lcd_panel_ops.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "sdkconfig.h"

#include "esp_lvgl_port.h"
#include "lv_demos.h"
#include "iot_knob.h"

#include "touch_cst816s.h"
#include "esp_lcd_st7789v.h"

// #define GPIO_ENCODER_A 48
// #define GPIO_ENCODER_B 21
// #define KNOB_NUM    46

#define YDP169H003V3_240X280 1 //ST7789V3-4SPI
#define YDP201H001V3_240X296 2 //ST7789-MCU-8BIT
#define YDP180B001V1_240X284 3 //NV3030B_4SPI

#define CURRENT_SCREEN_SIZE YDP169H003V3_240X280 //设置成你用屏幕型号

/* LCD IO and panel */
extern esp_lcd_panel_handle_t panel_handle;
extern esp_lcd_panel_io_handle_t io_handle;

/* LVGL display and encoder */
extern lv_display_t *lvgl_disp;
// extern lv_indev_t *disp_indev;
// extern button_handle_t encoder_btn_handle;

#if CURRENT_SCREEN_SIZE == YDP169H003V3_240X280

#define LCD_HOST  SPI2_HOST
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (40 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  0
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK           11
#define EXAMPLE_PIN_NUM_MOSI           12
#define EXAMPLE_PIN_NUM_MISO           -1
#define EXAMPLE_PIN_NUM_LCD_DC         9
#define EXAMPLE_PIN_NUM_LCD_RST        13
#define EXAMPLE_PIN_NUM_LCD_CS         10
#define EXAMPLE_PIN_NUM_BK_LIGHT       1


// 用来表示命令和参数的位号
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (60)

// 水平和垂直方向的像素数
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              280

static const st7789v_lcd_init_cmd_t lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0x36, (uint8_t []){0x00}, 1, 0},//翻转C0，正常00
    {0x3A, (uint8_t []){0x05}, 1, 0},
    {0xB2, (uint8_t []){0x0B,0x0B,0x00,0x33,0x33}, 5, 0},  
    {0xB7, (uint8_t []){0x11}, 1, 0},   
    {0xBB, (uint8_t []){0x2F}, 1, 0},   
    {0xC0, (uint8_t []){0x2C}, 1, 0},   
    {0xC2, (uint8_t []){0x01}, 1, 0},   
    {0xC3, (uint8_t []){0x0D}, 1, 0},   
    {0xC4, (uint8_t []){0x20}, 1, 0},
    {0xC6, (uint8_t []){0x18}, 1, 0},
    {0xD0, (uint8_t []){0xA7,0xA1}, 2, 0},
    {0xD0, (uint8_t []){0xA4,0xA1}, 2, 0},
    {0xD6, (uint8_t []){0xA1}, 1, 0},
    {0xE0, (uint8_t []){0xF0,0x06,0x0B,0x0A,0x09,0x26,0x29,0x33,0x41,0x18,0x16,0x15,0x29,0x2D}, 14, 0},
    {0xE1, (uint8_t []){0xF0,0x04,0x08,0x08,0x07,0x03,0x28,0x32,0x40,0x3B,0x19,0x18,0x2A,0x2E}, 14, 0},
    {0xE4, (uint8_t []){0x25,0x00,0x00}, 3, 0},
    {0x35, (uint8_t []){0x00}, 1, 0},
    {0x44, (uint8_t []){0x00,0x20}, 1, 0},
    {0x21, (uint8_t []){0x00}, 0, 0},
    {0x11, (uint8_t []){0x00}, 0, 120},   
    {0x29, (uint8_t []){0x00}, 0, 0},  
};

#elif CURRENT_SCREEN_SIZE == YDP201H001V3_240X296

#include "esp_lcd_st7789v.h"

#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_LCD_RST -1
#define EXAMPLE_PIN_NUM_LCD_CS -1 //低电平
#define EXAMPLE_PIN_NUM_LCD_DC 39 //RS
#define EXAMPLE_PIN_NUM_LCD_WR 38
#define EXAMPLE_PIN_NUM_LCD_DATA0 16
#define EXAMPLE_PIN_NUM_LCD_DATA1 15
#define EXAMPLE_PIN_NUM_LCD_DATA2 14
#define EXAMPLE_PIN_NUM_LCD_DATA3 13
#define EXAMPLE_PIN_NUM_LCD_DATA4 12
#define EXAMPLE_PIN_NUM_LCD_DATA5 11
#define EXAMPLE_PIN_NUM_LCD_DATA6 10
#define EXAMPLE_PIN_NUM_LCD_DATA7 9
#define EXAMPLE_PIN_NUM_BK_LIGHT 47

// 用来表示命令和参数的位号
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (30)

// 水平和垂直方向的像素数
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              296

static const st7789v_lcd_init_cmd_t lcd_init_cmds[] = {
//  {cmd, { data }, data_size, delay_ms}
    {0x11, (uint8_t []){0x00}, 0, 120},            
    {0x36, (uint8_t []){0x00}, 1, 0},
    {0x2A, (uint8_t []){0x00,0x22,0x00,0xCD}, 4, 0},
    {0x2B, (uint8_t []){0x00,0x00,0x01,0x3F}, 4, 0},
    {0x3A, (uint8_t []){0x05}, 1, 0},    
    {0xB2, (uint8_t []){0x0C,0x0C,0x00,0x33,0x33}, 5, 0},
    {0xB7, (uint8_t []){0x00}, 1, 0},
    {0xBB, (uint8_t []){0x34}, 1, 0},
    {0xC0, (uint8_t []){0x2C}, 1, 0},
    {0xC2, (uint8_t []){0x01}, 1, 0},   
    {0xC3, (uint8_t []){0x09}, 1, 0},   
    {0xC6, (uint8_t []){0x19}, 1, 0},
    {0xD0, (uint8_t []){0xA7}, 1, 0},   
    {0xD0, (uint8_t []){0xA4,0xA1}, 2, 0},   
    {0xD6, (uint8_t []){0xA1}, 1, 0},
    {0xE0, (uint8_t []){0xF0,0x04,0x08,0x0A,0x0A,0x05,0x25,0x33,0x3C,0x24,0x0E,0x0F,0x27,0x2F}, 14, 0},
    {0xE1, (uint8_t []){0xF0,0x02,0x06,0x06,0x04,0x22,0x25,0x32,0x3B,0x3A,0x15,0x17,0x2D,0x37}, 14, 0},
    {0x21, (uint8_t []){0x00}, 0, 0}, 
    {0x29, (uint8_t []){0x00}, 0, 0}, 
};

#elif CURRENT_SCREEN_SIZE == YDP180B001V1_240X284

#include "esp_lcd_nv3030b.h"

#define LCD_HOST  SPI2_HOST
#define EXAMPLE_LCD_PIXEL_CLOCK_HZ     (40 * 1000 * 1000)
#define EXAMPLE_LCD_BK_LIGHT_ON_LEVEL  1
#define EXAMPLE_LCD_BK_LIGHT_OFF_LEVEL !EXAMPLE_LCD_BK_LIGHT_ON_LEVEL
#define EXAMPLE_PIN_NUM_SCLK           11
#define EXAMPLE_PIN_NUM_MOSI           12
#define EXAMPLE_PIN_NUM_MISO           -1
#define EXAMPLE_PIN_NUM_LCD_DC         10
#define EXAMPLE_PIN_NUM_LCD_RST        13
#define EXAMPLE_PIN_NUM_LCD_CS         9
#define EXAMPLE_PIN_NUM_BK_LIGHT       47

// 用来表示命令和参数的位号
#define EXAMPLE_LCD_CMD_BITS           8
#define EXAMPLE_LCD_PARAM_BITS         8

#define EXAMPLE_LCD_DRAW_BUFF_HEIGHT (30)

// 水平和垂直方向的像素数
#define EXAMPLE_LCD_H_RES              240
#define EXAMPLE_LCD_V_RES              284

static const nv3030b_lcd_init_cmd_t lcd_init_cmds[] = {
    //  {cmd, { data }, data_size, delay_ms}
    {0xfd, (uint8_t []){0x06,0x08}, 2, 0},
    {0x61, (uint8_t []){0x07,0x04}, 2, 0},
    {0x62, (uint8_t []){0x00,0x04,0x40}, 3, 0},
    {0x65, (uint8_t []){0x08,0x10,0x21}, 3, 0},
    {0x66, (uint8_t []){0x08,0x10,0x21}, 3, 0},
    {0x67, (uint8_t []){0x21,0x40}, 2, 0},
    {0x68, (uint8_t []){0x9F,0x30,0x27,0x21}, 4, 0},
    {0xb1, (uint8_t []){0x0f,0x02,0x01}, 3, 0},
    {0xB4, (uint8_t []){0x02}, 1, 0},
    {0xB5, (uint8_t []){0x02,0x02,0x0a,0x14}, 4, 0},
    {0xB6, (uint8_t []){0x44,0x00,0x9f,0x00,0x02}, 5, 0},
    {0xE6, (uint8_t []){0x00,0xff}, 2, 0},
    {0xE7, (uint8_t []){0x01,0x04,0x03,0x03,0x00,0x12}, 6, 0},
    {0xE8, (uint8_t []){0x00,0x70,0x00}, 3, 0},
    {0xEc, (uint8_t []){0x43}, 1, 0},
    {0xdf, (uint8_t []){0x11}, 1, 0},
    {0xe0, (uint8_t []){0x06,0x05,0x0b,0x12,0x10,0x10,0x10,0x15}, 8, 0},
    {0xe3, (uint8_t []){0x15,0x10,0x11,0x0e,0x12,0x0d,0x06,0x06}, 8, 0},
    {0xe1, (uint8_t []){0x35,0x75}, 2, 0},
    {0xe4, (uint8_t []){0x74,0x35}, 2, 0},
    {0xe2, (uint8_t []){0x22,0x22,0x21,0x35,0x36,0x3f}, 6, 0},
    {0xe5, (uint8_t []){0x3f,0x35,0x34,0x21,0x22,0x22}, 6, 0},
    {0xF1, (uint8_t []){0x01,0x01,0x02}, 3, 0},
    {0xF6, (uint8_t []){0x09,0x30}, 2, 0},
    {0xfd, (uint8_t []){0xfa,0xfc}, 2, 0},
    {0x3a, (uint8_t []){0x55}, 1, 0},
    {0x35, (uint8_t []){0x00}, 1, 0},
    {0x36, (uint8_t []){0x08}, 1, 0},
    {0x11, (uint8_t []){0x00}, 0, 200},
    {0x29, (uint8_t []){0x00}, 0, 10},
};

#else
  #error "Unsupported screen size"
#endif

// LVGL配置
#define LVGL_TASK_STACK_SIZE       8192
#define LVGL_DRAW_BUFFER_HEIGHT    60
#define LVGL_DOUBLE_BUFFER         1
#define LVGL_TASK_PRIORITY         4

// 触摸调试任务
#define TOUCH_DEBUG_TASK_STACK     4096
#define TOUCH_DEBUG_TASK_PRIORITY  2

// I2C配置
#define TOUCH_I2C_FREQ             400000