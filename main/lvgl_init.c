// lvgl_init.c
#include "lvgl_init.h"
#include "touch_cst816s.h"
#include "config.h"
#include "esp_lcd_st7789v.h"
#include "esp_lcd_nv3030b.h"
#include "esp_lvgl_port.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/spi_master.h"
#include "esp_lcd_panel_ops.h"
#include "esp_lcd_panel_vendor.h"
// #include "button_gpio.h"

static const char *TAG = "LVGL_INIT";

/* 全局变量定义 */
esp_lcd_panel_handle_t panel_handle = NULL;
esp_lcd_panel_io_handle_t io_handle = NULL;
lv_display_t *lvgl_disp = NULL;
static lv_indev_t *touch_indev = NULL;  // 静态变量，仅在当前文件使用
// lv_indev_t *disp_indev = NULL;
// button_handle_t encoder_btn_handle = NULL;

// /* 编码器配置定义 */
// const button_gpio_config_t encoder_btn_config = {
//     .gpio_num = KNOB_NUM,
//     .active_level = 0,
// };
// const knob_config_t encoder_a_b_config = {
//     .default_direction = 0,
//     .gpio_encoder_a = GPIO_ENCODER_A,
//     .gpio_encoder_b = GPIO_ENCODER_B,
// };

/* 静态变量 */

/* 屏幕初始化命令数组 */
#if CURRENT_SCREEN_SIZE == YDP169H003V3_240X280

extern const st7789v_lcd_init_cmd_t lcd_init_cmds[];

#elif CURRENT_SCREEN_SIZE == YDP201H001V3_240X296

extern const st7789v_lcd_init_cmd_t lcd_init_cmds[];

#elif CURRENT_SCREEN_SIZE == YDP180B001V1_240X284

extern const nv3030b_lcd_init_cmd_t lcd_init_cmds[];

#endif


/* ============================================
 * 函数实现
 * ============================================ */

esp_err_t app_lcd_init(void)
{
#if CURRENT_SCREEN_SIZE == YDP169H003V3_240X280

    ESP_LOGI(TAG, "Turn off LCD backlight");
    gpio_config_t bk_gpio_config = {
        .mode = GPIO_MODE_OUTPUT,
        .pin_bit_mask = 1ULL << EXAMPLE_PIN_NUM_BK_LIGHT
    };
    ESP_ERROR_CHECK(gpio_config(&bk_gpio_config));

    ESP_LOGI(TAG, "Initialize SPI bus");
    spi_bus_config_t buscfg = {
        .sclk_io_num = EXAMPLE_PIN_NUM_SCLK,
        .mosi_io_num = EXAMPLE_PIN_NUM_MOSI,
        .miso_io_num = EXAMPLE_PIN_NUM_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = EXAMPLE_PIN_NUM_LCD_DC,
        .cs_gpio_num = EXAMPLE_PIN_NUM_LCD_CS,
        .pclk_hz = EXAMPLE_LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = EXAMPLE_LCD_CMD_BITS,
        .lcd_param_bits = EXAMPLE_LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST, &io_config, &io_handle));

    st7789v_vendor_config_t vendor_config = {
        .init_cmds = lcd_init_cmds,
        .init_cmds_size = sizeof(lcd_init_cmds) / sizeof(st7789v_lcd_init_cmd_t),
    };

    ESP_LOGI(TAG, "Install ST7789V panel driver");
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = EXAMPLE_PIN_NUM_LCD_RST,
        .rgb_endian = LCD_RGB_ENDIAN_BGR,
        .bits_per_pixel = 16,
        .vendor_config = &vendor_config,
    };

    ESP_RETURN_ON_ERROR(esp_lcd_new_panel_st7789v(io_handle, &panel_config, &panel_handle), TAG, "LCD init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_reset(panel_handle), TAG, "LCD init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_init(panel_handle), TAG, "LCD init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_set_gap(panel_handle, 0, 20), TAG, "LCD init failed");
    ESP_RETURN_ON_ERROR(esp_lcd_panel_disp_on_off(panel_handle, true), TAG, "LCD init failed");

    ESP_LOGI(TAG, "Turn on LCD backlight");
    gpio_set_level(EXAMPLE_PIN_NUM_BK_LIGHT, EXAMPLE_LCD_BK_LIGHT_ON_LEVEL);

#elif CURRENT_SCREEN_SIZE == YDP201H001V3_240X296

    // 8-bit MCU接口初始化代码（根据你提供的代码）

#elif CURRENT_SCREEN_SIZE == YDP180B001V1_240X284

    // NV3030B初始化代码（根据你提供的代码）

#else
    #error "Unsupported screen size"
#endif

    return ESP_OK;
}

// esp_err_t app_encoder_init(void)
// {
//     ESP_LOGI(TAG, "Initializing encoder...");
    
//     // 方案A：使用最简单的GPIO配置
//     // 如果LVGL只需要GPIO状态而不是按钮句柄
//     gpio_config_t btn_gpio_cfg = {
//         .pin_bit_mask = (1ULL << KNOB_NUM),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE,
//     };
//     ESP_ERROR_CHECK(gpio_config(&btn_gpio_cfg));
    
//     // 创建编码器A和B的GPIO配置
//     gpio_config_t encoder_a_cfg = {
//         .pin_bit_mask = (1ULL << GPIO_ENCODER_A),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE,
//     };
//     ESP_ERROR_CHECK(gpio_config(&encoder_a_cfg));
    
//     gpio_config_t encoder_b_cfg = {
//         .pin_bit_mask = (1ULL << GPIO_ENCODER_B),
//         .mode = GPIO_MODE_INPUT,
//         .pull_up_en = GPIO_PULLUP_ENABLE,
//         .pull_down_en = GPIO_PULLDOWN_DISABLE,
//         .intr_type = GPIO_INTR_DISABLE,
//     };
//     ESP_ERROR_CHECK(gpio_config(&encoder_b_cfg));
    
//     ESP_LOGI(TAG, "Encoder GPIO initialized (A:%d, B:%d, BTN:%d)", 
//              GPIO_ENCODER_A, GPIO_ENCODER_B, KNOB_NUM);
    
//     // 注意：这里不需要创建 encoder_btn_handle
//     // 因为LVGL的 knob 组件可能会自己处理
//     // 暂时设置为 NULL
//     encoder_btn_handle = NULL;
    
//     return ESP_OK;
// }

/* LVGL触摸读取回调函数 */
static void lvgl_touch_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    // 使用我们的触摸API读取数据
    touch_point_t point = {0};
    
    if (touch_cst816s_poll(&point) && point.valid) {
        data->point.x = point.x;
        data->point.y = point.y;
        
        if (point.event == TOUCH_EVENT_PRESS || point.event == TOUCH_EVENT_MOVE) {
            data->state = LV_INDEV_STATE_PRESSED;
        } else {
            data->state = LV_INDEV_STATE_RELEASED;
        }
        
        ESP_LOGD(TAG, "Touch: x=%d, y=%d, event=%d", point.x, point.y, point.event);
    } else {
        data->state = LV_INDEV_STATE_RELEASED;
    }
}

// //已集成到app_lvgl_init()内
// esp_err_t app_lvgl_touch_init(void)
// {
//     /* 初始化触摸硬件 */
//     ESP_ERROR_CHECK(touch_cst816s_init(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES));
    
//     /* 创建LVGL输入设备 */
//     touch_indev = lv_indev_create();
//     if (touch_indev == NULL) {
//         ESP_LOGE(TAG, "Failed to create touch input device");
//         return ESP_FAIL;
//     }
    
//     lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
//     lv_indev_set_read_cb(touch_indev, lvgl_touch_read_cb);
//     lv_indev_set_display(touch_indev, lvgl_disp);
    
//     ESP_LOGI(TAG, "LVGL touch input device initialized");
//     return ESP_OK;
// }

esp_err_t app_lvgl_init(void)
{
    /* 初始化LVGL核心 */
    const lvgl_port_cfg_t lvgl_cfg = {
        .task_priority = 4,
        .task_stack = 4096 * 2,
        .task_affinity = -1,
        .task_max_sleep_ms = 500,
        .timer_period_ms = 5
    };
    ESP_RETURN_ON_ERROR(lvgl_port_init(&lvgl_cfg), TAG, "LVGL port initialization failed");

    /* 添加LCD显示 */
    ESP_LOGD(TAG, "Add LCD screen");
    const lvgl_port_display_cfg_t disp_cfg = {
        .io_handle = io_handle,
        .panel_handle = panel_handle,
        .buffer_size = EXAMPLE_LCD_H_RES * EXAMPLE_LCD_DRAW_BUFF_HEIGHT,
        .double_buffer = true,
        .hres = EXAMPLE_LCD_H_RES,
        .vres = EXAMPLE_LCD_V_RES,
        .monochrome = false,
#if LVGL_VERSION_MAJOR >= 9
        .color_format = LV_COLOR_FORMAT_RGB565,
#endif
        .rotation = {
            .swap_xy = false,
            .mirror_x = false,
            .mirror_y = false,
        },
        .flags = {
            .buff_dma = true,
            .buff_spiram = false,
#if LVGL_VERSION_MAJOR >= 9
            .swap_bytes = true,
#endif
        }
    };
    lvgl_disp = lvgl_port_add_disp(&disp_cfg);

    // /* 添加编码器输入设备 */
    // const lvgl_port_encoder_cfg_t encoder = {
    //     .disp = lvgl_disp,
    //     .encoder_a_b = &encoder_a_b_config,
    //     .encoder_enter = encoder_btn_handle,
    // };
    // disp_indev = lvgl_port_add_encoder(&encoder);

    /* 初始化并添加触摸输入设备 */
 /* =========== 正确的触摸初始化方式（使用lvgl_port）=========== */
    /* 初始化触摸硬件 */
    ESP_LOGI(TAG, "Initializing touch hardware...");
    esp_err_t ret = touch_cst816s_init(EXAMPLE_LCD_H_RES, EXAMPLE_LCD_V_RES);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Touch hardware init failed, continuing without touch");
    } else {
        /* 创建LVGL触摸输入设备 */
        touch_indev = lv_indev_create();
        if (touch_indev == NULL) {
            ESP_LOGW(TAG, "Failed to create touch input device");
        } else {
            lv_indev_set_type(touch_indev, LV_INDEV_TYPE_POINTER);
            lv_indev_set_read_cb(touch_indev, lvgl_touch_read_cb);
            lv_indev_set_display(touch_indev, lvgl_disp);
            ESP_LOGI(TAG, "Touch input device created successfully");
        }
    }

    ESP_LOGI(TAG, "LVGL initialization completed");
    return ESP_OK;
}