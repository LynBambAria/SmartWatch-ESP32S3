set(INCLUDES "." "./ui/generated" "./ui/generated/guider_customer_fonts" "./ui/generated/guider_fonts" "./ui/custom" )

# 主源文件
set(MAIN_SOURCES
    main.c
    max30102.c
    adxl345.c
    gatts_table_creat_demo.c
    touch_cst816s.c
    lvgl_init.c
)

# UI生成的文件
file(GLOB UI_GENERATED_SOURCES
    "ui/custom/*.c"
    "ui/generated/*.c"
    "ui/generated/guider_fonts/*.c"
    "ui/generated/images/*.c"
)

# 合并所有源文件
set(SOURCES ${MAIN_SOURCES} ${UI_GENERATED_SOURCES})

idf_component_register(SRCS ${SOURCES}
                    INCLUDE_DIRS ${INCLUDES}
                    REQUIRES nvs_flash bt esp_lcd_st7789v esp_lcd_nv3030b)
