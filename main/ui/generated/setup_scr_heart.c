/*
* Copyright 2026 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "lvgl.h"
#include <stdio.h>
#include "gui_guider.h"
#include "events_init.h"
#include "widgets_init.h"
#include "custom.h"



void setup_scr_heart(lv_ui *ui)
{
    //Write codes heart
    ui->heart = lv_obj_create(NULL);
    lv_obj_set_size(ui->heart, 240, 280);
    lv_obj_set_scrollbar_mode(ui->heart, LV_SCROLLBAR_MODE_OFF);

    //Write style for heart, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->heart, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->heart, lv_color_hex(0x000000), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->heart, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes heart_label_1
    ui->heart_label_1 = lv_label_create(ui->heart);
    lv_obj_set_pos(ui->heart_label_1, 6, 8);
    lv_obj_set_size(ui->heart_label_1, 76, 20);
    lv_obj_add_flag(ui->heart_label_1, LV_OBJ_FLAG_CLICKABLE);
    lv_label_set_text(ui->heart_label_1, "< BACK\n");
    lv_label_set_long_mode(ui->heart_label_1, LV_LABEL_LONG_WRAP);

    //Write style for heart_label_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->heart_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->heart_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->heart_label_1, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->heart_label_1, &lv_font_montserratMedium_15, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->heart_label_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->heart_label_1, 2, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->heart_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->heart_label_1, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->heart_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->heart_label_1, 3, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->heart_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->heart_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->heart_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->heart_label_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes heart_label_2
    ui->heart_label_2 = lv_label_create(ui->heart);
    lv_obj_set_pos(ui->heart_label_2, 40, 170);
    lv_obj_set_size(ui->heart_label_2, 160, 50);
    lv_label_set_text(ui->heart_label_2, "Please place your finger on the sensor");
    lv_label_set_long_mode(ui->heart_label_2, LV_LABEL_LONG_WRAP);

    //Write style for heart_label_2, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_border_width(ui->heart_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->heart_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_color(ui->heart_label_2, lv_color_hex(0xffffff), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_font(ui->heart_label_2, &lv_font_SourceHanSerifSC_Regular_16, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_opa(ui->heart_label_2, 255, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_letter_space(ui->heart_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_line_space(ui->heart_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_text_align(ui->heart_label_2, LV_TEXT_ALIGN_CENTER, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_opa(ui->heart_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_top(ui->heart_label_2, 3, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_right(ui->heart_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_bottom(ui->heart_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_pad_left(ui->heart_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->heart_label_2, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write codes heart_bar_1
    ui->heart_bar_1 = lv_bar_create(ui->heart);
    lv_obj_set_pos(ui->heart_bar_1, 70, 150);
    lv_obj_set_size(ui->heart_bar_1, 100, 10);
    lv_obj_add_flag(ui->heart_bar_1, LV_OBJ_FLAG_HIDDEN);
    lv_obj_set_style_anim_duration(ui->heart_bar_1, 1000, 0);
    lv_bar_set_mode(ui->heart_bar_1, LV_BAR_MODE_NORMAL);
    lv_bar_set_range(ui->heart_bar_1, 0, 100);
    lv_bar_set_value(ui->heart_bar_1, 0, LV_ANIM_OFF);

    //Write style for heart_bar_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->heart_bar_1, 60, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->heart_bar_1, lv_color_hex(0x2195f6), LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->heart_bar_1, LV_GRAD_DIR_NONE, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->heart_bar_1, 10, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_shadow_width(ui->heart_bar_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);

    //Write style for heart_bar_1, Part: LV_PART_INDICATOR, State: LV_STATE_DEFAULT.
    lv_obj_set_style_bg_opa(ui->heart_bar_1, 255, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_color(ui->heart_bar_1, lv_color_hex(0x2195f6), LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_bg_grad_dir(ui->heart_bar_1, LV_GRAD_DIR_NONE, LV_PART_INDICATOR|LV_STATE_DEFAULT);
    lv_obj_set_style_radius(ui->heart_bar_1, 10, LV_PART_INDICATOR|LV_STATE_DEFAULT);

    //Write codes heart_img_1
    ui->heart_img_1 = lv_image_create(ui->heart);
    lv_obj_set_pos(ui->heart_img_1, 100, 100);
    lv_obj_set_size(ui->heart_img_1, 40, 40);
    lv_obj_add_flag(ui->heart_img_1, LV_OBJ_FLAG_CLICKABLE);
    lv_image_set_src(ui->heart_img_1, &_check_heart_RGB565A8_40x40);
    lv_image_set_pivot(ui->heart_img_1, 50,50);
    lv_image_set_rotation(ui->heart_img_1, 0);

    //Write style for heart_img_1, Part: LV_PART_MAIN, State: LV_STATE_DEFAULT.
    lv_obj_set_style_image_recolor_opa(ui->heart_img_1, 0, LV_PART_MAIN|LV_STATE_DEFAULT);
    lv_obj_set_style_image_opa(ui->heart_img_1, 255, LV_PART_MAIN|LV_STATE_DEFAULT);

    //The custom code of heart.


    //Update current screen layout.
    lv_obj_update_layout(ui->heart);

    //Init events for screen.
    events_init_heart(ui);
}
