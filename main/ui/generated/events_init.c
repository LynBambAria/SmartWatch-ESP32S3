/*
* Copyright 2026 NXP
* NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
* accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
* activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
* comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
* terms, then you may not retain, install, activate or otherwise use the software.
*/

#include "events_init.h"
#include <stdio.h>
#include "lvgl.h"
#include "custom.h"

#if LV_USE_GUIDER_SIMULATOR && LV_USE_FREEMASTER
#include "freemaster_client.h"
#endif

static void reset_timer_cb(lv_timer_t *timer)
{
    reset_heart_screen_timer();
    lv_timer_delete(timer);
}

static void screen_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_GESTURE:
    {
        lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
        switch(dir) {
        case LV_DIR_TOP:
        {
            lv_indev_wait_release(lv_indev_active());
            lv_obj_remove_flag(guider_ui.screen_cont_1, LV_OBJ_FLAG_HIDDEN);
            break;
        }
        case LV_DIR_BOTTOM:
        {
            lv_indev_wait_release(lv_indev_active());
            if (lv_obj_has_flag(guider_ui.screen_cont_1, LV_OBJ_FLAG_HIDDEN)) {

            } else {
                lv_obj_add_flag(guider_ui.screen_cont_1, LV_OBJ_FLAG_HIDDEN);
            }
            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void screen_imgbtn_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        ui_load_scr_animation(&guider_ui, &guider_ui.heart, guider_ui.heart_del, &guider_ui.screen_del, setup_scr_heart, LV_SCR_LOAD_ANIM_MOVE_TOP, 200, 200, true, false);
        enter_heart_page_immediately();
        lv_timer_create(reset_timer_cb, 250, NULL);
        break;
    }
    default:
        break;
    }
}

void events_init_screen (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->screen, screen_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->screen_imgbtn_1, screen_imgbtn_1_event_handler, LV_EVENT_ALL, ui);
}

static void heart_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_GESTURE:
    {
        lv_dir_t dir = lv_indev_get_gesture_dir(lv_indev_active());
        switch(dir) {
        case LV_DIR_RIGHT:
        {
            lv_indev_wait_release(lv_indev_active());
            ui_load_scr_animation(&guider_ui, &guider_ui.screen, guider_ui.screen_del, &guider_ui.heart_del, setup_scr_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 200, true, true);
            enter_heart_page_immediately();
            lv_timer_create(reset_timer_cb, 250, NULL);
            break;
        }
        default:
            break;
        }
        break;
    }
    default:
        break;
    }
}

static void heart_label_1_event_handler (lv_event_t *e)
{
    lv_event_code_t code = lv_event_get_code(e);
    switch (code) {
    case LV_EVENT_CLICKED:
    {
        ui_load_scr_animation(&guider_ui, &guider_ui.screen, guider_ui.screen_del, &guider_ui.heart_del, setup_scr_screen, LV_SCR_LOAD_ANIM_MOVE_RIGHT, 200, 200, true, true);
        // 从heart页面返回主页面，不需要立即进入
        lv_timer_create(reset_timer_cb, 250, NULL);
        break;
    }
    default:
        break;
    }
}

void events_init_heart (lv_ui *ui)
{
    lv_obj_add_event_cb(ui->heart, heart_event_handler, LV_EVENT_ALL, ui);
    lv_obj_add_event_cb(ui->heart_label_1, heart_label_1_event_handler, LV_EVENT_ALL, ui);
}


void events_init(lv_ui *ui)
{

}
