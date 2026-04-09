# Copyright 2026 NXP
# NXP Proprietary. This software is owned or controlled by NXP and may only be used strictly in
# accordance with the applicable license terms. By expressly accepting such terms or by downloading, installing,
# activating and/or otherwise using the software, you are agreeing that you have read, and that you agree to
# comply with and are bound by, such license terms.  If you do not agree to be bound by the applicable license
# terms, then you may not retain, install, activate or otherwise use the software.

import utime as time
import usys as sys
import lvgl as lv
import ustruct
import fs_driver

lv.init()

# Register display driver.
disp_drv = lv.sdl_window_create(240, 280)
lv.sdl_window_set_resizeable(disp_drv, False)
lv.sdl_window_set_title(disp_drv, "Simulator (MicroPython)")

# Regsiter input driver
mouse = lv.sdl_mouse_create()

# Add default theme for bottom layer
bottom_layer = lv.layer_bottom()
lv.theme_apply(bottom_layer)

fs_drv = lv.fs_drv_t()
fs_driver.fs_register(fs_drv, 'Z')

def anim_x_cb(obj, v):
    obj.set_x(v)

def anim_y_cb(obj, v):
    obj.set_y(v)

def anim_width_cb(obj, v):
    obj.set_width(v)

def anim_height_cb(obj, v):
    obj.set_height(v)

def anim_img_zoom_cb(obj, v):
    obj.set_scale(v)

def anim_img_rotate_cb(obj, v):
    obj.set_rotation(v)

global_font_cache = {}
def test_font(font_family, font_size):
    global global_font_cache
    if font_family + str(font_size) in global_font_cache:
        return global_font_cache[font_family + str(font_size)]
    if font_size % 2:
        candidates = [
            (font_family, font_size),
            (font_family, font_size-font_size%2),
            (font_family, font_size+font_size%2),
            ("montserrat", font_size-font_size%2),
            ("montserrat", font_size+font_size%2),
            ("montserrat", 16)
        ]
    else:
        candidates = [
            (font_family, font_size),
            ("montserrat", font_size),
            ("montserrat", 16)
        ]
    for (family, size) in candidates:
        try:
            if eval(f'lv.font_{family}_{size}'):
                global_font_cache[font_family + str(font_size)] = eval(f'lv.font_{family}_{size}')
                if family != font_family or size != font_size:
                    print(f'WARNING: lv.font_{family}_{size} is used!')
                return eval(f'lv.font_{family}_{size}')
        except AttributeError:
            try:
                load_font = lv.binfont_create(f"Z:MicroPython/lv_font_{family}_{size}.fnt")
                global_font_cache[font_family + str(font_size)] = load_font
                return load_font
            except:
                if family == font_family and size == font_size:
                    print(f'WARNING: lv.font_{family}_{size} is NOT supported!')

global_image_cache = {}
def load_image(file):
    global global_image_cache
    if file in global_image_cache:
        return global_image_cache[file]
    try:
        with open(file,'rb') as f:
            data = f.read()
    except:
        print(f'Could not open {file}')
        sys.exit()

    img = lv.image_dsc_t({
        'data_size': len(data),
        'data': data
    })
    global_image_cache[file] = img
    return img

def calendar_event_handler(e,obj):
    code = e.get_code()

    if code == lv.EVENT.VALUE_CHANGED:
        source = lv.calendar.__cast__(e.get_current_target())
        date = lv.calendar_date_t()
        if source.get_pressed_date(date) == lv.RESULT.OK:
            source.set_highlighted_dates([date], 1)

def spinbox_increment_event_cb(e, obj):
    code = e.get_code()
    if code == lv.EVENT.SHORT_CLICKED or code == lv.EVENT.LONG_PRESSED_REPEAT:
        obj.increment()
def spinbox_decrement_event_cb(e, obj):
    code = e.get_code()
    if code == lv.EVENT.SHORT_CLICKED or code == lv.EVENT.LONG_PRESSED_REPEAT:
        obj.decrement()

def digital_clock_cb(timer, obj, current_time, show_second, use_ampm):
    hour = int(current_time[0])
    minute = int(current_time[1])
    second = int(current_time[2])
    ampm = current_time[3]
    second = second + 1
    if second == 60:
        second = 0
        minute = minute + 1
        if minute == 60:
            minute = 0
            hour = hour + 1
            if use_ampm:
                if hour == 12:
                    if ampm == 'AM':
                        ampm = 'PM'
                    elif ampm == 'PM':
                        ampm = 'AM'
                if hour > 12:
                    hour = hour % 12
    hour = hour % 24
    if use_ampm:
        if show_second:
            obj.set_text("%d:%02d:%02d %s" %(hour, minute, second, ampm))
        else:
            obj.set_text("%d:%02d %s" %(hour, minute, ampm))
    else:
        if show_second:
            obj.set_text("%d:%02d:%02d" %(hour, minute, second))
        else:
            obj.set_text("%d:%02d" %(hour, minute))
    current_time[0] = hour
    current_time[1] = minute
    current_time[2] = second
    current_time[3] = ampm

def analog_clock_cb(timer, obj):
    datetime = time.localtime()
    hour = datetime[3]
    if hour >= 12: hour = hour - 12
    obj.set_time(hour, datetime[4], datetime[5])

def datetext_event_handler(e, obj):
    code = e.get_code()
    datetext = lv.label.__cast__(e.get_target())
    if code == lv.EVENT.FOCUSED:
        if obj is None:
            bg = lv.layer_top()
            bg.add_flag(lv.obj.FLAG.CLICKABLE)
            obj = lv.calendar(bg)
            scr = lv.screen_active()
            scr_height = scr.get_height()
            scr_width = scr.get_width()
            obj.set_size(int(scr_width * 0.8), int(scr_height * 0.8))
            datestring = datetext.get_text()
            year = int(datestring.split('/')[0])
            month = int(datestring.split('/')[1])
            day = int(datestring.split('/')[2])
            obj.set_showed_date(year, month)
            highlighted_days=[lv.calendar_date_t({'year':year, 'month':month, 'day':day})]
            obj.set_highlighted_dates(highlighted_days, 1)
            obj.align(lv.ALIGN.CENTER, 0, 0)
            lv.calendar_header_arrow(obj)
            obj.add_event_cb(lambda e: datetext_calendar_event_handler(e, datetext), lv.EVENT.ALL, None)
            scr.update_layout()

def datetext_calendar_event_handler(e, obj):
    code = e.get_code()
    calendar = lv.calendar.__cast__(e.get_current_target())
    if code == lv.EVENT.VALUE_CHANGED:
        date = lv.calendar_date_t()
        if calendar.get_pressed_date(date) == lv.RESULT.OK:
            obj.set_text(f"{date.year}/{date.month}/{date.day}")
            bg = lv.layer_top()
            bg.remove_flag(lv.obj.FLAG.CLICKABLE)
            bg.set_style_bg_opa(lv.OPA.TRANSP, 0)
            calendar.delete()

# Create screen
screen = lv.obj()
screen.set_size(240, 280)
screen.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)
# Set style for screen, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
screen.set_style_bg_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
screen.set_style_bg_color(lv.color_hex(0x000000), lv.PART.MAIN|lv.STATE.DEFAULT)
screen.set_style_bg_grad_dir(lv.GRAD_DIR.NONE, lv.PART.MAIN|lv.STATE.DEFAULT)

# Create screen_datetext_1
screen_datetext_1 = lv.label(screen)
screen_datetext_1.set_text("2024/04/22")
screen_datetext_1.set_style_text_align(lv.TEXT_ALIGN.CENTER, 0)
screen_datetext_1.add_flag(lv.obj.FLAG.CLICKABLE)
screen_datetext_1_calendar = None
screen_datetext_1.add_event_cb(lambda e: datetext_event_handler(e, screen_datetext_1_calendar), lv.EVENT.ALL, None)
screen_datetext_1.set_pos(55, 184)
screen_datetext_1.set_size(130, 36)
screen_datetext_1.remove_flag(lv.obj.FLAG.CLICKABLE)
# Set style for screen_datetext_1, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
screen_datetext_1.set_style_text_color(lv.color_hex(0xffffff), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_text_font(test_font("montserratMedium", 16), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_text_letter_space(2, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_text_align(lv.TEXT_ALIGN.CENTER, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_bg_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_bg_color(lv.color_hex(0x000000), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_bg_grad_dir(lv.GRAD_DIR.NONE, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_border_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_radius(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_pad_top(7, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_pad_right(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_pad_left(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_datetext_1.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)

# Create screen_digital_clock_1
screen_digital_clock_1_time = [int(23), int(59), int(50), ""]
screen_digital_clock_1 = lv.label(screen)
screen_digital_clock_1.set_text("23:59")
screen_digital_clock_1_timer = lv.timer_create_basic()
screen_digital_clock_1_timer.set_period(1000)
screen_digital_clock_1_timer.set_cb(lambda src: digital_clock_cb(screen_digital_clock_1_timer, screen_digital_clock_1, screen_digital_clock_1_time, False, False ))
screen_digital_clock_1.set_pos(35, 68)
screen_digital_clock_1.set_size(170, 80)
# Set style for screen_digital_clock_1, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
screen_digital_clock_1.set_style_radius(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_text_color(lv.color_hex(0xffffff), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_text_font(test_font("montserratMedium", 65), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_text_letter_space(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_text_align(lv.TEXT_ALIGN.CENTER, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_bg_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_bg_color(lv.color_hex(0x000000), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_bg_grad_dir(lv.GRAD_DIR.NONE, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_pad_top(5, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_pad_right(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_pad_bottom(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_pad_left(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_digital_clock_1.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)

# Create screen_cont_1
screen_cont_1 = lv.obj(screen)
screen_cont_1.set_pos(0, 180)
screen_cont_1.set_size(240, 100)
screen_cont_1.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)
screen_cont_1.add_flag(lv.obj.FLAG.HIDDEN)
# Set style for screen_cont_1, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
screen_cont_1.set_style_border_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_cont_1.set_style_radius(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_cont_1.set_style_bg_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_cont_1.set_style_bg_color(lv.color_hex(0x000000), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_cont_1.set_style_bg_grad_dir(lv.GRAD_DIR.NONE, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_cont_1.set_style_pad_top(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_cont_1.set_style_pad_bottom(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_cont_1.set_style_pad_left(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_cont_1.set_style_pad_right(0, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_cont_1.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
# Create screen_imgbtn_1
screen_imgbtn_1 = lv.imagebutton(screen_cont_1)
screen_imgbtn_1.add_flag(lv.obj.FLAG.CHECKABLE)
screen_imgbtn_1.set_src(lv.imagebutton.STATE.RELEASED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\heart_50_50.png"), None, None)
screen_imgbtn_1.set_src(lv.imagebutton.STATE.PRESSED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\heart_50_50.png"), None, None)
screen_imgbtn_1.set_src(lv.imagebutton.STATE.CHECKED_RELEASED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\heart_50_50.png"), None, None)
screen_imgbtn_1.set_src(lv.imagebutton.STATE.CHECKED_PRESSED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\heart_change_50_50.png"), None, None)
screen_imgbtn_1.add_flag(lv.obj.FLAG.CHECKABLE)
screen_imgbtn_1_label = lv.label(screen_imgbtn_1)
screen_imgbtn_1_label.set_text("")
screen_imgbtn_1_label.set_long_mode(lv.label.LONG.WRAP)
screen_imgbtn_1_label.set_width(lv.pct(100))
screen_imgbtn_1_label.align(lv.ALIGN.CENTER, 0, 0)
screen_imgbtn_1.set_style_pad_all(0, lv.STATE.DEFAULT)
screen_imgbtn_1.set_pos(22, 19)
screen_imgbtn_1.set_size(50, 50)
# Set style for screen_imgbtn_1, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
screen_imgbtn_1.set_style_text_color(lv.color_hex(0x000000), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_1.set_style_text_font(test_font("montserratMedium", 12), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_1.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_1.set_style_text_align(lv.TEXT_ALIGN.CENTER, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_1.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
# Set style for screen_imgbtn_1, Part: lv.PART.MAIN, State: lv.STATE.PRESSED.
screen_imgbtn_1.set_style_image_opa(255, lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_1.set_style_text_color(lv.color_hex(0xFF33FF), lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_1.set_style_text_font(test_font("montserratMedium", 12), lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_1.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_1.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.PRESSED)
# Set style for screen_imgbtn_1, Part: lv.PART.MAIN, State: lv.STATE.CHECKED.
screen_imgbtn_1.set_style_image_opa(255, lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_1.set_style_text_color(lv.color_hex(0xFF33FF), lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_1.set_style_text_font(test_font("montserratMedium", 12), lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_1.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_1.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.CHECKED)
# Set style for screen_imgbtn_1, Part: lv.PART.MAIN, State: LV_IMAGEBUTTON_STATE_RELEASED.
screen_imgbtn_1.set_style_image_opa(255, lv.PART.MAIN|lv.imagebutton.STATE.RELEASED)

# Create screen_imgbtn_2
screen_imgbtn_2 = lv.imagebutton(screen_cont_1)
screen_imgbtn_2.add_flag(lv.obj.FLAG.CHECKABLE)
screen_imgbtn_2.set_src(lv.imagebutton.STATE.RELEASED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\steps_50_50.png"), None, None)
screen_imgbtn_2.set_src(lv.imagebutton.STATE.PRESSED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\steps_50_50.png"), None, None)
screen_imgbtn_2.set_src(lv.imagebutton.STATE.CHECKED_RELEASED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\steps_50_50.png"), None, None)
screen_imgbtn_2.set_src(lv.imagebutton.STATE.CHECKED_PRESSED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\steps_change_50_50.png"), None, None)
screen_imgbtn_2.add_flag(lv.obj.FLAG.CHECKABLE)
screen_imgbtn_2_label = lv.label(screen_imgbtn_2)
screen_imgbtn_2_label.set_text("")
screen_imgbtn_2_label.set_long_mode(lv.label.LONG.WRAP)
screen_imgbtn_2_label.set_width(lv.pct(100))
screen_imgbtn_2_label.align(lv.ALIGN.CENTER, 0, 0)
screen_imgbtn_2.set_style_pad_all(0, lv.STATE.DEFAULT)
screen_imgbtn_2.set_pos(96, 20)
screen_imgbtn_2.set_size(50, 50)
# Set style for screen_imgbtn_2, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
screen_imgbtn_2.set_style_text_color(lv.color_hex(0x000000), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_2.set_style_text_font(test_font("montserratMedium", 12), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_2.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_2.set_style_text_align(lv.TEXT_ALIGN.CENTER, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_2.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
# Set style for screen_imgbtn_2, Part: lv.PART.MAIN, State: lv.STATE.PRESSED.
screen_imgbtn_2.set_style_image_opa(255, lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_2.set_style_text_color(lv.color_hex(0xFF33FF), lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_2.set_style_text_font(test_font("montserratMedium", 12), lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_2.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_2.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.PRESSED)
# Set style for screen_imgbtn_2, Part: lv.PART.MAIN, State: lv.STATE.CHECKED.
screen_imgbtn_2.set_style_image_opa(255, lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_2.set_style_text_color(lv.color_hex(0xFF33FF), lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_2.set_style_text_font(test_font("montserratMedium", 12), lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_2.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_2.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.CHECKED)
# Set style for screen_imgbtn_2, Part: lv.PART.MAIN, State: LV_IMAGEBUTTON_STATE_RELEASED.
screen_imgbtn_2.set_style_image_opa(255, lv.PART.MAIN|lv.imagebutton.STATE.RELEASED)

# Create screen_imgbtn_3
screen_imgbtn_3 = lv.imagebutton(screen_cont_1)
screen_imgbtn_3.add_flag(lv.obj.FLAG.CHECKABLE)
screen_imgbtn_3.set_src(lv.imagebutton.STATE.RELEASED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\settings_50_50.png"), None, None)
screen_imgbtn_3.set_src(lv.imagebutton.STATE.PRESSED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\settings_50_50.png"), None, None)
screen_imgbtn_3.set_src(lv.imagebutton.STATE.CHECKED_RELEASED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\settings_50_50.png"), None, None)
screen_imgbtn_3.set_src(lv.imagebutton.STATE.CHECKED_PRESSED, load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\settings_change_50_50.png"), None, None)
screen_imgbtn_3.add_flag(lv.obj.FLAG.CHECKABLE)
screen_imgbtn_3_label = lv.label(screen_imgbtn_3)
screen_imgbtn_3_label.set_text("")
screen_imgbtn_3_label.set_long_mode(lv.label.LONG.WRAP)
screen_imgbtn_3_label.set_width(lv.pct(100))
screen_imgbtn_3_label.align(lv.ALIGN.CENTER, 0, 0)
screen_imgbtn_3.set_style_pad_all(0, lv.STATE.DEFAULT)
screen_imgbtn_3.set_pos(168, 20)
screen_imgbtn_3.set_size(50, 50)
# Set style for screen_imgbtn_3, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
screen_imgbtn_3.set_style_text_color(lv.color_hex(0x000000), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_3.set_style_text_font(test_font("montserratMedium", 12), lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_3.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_3.set_style_text_align(lv.TEXT_ALIGN.CENTER, lv.PART.MAIN|lv.STATE.DEFAULT)
screen_imgbtn_3.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
# Set style for screen_imgbtn_3, Part: lv.PART.MAIN, State: lv.STATE.PRESSED.
screen_imgbtn_3.set_style_image_opa(255, lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_3.set_style_text_color(lv.color_hex(0xFF33FF), lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_3.set_style_text_font(test_font("montserratMedium", 12), lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_3.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.PRESSED)
screen_imgbtn_3.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.PRESSED)
# Set style for screen_imgbtn_3, Part: lv.PART.MAIN, State: lv.STATE.CHECKED.
screen_imgbtn_3.set_style_image_opa(255, lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_3.set_style_text_color(lv.color_hex(0xFF33FF), lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_3.set_style_text_font(test_font("montserratMedium", 12), lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_3.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.CHECKED)
screen_imgbtn_3.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.CHECKED)
# Set style for screen_imgbtn_3, Part: lv.PART.MAIN, State: LV_IMAGEBUTTON_STATE_RELEASED.
screen_imgbtn_3.set_style_image_opa(255, lv.PART.MAIN|lv.imagebutton.STATE.RELEASED)

screen.update_layout()
# Create heart
heart = lv.obj()
heart.set_size(240, 280)
heart.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)
# Set style for heart, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
heart.set_style_bg_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
heart.set_style_bg_color(lv.color_hex(0x000000), lv.PART.MAIN|lv.STATE.DEFAULT)
heart.set_style_bg_grad_dir(lv.GRAD_DIR.NONE, lv.PART.MAIN|lv.STATE.DEFAULT)

# Create heart_label_1
heart_label_1 = lv.label(heart)
heart_label_1.set_text("< BACK\n")
heart_label_1.set_long_mode(lv.label.LONG.WRAP)
heart_label_1.set_width(lv.pct(100))
heart_label_1.set_pos(6, 8)
heart_label_1.set_size(76, 20)
heart_label_1.add_flag(lv.obj.FLAG.CLICKABLE)
# Set style for heart_label_1, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
heart_label_1.set_style_border_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_radius(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_text_color(lv.color_hex(0xffffff), lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_text_font(test_font("montserratMedium", 15), lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_text_letter_space(2, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_text_line_space(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_text_align(lv.TEXT_ALIGN.CENTER, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_bg_opa(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_pad_top(3, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_pad_right(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_pad_bottom(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_pad_left(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_1.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)

# Create heart_label_2
heart_label_2 = lv.label(heart)
heart_label_2.set_text("Please place your finger on the sensor")
heart_label_2.set_long_mode(lv.label.LONG.WRAP)
heart_label_2.set_width(lv.pct(100))
heart_label_2.set_pos(40, 170)
heart_label_2.set_size(160, 50)
# Set style for heart_label_2, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
heart_label_2.set_style_border_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_radius(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_text_color(lv.color_hex(0xffffff), lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_text_font(test_font("SourceHanSerifSC_Regular", 16), lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_text_letter_space(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_text_line_space(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_text_align(lv.TEXT_ALIGN.CENTER, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_bg_opa(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_pad_top(3, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_pad_right(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_pad_bottom(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_pad_left(0, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_label_2.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)

# Create heart_bar_1
heart_bar_1 = lv.bar(heart)
heart_bar_1.set_style_anim_duration(1000, 0)
heart_bar_1.set_mode(lv.bar.MODE.NORMAL)
heart_bar_1.set_range(0, 100)
heart_bar_1.set_value(0, lv.ANIM.OFF)
heart_bar_1.set_pos(70, 150)
heart_bar_1.set_size(100, 10)
heart_bar_1.add_flag(lv.obj.FLAG.HIDDEN)
# Set style for heart_bar_1, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
heart_bar_1.set_style_bg_opa(60, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_bar_1.set_style_bg_color(lv.color_hex(0x2195f6), lv.PART.MAIN|lv.STATE.DEFAULT)
heart_bar_1.set_style_bg_grad_dir(lv.GRAD_DIR.NONE, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_bar_1.set_style_radius(10, lv.PART.MAIN|lv.STATE.DEFAULT)
heart_bar_1.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
# Set style for heart_bar_1, Part: lv.PART.INDICATOR, State: lv.STATE.DEFAULT.
heart_bar_1.set_style_bg_opa(255, lv.PART.INDICATOR|lv.STATE.DEFAULT)
heart_bar_1.set_style_bg_color(lv.color_hex(0x2195f6), lv.PART.INDICATOR|lv.STATE.DEFAULT)
heart_bar_1.set_style_bg_grad_dir(lv.GRAD_DIR.NONE, lv.PART.INDICATOR|lv.STATE.DEFAULT)
heart_bar_1.set_style_radius(10, lv.PART.INDICATOR|lv.STATE.DEFAULT)

# Create heart_img_1
heart_img_1 = lv.image(heart)
heart_img_1.set_src(load_image(r"E:\GUIGuider_project\SmartWatchUI\generated\MicroPython\check_heart_40_40.png"))
heart_img_1.add_flag(lv.obj.FLAG.CLICKABLE)
heart_img_1.set_pivot(50,50)
heart_img_1.set_rotation(0)
heart_img_1.set_pos(100, 100)
heart_img_1.set_size(40, 40)
# Set style for heart_img_1, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
heart_img_1.set_style_image_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)

heart.update_layout()
# Create step
step = lv.obj()
step.set_size(240, 280)
step.set_scrollbar_mode(lv.SCROLLBAR_MODE.OFF)
# Set style for step, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
step.set_style_bg_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
step.set_style_bg_color(lv.color_hex(0x000000), lv.PART.MAIN|lv.STATE.DEFAULT)
step.set_style_bg_grad_dir(lv.GRAD_DIR.NONE, lv.PART.MAIN|lv.STATE.DEFAULT)

# Create step_label_1
step_label_1 = lv.label(step)
step_label_1.set_text("< BACK\n")
step_label_1.set_long_mode(lv.label.LONG.WRAP)
step_label_1.set_width(lv.pct(100))
step_label_1.set_pos(6, 8)
step_label_1.set_size(76, 20)
step_label_1.add_flag(lv.obj.FLAG.CLICKABLE)
# Set style for step_label_1, Part: lv.PART.MAIN, State: lv.STATE.DEFAULT.
step_label_1.set_style_border_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_radius(0, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_text_color(lv.color_hex(0xffffff), lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_text_font(test_font("montserratMedium", 15), lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_text_opa(255, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_text_letter_space(2, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_text_line_space(0, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_text_align(lv.TEXT_ALIGN.CENTER, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_bg_opa(0, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_pad_top(3, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_pad_right(0, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_pad_bottom(0, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_pad_left(0, lv.PART.MAIN|lv.STATE.DEFAULT)
step_label_1.set_style_shadow_width(0, lv.PART.MAIN|lv.STATE.DEFAULT)

step.update_layout()

def screen_event_handler(e):
    code = e.get_code()
    indev = lv.indev_active()
    gestureDir = lv.DIR.NONE
    if indev is not None: gestureDir = indev.get_gesture_dir()
    if (code == lv.EVENT.GESTURE and lv.DIR.TOP == gestureDir):
        if indev is not None: indev.wait_release()
        pass
        screen_cont_1.remove_flag(lv.obj.FLAG.HIDDEN)
    indev = lv.indev_active()
    gestureDir = lv.DIR.NONE
    if indev is not None: gestureDir = indev.get_gesture_dir()
    if (code == lv.EVENT.GESTURE and lv.DIR.BOTTOM == gestureDir):
        if indev is not None: indev.wait_release()
        pass
        screen_cont_1.add_flag(lv.obj.FLAG.HIDDEN)
screen.add_event_cb(lambda e: screen_event_handler(e), lv.EVENT.ALL, None)

def screen_imgbtn_1_event_handler(e):
    code = e.get_code()
    if (code == lv.EVENT.CLICKED):
        pass
        lv.screen_load_anim(heart, lv.SCR_LOAD_ANIM.MOVE_TOP, 200, 200, False)
screen_imgbtn_1.add_event_cb(lambda e: screen_imgbtn_1_event_handler(e), lv.EVENT.ALL, None)

def screen_imgbtn_2_event_handler(e):
    code = e.get_code()
    if (code == lv.EVENT.CLICKED):
        pass
        lv.screen_load_anim(step, lv.SCR_LOAD_ANIM.MOVE_TOP, 200, 200, False)
screen_imgbtn_2.add_event_cb(lambda e: screen_imgbtn_2_event_handler(e), lv.EVENT.ALL, None)

def heart_event_handler(e):
    code = e.get_code()
    indev = lv.indev_active()
    gestureDir = lv.DIR.NONE
    if indev is not None: gestureDir = indev.get_gesture_dir()
    if (code == lv.EVENT.GESTURE and lv.DIR.RIGHT == gestureDir):
        if indev is not None: indev.wait_release()
        pass
        lv.screen_load_anim(screen, lv.SCR_LOAD_ANIM.MOVE_RIGHT, 200, 200, False)
heart.add_event_cb(lambda e: heart_event_handler(e), lv.EVENT.ALL, None)

def heart_label_1_event_handler(e):
    code = e.get_code()
    if (code == lv.EVENT.CLICKED):
        pass
        lv.screen_load_anim(screen, lv.SCR_LOAD_ANIM.MOVE_RIGHT, 200, 200, False)
heart_label_1.add_event_cb(lambda e: heart_label_1_event_handler(e), lv.EVENT.ALL, None)

def step_label_1_event_handler(e):
    code = e.get_code()
    if (code == lv.EVENT.CLICKED):
        pass
        lv.screen_load_anim(screen, lv.SCR_LOAD_ANIM.MOVE_LEFT, 200, 200, False)
step_label_1.add_event_cb(lambda e: step_label_1_event_handler(e), lv.EVENT.ALL, None)

# content from custom.py

# Load the default screen
lv.screen_load(screen)

if __name__ == '__main__':
    while True:
        lv.task_handler()
        time.sleep_ms(5)
