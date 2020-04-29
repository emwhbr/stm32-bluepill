LVGL_DIR =  ..

# We are using a non-standard place for configuration file lv_conf.h
LVGL_CFLAGS = -DLV_CONF_INCLUDE_SIMPLE

# Shall be included in the project Makefile
CFILES_LVGL =  $(CFILES_LVGL_CORE)
CFILES_LVGL += $(CFILES_LVGL_HAL)
CFILES_LVGL += $(CFILES_LVGL_OBJX)
CFILES_LVGL += $(CFILES_LVGL_FONT)
CFILES_LVGL += $(CFILES_LVGL_MISC)
CFILES_LVGL += $(CFILES_LVGL_THEMES)
CFILES_LVGL += $(CFILES_LVGL_DRAW)

# lv_core
CFILES_LVGL_CORE += lv_group.c
CFILES_LVGL_CORE += lv_indev.c
CFILES_LVGL_CORE += lv_disp.c
CFILES_LVGL_CORE += lv_obj.c
CFILES_LVGL_CORE += lv_refr.c
CFILES_LVGL_CORE += lv_style.c
CFILES_LVGL_CORE += lv_debug.c

VPATH += $(LVGL_DIR)/lvgl/src/lv_core

# lv_hal
CFILES_LVGL_HAL += lv_hal_disp.c
CFILES_LVGL_HAL += lv_hal_indev.c
CFILES_LVGL_HAL += lv_hal_tick.c

VPATH += :$(LVGL_DIR)/lvgl/src/lv_hal

# lv_objx
CFILES_LVGL_OBJX  += lv_arc.c
CFILES_LVGL_OBJX  += lv_bar.c
CFILES_LVGL_OBJX  += lv_cb.c
CFILES_LVGL_OBJX  += lv_cpicker.c
CFILES_LVGL_OBJX  += lv_ddlist.c
CFILES_LVGL_OBJX  += lv_kb.c
CFILES_LVGL_OBJX  += lv_line.c
CFILES_LVGL_OBJX  += lv_mbox.c
CFILES_LVGL_OBJX  += lv_preload.c
CFILES_LVGL_OBJX  += lv_roller.c
CFILES_LVGL_OBJX  += lv_table.c
CFILES_LVGL_OBJX  += lv_tabview.c
CFILES_LVGL_OBJX  += lv_tileview.c
CFILES_LVGL_OBJX  += lv_btn.c
CFILES_LVGL_OBJX  += lv_calendar.c
CFILES_LVGL_OBJX  += lv_chart.c
CFILES_LVGL_OBJX  += lv_canvas.c
CFILES_LVGL_OBJX  += lv_gauge.c
CFILES_LVGL_OBJX  += lv_label.c
CFILES_LVGL_OBJX  += lv_list.c
CFILES_LVGL_OBJX  += lv_slider.c
CFILES_LVGL_OBJX  += lv_ta.c
CFILES_LVGL_OBJX  += lv_spinbox.c
CFILES_LVGL_OBJX  += lv_btnm.c
CFILES_LVGL_OBJX  += lv_cont.c
CFILES_LVGL_OBJX  += lv_img.c
CFILES_LVGL_OBJX  += lv_imgbtn.c
CFILES_LVGL_OBJX  += lv_led.c
CFILES_LVGL_OBJX  += lv_lmeter.c
CFILES_LVGL_OBJX  += lv_page.c
CFILES_LVGL_OBJX  += lv_sw.c

VPATH += :$(LVGL_DIR)/lvgl/src/lv_objx

# lv_font
CFILES_LVGL_FONT += lv_font.c
CFILES_LVGL_FONT += lv_font_fmt_txt.c
CFILES_LVGL_FONT += lv_font_roboto_12.c
CFILES_LVGL_FONT += lv_font_roboto_16.c
CFILES_LVGL_FONT += lv_font_roboto_22.c
CFILES_LVGL_FONT += lv_font_roboto_28.c
CFILES_LVGL_FONT += lv_font_unscii_8.c

VPATH += :$(LVGL_DIR)/lvgl/src/lv_font

# lv_misc
CFILES_LVGL_MISC += lv_circ.c
CFILES_LVGL_MISC += lv_area.c
CFILES_LVGL_MISC += lv_task.c
CFILES_LVGL_MISC += lv_fs.c
CFILES_LVGL_MISC += lv_anim.c
CFILES_LVGL_MISC += lv_mem.c
CFILES_LVGL_MISC += lv_ll.c
CFILES_LVGL_MISC += lv_color.c
CFILES_LVGL_MISC += lv_txt.c
CFILES_LVGL_MISC += lv_math.c
CFILES_LVGL_MISC += lv_log.c
CFILES_LVGL_MISC += lv_gc.c
CFILES_LVGL_MISC += lv_utils.c
CFILES_LVGL_MISC += lv_async.c
CFILES_LVGL_MISC += lv_printf.c
CFILES_LVGL_MISC += lv_bidi.c

VPATH += :$(LVGL_DIR)/lvgl/src/lv_misc

# lv_themes
CFILES_LVGL_THEMES += lv_theme_alien.c
CFILES_LVGL_THEMES += lv_theme.c
CFILES_LVGL_THEMES += lv_theme_default.c
CFILES_LVGL_THEMES += lv_theme_night.c
CFILES_LVGL_THEMES += lv_theme_templ.c
CFILES_LVGL_THEMES += lv_theme_zen.c
CFILES_LVGL_THEMES += lv_theme_material.c
CFILES_LVGL_THEMES += lv_theme_nemo.c
CFILES_LVGL_THEMES += lv_theme_mono.c

VPATH += :$(LVGL_DIR)/lvgl/src/lv_themes

# lv_draw
CFILES_LVGL_DRAW += lv_draw_basic.c
CFILES_LVGL_DRAW += lv_draw.c
CFILES_LVGL_DRAW += lv_draw_rect.c
CFILES_LVGL_DRAW += lv_draw_label.c
CFILES_LVGL_DRAW += lv_draw_line.c
CFILES_LVGL_DRAW += lv_draw_img.c
CFILES_LVGL_DRAW += lv_draw_arc.c
CFILES_LVGL_DRAW += lv_draw_triangle.c
CFILES_LVGL_DRAW += lv_img_decoder.c
CFILES_LVGL_DRAW += lv_img_cache.c

VPATH += :$(LVGL_DIR)/lvgl/src/lv_draw
