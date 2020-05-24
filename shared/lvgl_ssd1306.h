#ifndef __SHARED_LVGL_SSD1306_H__
#define __SHARED_LVGL_SSD1306_H__

#include <stdint.h>

#include <lvgl/lvgl.h>


void lvgl_ssd1306_flush_cb(struct _disp_drv_t *disp_drv, 
                           const lv_area_t *area,
                           lv_color_t *color_p);

void lvgl_ssd1306_rounder_cb(struct _disp_drv_t *disp_drv,
                             lv_area_t *area);

void lvgl_ssd1306_set_px_cb(struct _disp_drv_t *disp_drv,
                            uint8_t *buf,
                            lv_coord_t buf_w,
                            lv_coord_t x,
                            lv_coord_t y,
                            lv_color_t color,
                            lv_opa_t opa);

#endif
