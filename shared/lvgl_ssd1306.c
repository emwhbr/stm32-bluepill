#include "lvgl_ssd1306.h"
#include "ssd1306.h"

/////////////////////////////////////////////////////////////

void lvgl_ssd1306_flush_cb(struct _disp_drv_t *disp_drv, 
                           const lv_area_t *area,
                           lv_color_t *color_p)
{
   ssd1306_flush((const uint8_t *)color_p,
                 area->x1, area->y1,
                 area->x2, area->y2);

   // Inform LittlevGL that you are ready with the flushing
   lv_disp_flush_ready(disp_drv);
}

/////////////////////////////////////////////////////////////

void lvgl_ssd1306_rounder_cb(struct _disp_drv_t *disp_drv __attribute__((unused)),
                             lv_area_t *area)
{
   ssd1306_rounder((struct ssd1306_area_t *)area);
}

/////////////////////////////////////////////////////////////

void lvgl_ssd1306_set_px_cb(struct _disp_drv_t *disp_drv __attribute__((unused)),
                            uint8_t *buf,
                            lv_coord_t buf_w,
                            lv_coord_t x,
                            lv_coord_t y,
                            lv_color_t color,
                            lv_opa_t opa __attribute__((unused)))
{
   ssd1306_set_pixel(buf, buf_w, x, y, color.full ? SSD1306_COLOR_BLACK : SSD1306_COLOR_WHITE);
}
