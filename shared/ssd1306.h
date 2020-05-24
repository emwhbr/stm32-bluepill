#ifndef __SHARED_SSD1306_H__
#define __SHARED_SSD1306_H__

#include <stdint.h>

#define SSD1306_SCREEN_WIDTH   128
#define SSD1306_SCREEN_HEIGHT   64

typedef enum
{
   SSD1306_COLOR_BLACK = 0,
   SSD1306_COLOR_WHITE = 1,
   SSD1306_COLOR_SWAP  = 2
} SSD1306_COLOR;

struct ssd1306_area_t
{
   uint16_t x1;
   uint16_t y1;
   uint16_t x2;
   uint16_t y2;
};

void ssd1306_init(void);

void ssd1306_set_contrast(uint8_t val);

void ssd1306_invert(uint8_t val);

void ssd1306_set_pixel(uint8_t *buf, uint16_t buf_w, uint16_t x, uint16_t y, SSD1306_COLOR color);

SSD1306_COLOR ssd1306_get_pixel(const uint8_t *buf, uint16_t buf_w, uint16_t x, uint16_t y);

void ssd1306_flush(const uint8_t *buf,
                   uint16_t x1, uint16_t y1,
                   uint16_t x2, uint16_t y2);

void ssd1306_rounder(struct ssd1306_area_t *area);

#endif
