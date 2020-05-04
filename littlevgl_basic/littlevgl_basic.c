#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include <lvgl/lvgl.h>

#include "dwt_delay.h"
#include "lvgl_ssd1306.h"
#include "ssd1306.h"

/////////////////////////////////////////////////////////////

static volatile uint8_t task_handle_flag = 0;

// framebuffer
#define FRAMEBUFFER_WIDTH  (LV_HOR_RES_MAX)
#define FRAMEBUFFER_HEIGHT (LV_VER_RES_MAX)

static uint8_t g_framebuffer[FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT/8];
static lv_disp_buf_t g_disp_buf;
static lv_disp_t *g_disp = NULL;

// LittlevGL screens
lv_obj_t *g_pScreenTop = NULL;
lv_obj_t *g_pScreenEntry = NULL;

// LittlevGL global objects
lv_obj_t *g_label_count = NULL;
lv_obj_t *g_progress_bar = NULL;

/////////////////////////////////////////////////////////////

static void init_clock(void)
{
   rcc_clock_setup_in_hse_8mhz_out_72mhz();
}

/////////////////////////////////////////////////////////////

static void init_gpio(void)
{
   // clock for GPIO port C: USER_LED
   rcc_periph_clock_enable(RCC_GPIOC);

   // USER_LED: PC13
   gpio_set_mode(GPIOC,
      GPIO_MODE_OUTPUT_2_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      GPIO13);
}

/////////////////////////////////////////////////////////////

static void init_littlevgl(void)
{
   lv_init();

   // initialize display buffer
   memset(g_framebuffer, 0, sizeof(g_framebuffer));
   lv_disp_buf_init(&g_disp_buf, &g_framebuffer, NULL, FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT);

   // initialize display driver
   lv_disp_drv_t disp_drv;
   lv_disp_drv_init(&disp_drv);

   disp_drv.buffer = &g_disp_buf;
   disp_drv.flush_cb = lvgl_ssd1306_flush_cb;
   disp_drv.rounder_cb = lvgl_ssd1306_rounder_cb;
   disp_drv.set_px_cb = lvgl_ssd1306_set_px_cb;

   g_disp = lv_disp_drv_register(&disp_drv);
}

/////////////////////////////////////////////////////////////

static void init_littlevgl_tasks(void)
{
   // LittlevGL internal timing is driven by TIM4

   rcc_periph_clock_enable(RCC_TIM4);

   // set timer start value
   TIM_CNT(TIM4) = 1;

   // set timer prescaler. 72MHz/72 => one tick = 1us
   TIM_PSC(TIM4) = 72;

   // end timer value, an interrupt is generated
   TIM_ARR(TIM4) = 1000; // 1000 us = 1ms

   // enable interrupts and start timer
   nvic_enable_irq(NVIC_TIM4_IRQ);
   nvic_set_priority(NVIC_TIM4_IRQ, 1);
   TIM_DIER(TIM4) |= TIM_DIER_UIE;
   TIM_CR1(TIM4) |= TIM_CR1_CEN;
}

/////////////////////////////////////////////////////////////

void tim4_isr(void)
{
   static uint8_t cnt = 0;

   // for internal timing in LittlevGL, every 1ms
   lv_tick_inc(1);

   // signal main thread every 10ms
   if (++cnt == 10)
   {
      task_handle_flag = 1;
      cnt = 0;
   }

   TIM_SR(TIM4) &= ~TIM_SR_UIF;  // clear interrupt flag
}

/////////////////////////////////////////////////////////////

static void custom_bar_animator(void *obj, lv_anim_value_t value);

static void create_entry_screen(void)
{
   lv_scr_load(g_pScreenTop);

   // create screen
   g_pScreenEntry = lv_obj_create(NULL, NULL);
   lv_obj_set_size(g_pScreenEntry, LV_HOR_RES_MAX, LV_VER_RES_MAX);

   // load this screen
   lv_scr_load(g_pScreenEntry);

   // create a heading label
   lv_obj_t *label1 = lv_label_create(lv_scr_act(), NULL);
   lv_label_set_text(label1, "Tritech - STM32");
   lv_obj_align(label1, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);

   // create a global label that can be updated
   lv_style_t style_label_count;
   lv_style_copy(&style_label_count, &lv_style_plain);
   style_label_count.text.font = &lv_font_unscii_8;

   g_label_count = lv_label_create(lv_scr_act(), NULL);
   lv_label_set_style(g_label_count, LV_LABEL_STYLE_MAIN, &style_label_count);
   lv_obj_align(g_label_count, NULL, LV_ALIGN_IN_BOTTOM_MID, -12, 0);

   // create a progress bar
   lv_style_t style_bar_bg;
   lv_style_t style_bar_ind;

   lv_style_copy(&style_bar_bg, &lv_style_plain);
   style_bar_bg.body.main_color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
   style_bar_bg.body.grad_color = LV_COLOR_MAKE(0xff, 0xff, 0xff);
   style_bar_bg.body.radius = LV_RADIUS_CIRCLE;
   style_bar_bg.body.border.width = 1;
   style_bar_bg.body.radius = LV_RADIUS_CIRCLE;

   lv_style_copy(&style_bar_ind, &lv_style_plain);
   style_bar_ind.body.main_color = LV_COLOR_MAKE(0x00, 0x00, 0x00);
   style_bar_ind.body.grad_color = LV_COLOR_MAKE(0x00, 0x00, 0x00);
   style_bar_ind.body.radius = LV_RADIUS_CIRCLE;
   style_bar_ind.body.border.width = 1;
   
   g_progress_bar = lv_bar_create(lv_scr_act(), NULL);
   lv_obj_set_size(g_progress_bar, 100, 18);
   lv_bar_set_range(g_progress_bar, 0, 100);
   lv_bar_set_style(g_progress_bar, LV_BAR_STYLE_BG, &style_bar_bg);
   lv_bar_set_style(g_progress_bar, LV_BAR_STYLE_INDIC, &style_bar_ind);
   lv_obj_align(g_progress_bar, NULL, LV_ALIGN_CENTER, 0, 0);

   lv_anim_t a;
   a.var = g_progress_bar;
   a.start = 0;
   a.end = 100;
   a.exec_cb = (lv_anim_exec_xcb_t)custom_bar_animator;
   a.path_cb = lv_anim_path_linear;
   a.ready_cb = NULL;
   a.act_time = 0;
   a.time = 2000;
   a.playback = 1;
   a.playback_pause = 0;
   a.repeat = 1;
   a.repeat_pause = 0;
   lv_anim_create(&a);
}

/////////////////////////////////////////////////////////////

static void custom_bar_animator(void *obj, lv_anim_value_t value)
{
   if (obj)
   {
      lv_bar_set_value(obj, value, LV_ANIM_OFF);
   }
}

/////////////////////////////////////////////////////////////

static void redraw_screen(lv_obj_t *screen)
{
   lv_scr_load(screen);
   lv_obj_invalidate(screen);
}

/////////////////////////////////////////////////////////////

static void create_gui(void)
{
   // get first active screen
   g_pScreenTop = lv_scr_act();
   lv_obj_set_size(g_pScreenTop, LV_HOR_RES_MAX, LV_VER_RES_MAX);

   create_entry_screen();

   redraw_screen(g_pScreenEntry);
}

/////////////////////////////////////////////////////////////

int main(void)
{
   // initialize hardware
   init_clock();
   init_gpio();
   dwt_delay_init();
   ssd1306_init();

   // USER_LED: turn on
   gpio_clear(GPIOC, GPIO13);

   // initialize LittlevGL
   init_littlevgl();
   init_littlevgl_tasks();
   create_gui();

   uint8_t led_cnt = 0;
   uint8_t run_cnt = 0;
   uint32_t tot_cnt = 0;

   while (1)
   {
      // wait for signal from TIM4 ISR each 10ms
      task_handle_flag = 0;
      while (!task_handle_flag);

      // handle LittlevGL related tasks
      lv_task_handler();

      // toggle USER_LED every 250ms
      if (++led_cnt == 25)
      {
         gpio_toggle(GPIOC, GPIO13);
         led_cnt = 0;
      }

      // update global label every 100ms
      if (++run_cnt == 10)
      {
         lv_label_set_text_fmt(g_label_count, "%08u", ++tot_cnt);
         run_cnt = 0;
      }
   }

   return 0;
}
