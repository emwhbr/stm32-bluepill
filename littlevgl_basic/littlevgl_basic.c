#include "libopencm3/stm32/rcc.h"
#include "libopencm3/stm32/gpio.h"
#include <libopencm3/stm32/timer.h>
#include <libopencm3/cm3/nvic.h>

#include <lvgl/lvgl.h>

/////////////////////////////////////////////////////////////

static volatile uint8_t task_handle_flag = 0;

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

   lv_disp_drv_t disp_drv;
   lv_disp_drv_init(&disp_drv);
   //disp_drv.disp_flush = fbdev_flush;      //It flushes the internal graphical buffer to the frame buffer
   lv_disp_drv_register(&disp_drv);

   lv_obj_t *label = lv_label_create(lv_scr_act(), NULL);
   lv_label_set_text(label, "Hello world!");
   lv_obj_align(label, NULL, LV_ALIGN_IN_TOP_MID, 0, 5);
}

/////////////////////////////////////////////////////////////

static void init_littlevgl_tasks(void)
{
   // LittlevGL internal timing is driven by TM4

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

int main(void)
{
   // initialize hardware
   init_clock();
   init_gpio();

   // USER_LED: turn on
   gpio_clear(GPIOC, GPIO13);

   // initialize LittlevGL
   init_littlevgl();
   init_littlevgl_tasks();

   uint8_t led_cnt = 0;

   while (1)
   {
      // wait for signal from TM4 ISR each 10ms
      task_handle_flag = 0;
      while (!task_handle_flag);

      // handle LittlevGL realated tasks
      //lv_task_handler();

      // toggle USER_LED every 250ms
      if (++led_cnt == 25)
      {
         gpio_toggle(GPIOC, GPIO13);
         led_cnt = 0;
      }
   }

   return 0;
}
