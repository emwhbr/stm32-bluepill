#include <stdint.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/timer.h>
#include "libopencm3/cm3/nvic.h"

#include <FreeRTOS.h>
#include <task.h>
#include <semphr.h>

#include <lvgl/lvgl.h>

#include "gui.h"
#include "application_freertos_prio.h"
#include "lvgl_ssd1306.h"

/////////////////////////////////////////////////////////////

// framebuffer
#define FRAMEBUFFER_WIDTH      (LV_HOR_RES_MAX)
#define FRAMEBUFFER_HEIGHT     (LV_VER_RES_MAX)
#define FRAMEBUFFER_SIZE_BYTES (FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT/8)

// Timer support (drives LittlevGL internal timing)
#define LVGL_TIMER_RCC       RCC_TIM3
#define LVGL_TIMER_RST       RST_TIM3
#define LVGL_TIMER           TIM3
#define LVGL_TIMER_IRQ       NVIC_TIM3_IRQ
#define LVGL_TIMER_IRQ_PRIO  0

// GPIO support
#define LVGL_GPIO_RCC       RCC_GPIOB
#define LVGL_GPIO_PORT      GPIOB
#define LVGL_GPIO_PIN_DBG   GPIO7  // Debug signal

#define LVGL_DBG_HI  gpio_set(LVGL_GPIO_PORT,   LVGL_GPIO_PIN_DBG)
#define LVGL_DBG_LO  gpio_clear(LVGL_GPIO_PORT, LVGL_GPIO_PIN_DBG)
#define LVGL_DBG_TG  gpio_toggle(LVGL_GPIO_PORT, LVGL_GPIO_PIN_DBG)

// internal driver data (state and synchronization)
struct gui_dev
{
   // framebuffer
   uint8_t *fb;

   // LittlevGL display driver
   lv_disp_buf_t disp_buf;
   lv_disp_t    *disp;

   // LittlevGL screens
   lv_obj_t *pScreenTop;
   lv_obj_t *pScreenEntry;

   // LittlevGL objects
   lv_obj_t *label_nec_addr;
   lv_obj_t *label_nec_cmd;
   lv_obj_t *label_nec_cmd_n;
   lv_obj_t *label_nec_count;

   // LittlevGL update task
   SemaphoreHandle_t xLVGLMtx;
   TaskHandle_t      xLVGLTask;
};

static struct gui_dev *g_dev = NULL;

// helper macros
#define LVGL_LOCK   xSemaphoreTake(g_dev->xLVGLMtx, portMAX_DELAY)
#define LVGL_UNLOCK xSemaphoreGive(g_dev->xLVGLMtx)

// internal functions
static void gui_init_gpio(void);
static void gui_init_littlevgl(void);
static void gui_init_littlevgl_timer(void);
static void gui_lvgl_task(__attribute__((unused))void * pvParameters);
static void gui_create_entry_screen(void);
static void gui_create_gui(void);

/////////////////////////////////////////////////////////////

static void gui_init_littlevgl(void)
{
   g_dev = pvPortMalloc(sizeof(struct gui_dev));

   lv_init();

   // create framebuffer
   g_dev->fb = pvPortMalloc(FRAMEBUFFER_SIZE_BYTES);

   // initialize display buffer
   memset(g_dev->fb, 0, FRAMEBUFFER_SIZE_BYTES);
   lv_disp_buf_init(&g_dev->disp_buf, g_dev->fb, NULL, FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT);

   // initialize display driver
   lv_disp_drv_t disp_drv;
   lv_disp_drv_init(&disp_drv);

   disp_drv.buffer = &g_dev->disp_buf;
   disp_drv.flush_cb = lvgl_ssd1306_flush_cb;
   disp_drv.rounder_cb = lvgl_ssd1306_rounder_cb;
   disp_drv.set_px_cb = lvgl_ssd1306_set_px_cb;

   g_dev->disp = lv_disp_drv_register(&disp_drv);

   // initialize internal timer
   gui_init_littlevgl_timer();

   // create the "thread safe" mutex
   g_dev->xLVGLMtx = xSemaphoreCreateMutex();
}

/////////////////////////////////////////////////////////////

static void gui_init_gpio(void)
{
   rcc_periph_clock_enable(LVGL_GPIO_RCC);

   gpio_set_mode(
      LVGL_GPIO_PORT,
      GPIO_MODE_OUTPUT_50_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      LVGL_GPIO_PIN_DBG);

   LVGL_DBG_HI;
}

/////////////////////////////////////////////////////////////

static void gui_init_littlevgl_timer(void)
{
   rcc_periph_clock_enable(LVGL_TIMER_RCC);
   rcc_periph_reset_pulse(LVGL_TIMER_RST);

   timer_disable_counter(LVGL_TIMER);

   timer_set_mode(LVGL_TIMER,
                  TIM_CR1_CKD_CK_INT,
                  TIM_CR1_CMS_EDGE,
                  TIM_CR1_DIR_UP);

   // we assume SYSCLK = 72MHz
   // prescaler 72 => F = 72MHz / 72 = 1MHz, T = 1us
   timer_set_prescaler(LVGL_TIMER, 72);

   // we want LittlevGL to update internal timimg every 1ms
   timer_enable_preload(LVGL_TIMER);
   timer_set_counter(LVGL_TIMER, 1);
   timer_continuous_mode(LVGL_TIMER);
   timer_set_period(LVGL_TIMER, 1000);  // 1000 * 1us => 1ms

   // generate an update event to cause shadow registers
   // to be preloaded before the timer is started 
   timer_generate_event(LVGL_TIMER, TIM_EGR_UG);
   timer_clear_flag(LVGL_TIMER, TIM_SR_UIF);
   timer_update_on_overflow(LVGL_TIMER);

   timer_enable_irq(LVGL_TIMER, TIM_DIER_UIE);
   nvic_set_priority(LVGL_TIMER_IRQ, LVGL_TIMER_IRQ_PRIO);
   nvic_enable_irq(LVGL_TIMER_IRQ);

   timer_enable_counter(LVGL_TIMER);
}

/////////////////////////////////////////////////////////////

// This is the ISR for the LittlevGL internal timer

void tim3_isr(void)
{
   if (timer_get_flag(LVGL_TIMER, TIM_SR_UIF))
   {
      timer_clear_flag(LVGL_TIMER, TIM_SR_UIF);
      lv_tick_inc(1);
   }
}

/////////////////////////////////////////////////////////////

static void gui_lvgl_task(__attribute__((unused))void * pvParameters)
{
   TickType_t xLastWakeTime = xTaskGetTickCount();

   while (1)
   {
      // handle LittlevGL related tasks (every 10ms)
      LVGL_DBG_TG;

      LVGL_LOCK;
      lv_task_handler();
      LVGL_UNLOCK;

      vTaskDelayUntil(&xLastWakeTime, pdMS_TO_TICKS(10));
   }
}

/////////////////////////////////////////////////////////////

static void gui_create_entry_screen(void)
{
   lv_scr_load(g_dev->pScreenTop);

   // create screen
   g_dev->pScreenEntry = lv_obj_create(NULL, NULL);
   lv_obj_set_size(g_dev->pScreenEntry, LV_HOR_RES_MAX, LV_VER_RES_MAX);

   // load this screen
   lv_scr_load(g_dev->pScreenEntry);

   // create a heading label
   lv_obj_t *label1 = lv_label_create(lv_scr_act(), NULL);
   lv_label_set_text(label1, "NEC IR DECODE");
   lv_obj_align(label1, NULL, LV_ALIGN_IN_TOP_MID, 0, 0);

   // create a global NEC message label that can be updated
   g_dev->label_nec_addr = lv_label_create(lv_scr_act(), NULL);
   lv_obj_align(g_dev->label_nec_addr, NULL, LV_ALIGN_CENTER, -8, -10);
   lv_label_set_text_fmt(g_dev->label_nec_addr, "0x0000");

   g_dev->label_nec_cmd = lv_label_create(lv_scr_act(), NULL);
   lv_obj_align(g_dev->label_nec_cmd, NULL, LV_ALIGN_CENTER, -20, 8);
   lv_label_set_text_fmt(g_dev->label_nec_cmd, "0x00");

   g_dev->label_nec_cmd_n = lv_label_create(lv_scr_act(), NULL);
   lv_obj_align(g_dev->label_nec_cmd_n, NULL, LV_ALIGN_CENTER, 20, 8);
   lv_label_set_text_fmt(g_dev->label_nec_cmd_n, "0x00");

   // create a global NEC counter label that can be updated
   g_dev->label_nec_count = lv_label_create(lv_scr_act(), NULL);
   lv_obj_align(g_dev->label_nec_count, NULL, LV_ALIGN_IN_BOTTOM_MID, -12, 0);
   lv_label_set_text_fmt(g_dev->label_nec_count, "00000000");
}

/////////////////////////////////////////////////////////////

static void gui_create_gui(void)
{
   // get first active screen
   g_dev->pScreenTop = lv_scr_act();
   lv_obj_set_size(g_dev->pScreenTop, LV_HOR_RES_MAX, LV_VER_RES_MAX);
   gui_create_entry_screen();
}

/////////////////////////////////////////////////////////////

void gui_init(void)
{
   // initialize LittlevGL
   gui_init_gpio();
   gui_init_littlevgl();
   gui_create_gui();

   // LittlevGL update task
   xTaskCreate(gui_lvgl_task, "LVGL", 300, NULL, TASK_LVGL_PRIO, &g_dev->xLVGLTask);
}

/////////////////////////////////////////////////////////////

void gui_set_nec_message(IR_NEC_MSG msg)
{
   LVGL_LOCK;
   lv_label_set_text_fmt(g_dev->label_nec_addr,  "0x%04x", msg.bs.addr);
   lv_label_set_text_fmt(g_dev->label_nec_cmd,   "0x%02x", msg.bs.cmd);
   lv_label_set_text_fmt(g_dev->label_nec_cmd_n, "0x%02x", msg.bs.cmd_inv);
   LVGL_UNLOCK;
}

/////////////////////////////////////////////////////////////

void gui_set_nec_count(uint32_t count)
{
   LVGL_LOCK;
   lv_label_set_text_fmt(g_dev->label_nec_count, "%08u", count);
   LVGL_UNLOCK;
}
