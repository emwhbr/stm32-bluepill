#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "uart.h"
#include "dwt_delay.h"
#include "ssd1306.h"


static uint8_t g_framebuffer[SSD1306_SCREEN_WIDTH * SSD1306_SCREEN_HEIGHT/8];

/////////////////////////////////////////////////////////////

static void init_clock(void)
{
   rcc_clock_setup_in_hse_8mhz_out_72mhz();

   // clock for GPIO port C: USER_LED
   rcc_periph_clock_enable(RCC_GPIOC);
}

/////////////////////////////////////////////////////////////

static void init_gpio(void)
{
   // USER_LED: PC13
   gpio_set_mode(GPIOC,
      GPIO_MODE_OUTPUT_2_MHZ,
      GPIO_CNF_OUTPUT_PUSHPULL,
      GPIO13);
}

/////////////////////////////////////////////////////////////

static void task_led(void *args)
{
   (void)args;

   while (1)
   {
      gpio_toggle(GPIOC, GPIO13);
      vTaskDelay(pdMS_TO_TICKS(250));
   }
}

/////////////////////////////////////////////////////////////

static void test_ssd1306_flush_screen(void)
{
   ssd1306_flush(g_framebuffer,
                 0, 0,
                 SSD1306_SCREEN_WIDTH - 1, SSD1306_SCREEN_HEIGHT-1);
}

/////////////////////////////////////////////////////////////

static void test_ssd1306_clear_screen(void)
{
   memset(g_framebuffer, 0, sizeof(g_framebuffer));
   test_ssd1306_flush_screen();
}

/////////////////////////////////////////////////////////////

static void test_ssd1306_invert_screen(void)
{
   static uint8_t invert = 0;
   ssd1306_invert(++invert % 2);
}

/////////////////////////////////////////////////////////////

static void test_ssd1306_set_contrast(void)
{
   char input_buf[16];
   unsigned contrast_value;
  
   printf("Enter contrast [0-255]: ");
   fflush(stdout);

   fgets(input_buf, 16, stdin);
   sscanf(input_buf, "%u", &contrast_value);

   ssd1306_set_contrast(contrast_value);
}

/////////////////////////////////////////////////////////////

static void test_ssd1306_pattern_1(void)
{
   // pattern: dots in each corner and one in the middle

   memset(g_framebuffer, 0, sizeof(g_framebuffer));

   ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, 0,                      0, SSD1306_COLOR_WHITE);
   ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, SSD1306_SCREEN_WIDTH-1, 0, SSD1306_COLOR_WHITE);
   ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, SSD1306_SCREEN_WIDTH/2, SSD1306_SCREEN_HEIGHT/2, SSD1306_COLOR_WHITE);
   ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, 0,                      SSD1306_SCREEN_HEIGHT-1, SSD1306_COLOR_WHITE);
   ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, SSD1306_SCREEN_WIDTH-1, SSD1306_SCREEN_HEIGHT-1, SSD1306_COLOR_WHITE);

   test_ssd1306_flush_screen();
}

/////////////////////////////////////////////////////////////

static void test_ssd1306_pattern_2(void)
{
   //pattern : a big 'X' on the entire screen

   memset(g_framebuffer, 0, sizeof(g_framebuffer));

   for (int y=0; y < SSD1306_SCREEN_HEIGHT; y++)
   {
      ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, y*2,                          y, SSD1306_COLOR_WHITE);
      ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, SSD1306_SCREEN_WIDTH-1 - y*2, y, SSD1306_COLOR_WHITE);
   }

   test_ssd1306_flush_screen();
}

/////////////////////////////////////////////////////////////

static void test_ssd1306_pattern_3(void)
{
   // pattern : fill upper left corner

   memset(g_framebuffer, 0, sizeof(g_framebuffer));

   for (int y=0; y < SSD1306_SCREEN_HEIGHT / 2; y++)
   {
      for (int x=0; x < SSD1306_SCREEN_WIDTH / 2; x++)
      {
         ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, x, y, SSD1306_COLOR_WHITE);
      }
   }

   test_ssd1306_flush_screen();
}

/////////////////////////////////////////////////////////////

static void test_ssd1306_pattern_4(void)
{
   // pattern : fill lower right corner

   memset(g_framebuffer, 0, sizeof(g_framebuffer));

   for (int y=SSD1306_SCREEN_HEIGHT / 2; y < SSD1306_SCREEN_HEIGHT; y++)
   {
      for (int x=SSD1306_SCREEN_WIDTH / 2; x < SSD1306_SCREEN_WIDTH; x++)
      {
         ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, x, y, SSD1306_COLOR_WHITE);
      }
   }

   test_ssd1306_flush_screen();
}

/////////////////////////////////////////////////////////////

static void test_ssd1306_pattern_5(void)
{
   // pattern : fill middle

   memset(g_framebuffer, 0, sizeof(g_framebuffer));

   for (int y=SSD1306_SCREEN_HEIGHT / 4; y < (SSD1306_SCREEN_HEIGHT * 3) / 4; y++)
   {
      for (int x=SSD1306_SCREEN_WIDTH / 4; x < (SSD1306_SCREEN_WIDTH * 3) / 4; x++)
      {
         ssd1306_set_pixel(g_framebuffer, SSD1306_SCREEN_WIDTH, x, y, SSD1306_COLOR_WHITE);
      }
   }

   test_ssd1306_flush_screen();
}

/////////////////////////////////////////////////////////////

static void print_ssd1306_menu(void)
{
  printf("\n");
  printf("-----------------------------------------\n");
  printf("--           TEST MENU SSD1306         --\n");
  printf("-----------------------------------------\n");
  printf(" 1. clear screen\n");
  printf(" 2. invert screen\n");
  printf(" 3. set contrast\n");
  printf(" 4. pattern 1 - dots\n");
  printf(" 5. pattern 2 - big X\n");
  printf(" 6. pattern 3 - upper left corner\n");
  printf(" 7. pattern 4 - lower right corner\n");
  printf(" 8. pattern 5 - middle\n");
  printf("\n");
}

/////////////////////////////////////////////////////////////

static void task_ssd1306(void *args)
{
   (void)args;

   char input_buf[16];
   int value;

   ssd1306_init();

   while (1)
   {
      print_ssd1306_menu();
      
      printf("Enter choice : ");
      fflush(stdout);

      fgets(input_buf, 16, stdin);
      sscanf(input_buf, "%d", &value);

      switch (value)
      {
         case 1:
            test_ssd1306_clear_screen();
            break;
         case 2:
            test_ssd1306_invert_screen();
            break;
         case 3:
            test_ssd1306_set_contrast();
            break;
         case 4:
            test_ssd1306_pattern_1();
            break;
         case 5:
            test_ssd1306_pattern_2();
            break;
         case 6:
            test_ssd1306_pattern_3();
            break;
         case 7:
            test_ssd1306_pattern_4();
            break;
         case 8:
            test_ssd1306_pattern_5();
            break;
         default:
            printf("*** Illegal choice : %s\n", input_buf);
      }
   }
}

/////////////////////////////////////////////////////////////

int main(void)
{
   // initialize hardware
   init_clock();
   init_gpio();
   uart_init();
   dwt_delay_init();

   printf("\nssd1306_freertos - started\n");

   xTaskCreate(task_led,     "LED",     100, NULL, configMAX_PRIORITIES-1, NULL);
   xTaskCreate(task_ssd1306, "SSD1306", 250, NULL, configMAX_PRIORITIES-2, NULL);

   vTaskStartScheduler();
   while(1)
   {
      ;
   }

   return 0;
}
