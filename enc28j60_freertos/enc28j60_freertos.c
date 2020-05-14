#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include "uart.h"
#include "dwt_delay.h"
#include "enc28j60.h"
#include "ip_defs.h"

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
      vTaskDelay(pdMS_TO_TICKS(1000));
   }
}

/////////////////////////////////////////////////////////////

static void test_enc28j60_init(void)
{
   enc28j60_init();
}

/////////////////////////////////////////////////////////////

static void test_create_packet(ETH_PACKET *p)
{
   // Ethernet

   // 00:13:3b:00:00:0b
   p->eth_h.daddr[0] = 0x00;
   p->eth_h.daddr[1] = 0x13;
   p->eth_h.daddr[2] = 0x3b;
   p->eth_h.daddr[3] = 0x00;
   p->eth_h.daddr[4] = 0x00;
   p->eth_h.daddr[5] = 0x0b;

   // 00:13:3b:00:00:01
   p->eth_h.saddr[0] = 0x00;
   p->eth_h.saddr[1] = 0x13;
   p->eth_h.saddr[2] = 0x3b;
   p->eth_h.saddr[3] = 0x00;
   p->eth_h.saddr[4] = 0x00;
   p->eth_h.saddr[5] = 0x01;

   p->eth_h.type = htons(ETH_TYPE_IP);

   // IP
   p->ipv4_h.version     = 4;
   p->ipv4_h.ihl         = sizeof(IPV4_HEADER) / 4;
   p->ipv4_h.tos         = 0;
   p->ipv4_h.len         = htons(sizeof(IPV4_HEADER) + sizeof(UDP_DGRAM));
   p->ipv4_h.id          = 0;
   p->ipv4_h.flags       = 0;
   p->ipv4_h.frag_offset = 0;
   p->ipv4_h.ttl         = 32;
   p->ipv4_h.proto       = IP_PROTO_UDP;
   p->ipv4_h.chksum      = 0;
   p->ipv4_h.saddr       = htonl(TO_IPV4_ADDR(192, 168, 100, 65));
   p->ipv4_h.daddr       = htonl(TO_IPV4_ADDR(192, 168, 100, 1));

   // UDP
   p->udp_dgram.udp_h.sport  = htons(8000);
   p->udp_dgram.udp_h.dport  = htons(8001);
   p->udp_dgram.udp_h.len    = htons(sizeof(UDP_DGRAM));
   p->udp_dgram.udp_h.chksum = 0;

   p->udp_dgram.udp_data = htonl(0x1122aabb);
}

/////////////////////////////////////////////////////////////

static void test_enc28j60_test_send_packet_1(void)
{
   ETH_PACKET *packet = pvPortMalloc(sizeof(ETH_PACKET));

   test_create_packet(packet);
   enc28j60_test_send_packet((const uint8_t *)packet, sizeof(ETH_PACKET));

   vPortFree(packet);
}

/////////////////////////////////////////////////////////////

static void test_enc28j60_test_send_packet_n(void)
{
   char input_buf[16];
   unsigned transfers;
  
   printf("Enter transfers [0-N]: ");
   fflush(stdout);

   fgets(input_buf, 16, stdin);
   sscanf(input_buf, "%u", &transfers);

   ETH_PACKET *packet = pvPortMalloc(sizeof(ETH_PACKET));

   test_create_packet(packet);
   for (unsigned i=0; i < transfers; i++)
   {
      packet->udp_dgram.udp_data = htonl(0xabcd0000 + i);
      enc28j60_test_send_packet((const uint8_t *)packet, sizeof(ETH_PACKET));
   }

   vPortFree(packet);
}

/////////////////////////////////////////////////////////////

static void print_enc28j60_menu(void)
{
  printf("\n");
  printf("-----------------------------------------\n");
  printf("--          TEST MENU ENC28J60         --\n");
  printf("-----------------------------------------\n");
  printf(" 1. init\n");
  printf(" 2. (test)send packet x1\n");
  printf(" 3. (test)send packet xN\n");
  printf("\n");
}

/////////////////////////////////////////////////////////////

static void task_enc28j60(void *args)
{
   (void)args;

   char input_buf[16];
   int value;

   while (1)
   {
      print_enc28j60_menu();
      
      printf("Enter choice : ");
      fflush(stdout);

      fgets(input_buf, 16, stdin);
      sscanf(input_buf, "%d", &value);

      switch (value)
      {
         case 1:
            test_enc28j60_init();
            break;
         case 2:
            test_enc28j60_test_send_packet_1();
            break;
         case 3:
            test_enc28j60_test_send_packet_n();
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

   printf("\nenc28j60_freertos - started\n");

   xTaskCreate(task_led,      "LED",      100, NULL, configMAX_PRIORITIES-1, NULL);
   xTaskCreate(task_enc28j60, "ENC28J60", 250, NULL, configMAX_PRIORITIES-2, NULL);

   printf("heap-free-0: %u\n", xPortGetFreeHeapSize());

   vTaskStartScheduler();
   while(1)
   {
      ;
   }

   return 0;
}

/////////////////////////////////////////////////////////////

#if (configASSERT_DEFINED == 1)

void vAssertCalled(unsigned long ulLine, const char * const pcFileName)
{
   taskENTER_CRITICAL();
   {
      printf("*** ASSERT => %s:%lu\n", pcFileName, ulLine);
      fflush(stdout);
   }
   taskEXIT_CRITICAL();

   while(1)
   {
      ;
   }
}
#endif // configASSERT_DEFINED
