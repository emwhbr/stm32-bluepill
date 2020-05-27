#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

#include "application_freertos_prio.h"
#include "uart.h"
#include "dwt_delay.h"

/////////////////////////////////////////////////////////////

// Define the network addressing.
// These parameters will be used if either ipconfigUDE_DHCP is 0
// or if ipconfigUSE_DHCP is 1 but DHCP auto configuration failed.
static const uint8_t ucIPAddress[4]      = { 192, 168, 100, 65 };
static const uint8_t ucNetMask[4]        = { 255, 255, 255, 0 };
static const uint8_t ucGatewayAddress[4] = { 192, 168, 100, 1 };

// The gateway is the host PC used when testing
static const uint32_t ulHostPcIPAddress = 
   FreeRTOS_inet_addr_quick(ucGatewayAddress[0],
                            ucGatewayAddress[1],
                            ucGatewayAddress[2],
                            ucGatewayAddress[3]);

// The following is the address of an DNS server.
static const uint8_t ucDNSServerAddress[4] = { 192, 168, 100, 1 };

// The MAC address array is not declared const as the MAC address will normally
// be read from an EEPROM and not hard coded (in real deployed applications).
static uint8_t ucMACAddress[6] = { 0x00, 0x13, 0x3b, 0x00, 0x00, 0x01 };

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

static void test_init(void)
{
   BaseType_t rc;

   rc = FreeRTOS_IPInit(ucIPAddress,
                        ucNetMask,
                        ucGatewayAddress,
                        ucDNSServerAddress,
                        ucMACAddress);

   printf("rc=%ld\n", rc); fflush(stdout);
}

/////////////////////////////////////////////////////////////

static void test_send_ping(void)
{
   const TickType_t xBlockTime = pdMS_TO_TICKS(2000);
   uint32_t ulIPAddress;
   BaseType_t rc;

   // If a ping request is successfully sent then the
   // sequence number sent in the ping message is returned.
   //
   // The TCP/IP stack calls the application defined
   // vApplicationPingReplyHook() hook function when it
   // receives a reply to an outgoing ping request.

   rc = FreeRTOS_SendPingRequest(ulHostPcIPAddress, 4, xBlockTime);

   printf("rc=%ld\n", rc); fflush(stdout);
}

/////////////////////////////////////////////////////////////

static void test_send_udp(void)
{
   Socket_t xClientSocket;
   struct freertos_sockaddr xDestinationAddress;
   int32_t rc;

   xDestinationAddress.sin_addr = ulHostPcIPAddress;
   xDestinationAddress.sin_port = FreeRTOS_htons(8000);

   // create socket
   xClientSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_DGRAM, FREERTOS_IPPROTO_UDP);
   configASSERT( xClientSocket != FREERTOS_INVALID_SOCKET );

   // send data on socket
   char msg[] = "Tritech on the wire";
   rc = FreeRTOS_sendto(xClientSocket,
                       (void *)msg,
                       strlen((const char *)msg),
                       0, 
                       &xDestinationAddress, 
                       sizeof(xDestinationAddress));

   printf("rc=%ld\n", rc); fflush(stdout);

   // close socket
   FreeRTOS_closesocket( xClientSocket );
}

/////////////////////////////////////////////////////////////

static void print_test_menu(void)
{
  printf("\n");
  printf("heap-free: %u\n", xPortGetFreeHeapSize());
  printf("-----------------------------------\n");
  printf("--          TEST MENU            --\n");
  printf("-----------------------------------\n");
  printf(" 1. init\n");
  printf(" 2. send PING\n");
  printf(" 3. send UDP\n");
  printf("\n");
}

////////////////////////////////////////////////////////////

static void task_test(void *args)
{
   (void)args;

   char input_buf[16];
   int value;

   while (1)
   {
      print_test_menu();

      printf("Enter choice : ");
      fflush(stdout);

      fgets(input_buf, 16, stdin);
      sscanf(input_buf, "%d", &value);

      switch (value)
      {
         case 1:
            test_init();
            break;
         case 2:
            test_send_ping();
            break;
         case 3:
            test_send_udp();
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

   printf("\nalive_freertosip - started\n");

   xTaskCreate(task_led,  "LED",   50, NULL, TASK_LED_PRIO,  NULL);
   xTaskCreate(task_test, "TEST", 250, NULL, TASK_TEST_PRIO, NULL);

   printf("heap-free: %u\n", xPortGetFreeHeapSize());

   vTaskStartScheduler();
   while (1)
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
