#include <stdio.h>
#include <string.h>

#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>

#include <FreeRTOS.h>
#include <task.h>

#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

#include "application_freertos_prio.h"
#include "web_server.h"
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

static void task_led(__attribute__((unused))void * pvParameters)
{
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

////////////////////////////////////////////////////////////

static void test_server_tcp(void)
{
   Socket_t xServerSocket;
   Socket_t xClientSocket;
   struct freertos_sockaddr xClientAddr;
   struct freertos_sockaddr xBindAddr;
   socklen_t xSize = sizeof(xClientAddr);
   static const TickType_t xReceiveTimeOut = portMAX_DELAY;

   // create server socket
   xServerSocket = FreeRTOS_socket(FREERTOS_AF_INET, FREERTOS_SOCK_STREAM, FREERTOS_IPPROTO_TCP);

   // set a time out so accept() will just wait for a connection
   FreeRTOS_setsockopt(xServerSocket,
                       0,
                       FREERTOS_SO_RCVTIMEO,
                       &xReceiveTimeOut,
                       sizeof(xReceiveTimeOut));

   // set the listening port to 10000
   xBindAddr.sin_port = FreeRTOS_htons(10000);

   // bind the socket to the port that the client can connect
   FreeRTOS_bind(xServerSocket, &xBindAddr, sizeof(xBindAddr));

   // set the socket into a listening state so it can accept connections
   FreeRTOS_listen(xServerSocket, 1);

   // wait for incoming connection from client (host PC)
   // use the following command on host PC: netcat 192.168.100.66 10000
   xClientSocket = FreeRTOS_accept(xServerSocket, &xClientAddr, &xSize);

   // client connected
   BaseType_t lBytes, lSent, lTotalSent;
   uint8_t *pucRxBuffer = NULL;

   pucRxBuffer = (uint8_t *) pvPortMalloc(ipconfigTCP_MSS);

   if (pucRxBuffer != NULL)
   {
      for (;;)
      {
         memset(pucRxBuffer, 0x00, ipconfigTCP_MSS);

         // receive data on the socket from client
         lBytes = FreeRTOS_recv(xClientSocket, pucRxBuffer, ipconfigTCP_MSS, 0);

         // if data was received, echo it back
         if (lBytes >= 0)
         {
            lSent = 0;
            lTotalSent = 0;

            // send all the data
            while ( (lSent >= 0) && (lTotalSent < lBytes) )
            {
               lSent = FreeRTOS_send(xClientSocket, pucRxBuffer, lBytes - lTotalSent, 0);
               lTotalSent += lSent;
            }

            if (lSent < 0)
            {
               // scket closed?
               break;
            }
         }
         else
         {
            // socket closed?
            break;
         }
      }
   }

   // initiate a shutdown in case it has not already been initiated
   FreeRTOS_shutdown(xClientSocket, FREERTOS_SHUT_RDWR);

   // Wait for the shutdown to take effect, indicated by FreeRTOS_recv() returning an error
   TickType_t xTimeOnShutdown = xTaskGetTickCount();
   do
   {
      if (FreeRTOS_recv(xClientSocket, pucRxBuffer, ipconfigTCP_MSS, 0 ) < 0)
      {
         break;
      }
   } while( (xTaskGetTickCount() - xTimeOnShutdown) < pdMS_TO_TICKS(5000) ) ;

   // finished with the sockets and the buffer
   vPortFree(pucRxBuffer);
   FreeRTOS_closesocket(xClientSocket);
   FreeRTOS_closesocket(xServerSocket);
}

////////////////////////////////////////////////////////////

static void test_web_server(void)
{
   web_server_start();
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
  printf(" 4. server TCP\n");
  printf(" 5. web server\n");
  printf("\n");
}

////////////////////////////////////////////////////////////

static void task_test(__attribute__((unused))void * pvParameters)
{
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
         case 4:
            test_server_tcp();
            break;
         case 5:
            test_web_server();
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

   // Minium stack size for LED is 25 words (checked with configCHECK_FOR_STACK_OVERFLOW)
   xTaskCreate(task_led,  "LED",   25, NULL, TASK_LED_PRIO,  NULL);
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

   while (1)
   {
      ;
   }
}
#endif // configASSERT_DEFINED

/////////////////////////////////////////////////////////////

#if (configCHECK_FOR_STACK_OVERFLOW == 1 || configCHECK_FOR_STACK_OVERFLOW == 2)

// Used while developing.
// FreeRTOS will call this function when detecting a stack overflow.
// The parameters could themselves be corrupted, in which case the
// pxCurrentTCB variable can be inspected directly
// Set a breakpoint in this function using the hw-debugger.

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName);

void vApplicationStackOverflowHook(TaskHandle_t xTask,
                                   signed char *pcTaskName)
{
   (void) xTask;
   (void) pcTaskName;

   gpio_set(GPIOC, GPIO13);
   while (1)
   {
      ;
   }
}

#endif // configCHECK_FOR_STACK_OVERFLOW
