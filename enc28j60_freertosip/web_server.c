#include <stdio.h>
#include <string.h>

#include <FreeRTOS.h>
#include <task.h>

#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

#include "web_server.h"
#include "application_freertos_prio.h"

/////////////////////////////////////////////////////////////

// HTML page to be sent to the client
const char HTTP_Header[] =
"HTTP/1.1 200 OK\n\
Content-Type: text/html\n\
Connection: close\n\
\n\n";

const char HTTP_WebPage[] =
"<!DOCTYPE html>\
<html lang=\"en\">\
<head>\
<meta http-equiv=\"refresh\" content=\"5\">\
<meta charset=\"utf-8\">\
<title>Tritech</title>\
</head>\
<body>\
<h1>STM32 Bluepill</h1>\
<p>Welcome to the embedded world!</p>\
</body>\
</html>\
";

// The task that runs the web server
static TaskHandle_t g_xServerTask = NULL;

/////////////////////////////////////////////////////////////

static void web_server_send_all(Socket_t xClientSocket,
                                const char *pcMsg,
                                size_t len)
{
   BaseType_t lBytes, lSent, lTotalSent;

   lBytes = len;

   if (lBytes >= 0 && pcMsg != NULL)
   {
      lSent = 0;
      lTotalSent = 0;

      // send all the data
      while ( (lSent >= 0) && (lTotalSent < lBytes) )
      {
         lSent = FreeRTOS_send(xClientSocket, pcMsg, lBytes - lTotalSent, 0);
         lTotalSent += lSent;
      }
   }
}

/////////////////////////////////////////////////////////////

static void web_server_handle_client(Socket_t xClientSocket)
{
   char HTTP_Msg[60];
   char *pcRxBuffer = (char *) pvPortMalloc(ipconfigTCP_MSS);

   if (pcRxBuffer != NULL)
   {
      FreeRTOS_recv(xClientSocket, pcRxBuffer, ipconfigTCP_MSS, 0);

      web_server_send_all(xClientSocket, HTTP_Header, sizeof(HTTP_Header) - 1);
      web_server_send_all(xClientSocket, HTTP_WebPage, sizeof(HTTP_WebPage) - 1);

      TickType_t xTimeNow = xTaskGetTickCount();
      memset(HTTP_Msg, 0, sizeof(HTTP_Msg));
      snprintf(HTTP_Msg, 50, "Tick=%lu, Sec=%lu\n\n", xTimeNow, xTimeNow / configTICK_RATE_HZ);
      web_server_send_all(xClientSocket, HTTP_Msg, strlen(HTTP_Msg));

      FreeRTOS_shutdown(xClientSocket, FREERTOS_SHUT_RDWR);

      // Wait for the shutdown to take effect, indicated by FreeRTOS_recv() returning an error
      TickType_t xTimeOnShutdown = xTaskGetTickCount();
      do
      {
         if (FreeRTOS_recv(xClientSocket, pcRxBuffer, ipconfigTCP_MSS, 0) < 0)
         {
            break;
         }
      } while( (xTaskGetTickCount() - xTimeOnShutdown) < pdMS_TO_TICKS(5000) ) ;

      vPortFree(pcRxBuffer);
   }
}

/////////////////////////////////////////////////////////////

static void web_server_task(__attribute__((unused))void * pvParameters)
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

   // set the listening port to default web server port
   xBindAddr.sin_port = FreeRTOS_htons(80);

   // bind the socket to the port that the client can connect
   FreeRTOS_bind(xServerSocket, &xBindAddr, sizeof(xBindAddr));

   // set the socket into a listening state so it can accept connections
   FreeRTOS_listen(xServerSocket, 1);

   while (1)
   {
      // wait for incoming connection from client (host PC)
      xClientSocket = FreeRTOS_accept(xServerSocket, &xClientAddr, &xSize);

      // client connected
      web_server_handle_client(xClientSocket);

      // finished with the sockets
      FreeRTOS_closesocket(xClientSocket);
   }

   FreeRTOS_closesocket(xServerSocket);
}

/////////////////////////////////////////////////////////////

void web_server_start(void)
{
   if (g_xServerTask == NULL)
   {
      xTaskCreate(web_server_task , "WEB", 200, NULL, TASK_WEB_SERVER_PRIO, &g_xServerTask);
   }
}
