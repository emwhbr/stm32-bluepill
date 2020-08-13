#include <FreeRTOS.h>
#include <queue.h>

#include <FreeRTOS_IP.h>
#include <NetworkInterface.h>
#include <NetworkBufferManagement.h>

#include "enc28j60.h"
#include "application_freertos_prio.h"

/////////////////////////////////////////////////////////////

static BaseType_t prvMACWaitLinkUp(TickType_t xMaxTime)
{
   BaseType_t xReturn;
   TickType_t xStartTime = xTaskGetTickCount();

   for(;;)
   {
      if (xTaskGetTickCount() - xStartTime > xMaxTime)
      {
         xReturn = pdFALSE;
         break;
      }

      if (enc28j60_get_phy_link_status())
      {
         xReturn = pdTRUE;
         break;
      }

      vTaskDelay(pdMS_TO_TICKS(20));
   }

   return xReturn;
}

/////////////////////////////////////////////////////////////

BaseType_t xNetworkInterfaceInitialise(void)
{
   static BaseType_t xDriverInitialized = pdFALSE;
   BaseType_t xLinkStatus;

   // initialize driver only once
   if (xDriverInitialized != pdTRUE)
   {
      if (enc28j60_init(FreeRTOS_GetMACAddress(),
                        TASK_ENC_INT_WRK_PRIO,
                        ipTOTAL_ETHERNET_FRAME_SIZE) == ENC_OK)
      {
         xDriverInitialized = pdTRUE;
      }
      else
      {
         return pdFAIL;
      }
   }

   // When returning non-zero, the stack will become
   // active and start DHCP (if configured).
   xLinkStatus = prvMACWaitLinkUp(pdMS_TO_TICKS(1000));

   return (xLinkStatus == pdTRUE ? pdPASS : pdFAIL);
}

/////////////////////////////////////////////////////////////

BaseType_t xNetworkInterfaceOutput(NetworkBufferDescriptor_t * const pxDescriptor,
                                   BaseType_t bReleaseAfterSend )
{
   BaseType_t xReturn = pdTRUE;

   if (enc28j60_send_packet(pxDescriptor->pucEthernetBuffer,
                            pxDescriptor->xDataLength) != ENC_OK)
   {
      xReturn = pdFALSE;
   }

   // buffer has been sent so can be released
   if(bReleaseAfterSend != pdFALSE)
   {
      vReleaseNetworkBufferAndDescriptor(pxDescriptor);
   }

   return xReturn;
}
