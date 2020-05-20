#include <stdio.h>
#include <string.h>
#include <stdint.h>
#include <stdarg.h>

#include "FreeRTOS.h"
#include <task.h>

#include <FreeRTOS_IP.h>
#include <FreeRTOS_Sockets.h>

/////////////////////////////////////////////////////////////

extern uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                                   uint16_t usSourcePort,
                                                   uint32_t ulDestinationAddress,
                                                   uint16_t usDestinationPort);

// Use by the pseudo random number generator
static UBaseType_t ulNextRand;

/////////////////////////////////////////////////////////////

uint32_t ulApplicationGetNextSequenceNumber(uint32_t ulSourceAddress,
                                            uint16_t usSourcePort,
                                            uint32_t ulDestinationAddress,
                                            uint16_t usDestinationPort)
{
   ( void ) ulSourceAddress;
   ( void ) usSourcePort;
   ( void ) ulDestinationAddress;
   ( void ) usDestinationPort;

   return uxRand();
}

/////////////////////////////////////////////////////////////

BaseType_t xApplicationGetRandomNumber(uint32_t* pulNumber)
{
   *(pulNumber) = uxRand();
   return pdTRUE;
}

/////////////////////////////////////////////////////////////

UBaseType_t uxRand(void)
{
const uint32_t ulMultiplier = 0x015a4e35UL, ulIncrement = 1UL;

   /* Utility function to generate a pseudo random number. */

   ulNextRand = ( ulMultiplier * ulNextRand ) + ulIncrement;
   return( ( int ) ( ulNextRand >> 16UL ) & 0x7fffUL );
}

/////////////////////////////////////////////////////////////

// Called by FreeRTOS+TCP when the network connects or disconnects.
// Disconnect events are only received if implemented in the MAC driver.

void vApplicationIPNetworkEventHook(eIPCallbackEvent_t eNetworkEvent)
{
   uint32_t ulIPAddress, ulNetMask, ulGatewayAddress, ulDNSServerAddress;
   char cBuffer[16];

   printf("\nEV:%u\n", eNetworkEvent);

   if (eNetworkEvent == eNetworkUp)
   {
      // Print out the network configuration, which may have come from a DHCP server
      FreeRTOS_GetAddressConfiguration(&ulIPAddress, &ulNetMask, &ulGatewayAddress, &ulDNSServerAddress);

      FreeRTOS_inet_ntoa( ulIPAddress, cBuffer );
      printf("IP: %s\n", cBuffer);
      FreeRTOS_inet_ntoa( ulNetMask, cBuffer );
      printf("SM: %s\n", cBuffer);
      FreeRTOS_inet_ntoa( ulGatewayAddress, cBuffer );
      printf("GW: %s\n", cBuffer);
      FreeRTOS_inet_ntoa( ulDNSServerAddress, cBuffer );
      printf("DNS: %s\n", cBuffer);
      fflush(stdout);
   }
}

/////////////////////////////////////////////////////////////

void vLoggingPrintf(const char *pcFormat, ...)
{
   va_list args;
   va_start(args, pcFormat);
   printf(pcFormat, args);
   va_end(args);
}
