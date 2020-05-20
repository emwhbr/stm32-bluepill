#include "FreeRTOS.h"
#include "queue.h"

#include "FreeRTOS_IP.h"
#include "NetworkInterface.h"

/////////////////////////////////////////////////////////////

BaseType_t xNetworkInterfaceInitialise( void )
{
   return 0;
}

/////////////////////////////////////////////////////////////

BaseType_t xNetworkInterfaceOutput( NetworkBufferDescriptor_t * const pxDescriptor,
                                    BaseType_t bReleaseAfterSend )
{
   (void)pxDescriptor;
   (void)bReleaseAfterSend;
   return pdTRUE;
}
