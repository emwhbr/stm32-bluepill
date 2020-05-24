# FreeRTOS+TCP

## General
FreeRTOS+TCP is a scalable open source TCP/IP stack for FreeRTOS.<br/>
Further information is found at: https://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/index.html

This directory includes the common FreeRTOS+TCP source files and specifics from portable/Compiler/GCC.<br/>

The files were copied from the GitHub repository, https://github.com/FreeRTOS/FreeRTOS<br/>
(commit: 5003d17feda25490e655c0f1c15d2b13e395c9f7).<br/>

In the repository, the source code for the TCP/IP stack is found in the directory:<br/>
...FreeRTOS-Plus/Source/FreeRTOS-Plus-TCP

## FreeRTOSIPConfig.h
The FreeRTOSIPConfig.h is specific for each individual project.<br/>

Template for this file was taken from the demo source code.<br/>
...Demo/FreeRTOS_Plus_TCP_Minimal_Windows_Simulator/FreeRTOSIPConfig.h<br/>

See the following URL for configuration information.<br/>
http://www.freertos.org/FreeRTOS-Plus/FreeRTOS_Plus_TCP/TCP_IP_Configuration.html<br/>

## Structure
FreeRTOS-Plus-TCP/<br/>
├── __BufferAllocation_1.c <-- Not used in this project__<br/>
├── BufferAllocation_2.c<br/>
├── FreeRTOS_ARP.c<br/>
├── FreeRTOS_DHCP.c<br/>
├── FreeRTOS_DNS.c<br/>
├── FreeRTOS_IP.c<br/>
├── FreeRTOS_Sockets.c<br/>
├── FreeRTOS_Stream_Buffer.c<br/>
├── FreeRTOS_TCP_IP.c<br/>
├── FreeRTOS_TCP_WIN.c<br/>
├── FreeRTOS_UDP_IP.c<br/>
├── include<br/>
│   ├── FreeRTOS_ARP.h<br/>
│   ├── FreeRTOS_DHCP.h<br/>
│   ├── FreeRTOS_DNS.h<br/>
│   ├── FreeRTOS_errno_TCP.h<br/>
│   ├── FreeRTOSIPConfigDefaults.h<br/>
│   ├── FreeRTOS_IP.h<br/>
│   ├── FreeRTOS_IP_Private.h<br/>
│   ├── FreeRTOS_Sockets.h<br/>
│   ├── FreeRTOS_Stream_Buffer.h<br/>
│   ├── FreeRTOS_TCP_IP.h<br/>
│   ├── FreeRTOS_TCP_WIN.h<br/>
│   ├── FreeRTOS_UDP_IP.h<br/>
│   ├── IPTraceMacroDefaults.h<br/>
│   ├── NetworkBufferManagement.h<br/>
│   ├── NetworkInterface.h<br/>
│   ├── pack_struct_end.h<br/>
│   └── pack_struct_start.h<br/>
└── README.md<br/>
