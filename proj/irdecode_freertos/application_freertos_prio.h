#ifndef __APP_FREERTOS_PRIO_H__
#define __APP_FREERTOS_PRIO_H__

#include <FreeRTOS.h>

/////////////////////////////////////////////////////////////

// Application task priorities
#define TASK_IR_DECODE_INT_WRK_PRIO  (configMAX_PRIORITIES - 1)
#define TASK_LED_PRIO                (configMAX_PRIORITIES - 2)
#define TASK_TEST_PRIO               (configMAX_PRIORITIES - 3)

#endif
