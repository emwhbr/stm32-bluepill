#ifndef __SHARED_IR_NEC_H__
#define __SHARED_IR_NEC_H__

#include <stdint.h>

#include <FreeRTOS.h>

typedef enum
{
   NEC_OK,       // code indicating success (no error)
   NEC_TIMEOUT,  // timeout occurred
   NEC_FAIL      // generic code indicating failure
} nec_err_t;

typedef union
{
   struct
   {
      uint16_t addr    : 16;
      uint8_t  cmd     : 8;
      uint8_t  cmd_inv : 8;
   } __attribute__ ((packed)) bs;
   uint32_t wd;
} IR_NEC_MSG;

#define IR_NEC_MSG_REPEAT_CODE  0xffffffff

void ir_nec_init(UBaseType_t int_work_task_prio);

nec_err_t ir_nec_recv(IR_NEC_MSG *msg,
                      TickType_t xTicksToWait);

#endif
