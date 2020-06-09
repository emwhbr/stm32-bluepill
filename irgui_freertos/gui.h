#ifndef __GUI_H__
#define __GUI_H__

#include <stdint.h>

#include "ir_nec.h"

void gui_init(void);

void gui_set_nec_message(IR_NEC_MSG msg);

void gui_set_nec_count(uint32_t count);

#endif
