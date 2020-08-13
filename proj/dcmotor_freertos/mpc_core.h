#ifndef __MPC_CORE_H__
#define __MPC_CORE_H__

#include <stdbool.h>

void mpc_core_initialize(bool zero_shaft);

void mpc_core_calibrate(bool zero_shaft);

void mpc_core_position(void);

#endif
