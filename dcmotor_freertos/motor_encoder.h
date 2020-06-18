#ifndef __MOTOR_ENCODER_H__
#define __MOTOR_ENCODER_H__

#include <stdint.h>

// quadrature encoder definitions, Pololu#3240
#define MOTOR_ENCODER_CPR_GEAR_SHAFT  1632 // CPR x GearRatio = 48 * 34.014

void motor_encoder_init(void);

void motor_encoder_zero_gearbox_shaft_pos(void);

uint32_t motor_encoder_get_gearbox_shaft_pos(void);

#endif