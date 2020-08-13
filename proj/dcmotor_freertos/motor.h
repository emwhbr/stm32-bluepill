#ifndef __MOTOR_H__
#define __MOTOR_H__

#include <stdint.h>
#include <stdbool.h>

void motor_init(void);

uint16_t motor_pwm_max_duty(void);

void motor_ctrl(bool clockwise, uint16_t duty);

void motor_zero_shaft_position(void);

uint16_t motor_get_shaft_position(void);

uint16_t motor_shaft_max_position(void);

#endif
