#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include <stdint.h>
#include <stdbool.h>

void motor_ctrl_init(void);

void motor_ctrl_pwm_set_duty(uint16_t duty_ctrl);

void motor_ctrl_pwm_set_direction(bool forward);

void motor_ctrl_brake(void);

#endif
