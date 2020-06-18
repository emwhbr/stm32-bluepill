#ifndef __MOTOR_CTRL_H__
#define __MOTOR_CTRL_H__

#include <stdint.h>
#include <stdbool.h>

// Maximum PWM duty value that can be applied
#define MOTOR_PWM_MAX_DUTY  (800) // T=100us, Fpwm=10kHz

void motor_ctrl_init(void);

void motor_ctrl_set_speed(bool forward, uint16_t duty_ctrl);

void motor_ctrl_brake(void);

#endif
