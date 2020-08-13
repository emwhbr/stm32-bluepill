#include "motor.h"
#include "motor_ctrl.h"
#include "motor_encoder.h"

/////////////////////////////////////////////////////////////

void motor_init()
{
   motor_encoder_init();
   motor_ctrl_init();
}

/////////////////////////////////////////////////////////////

uint16_t motor_pwm_max_duty(void)
{
   return MOTOR_PWM_MAX_DUTY;
}

/////////////////////////////////////////////////////////////

void motor_ctrl(bool clockwise, uint16_t duty)
{
   if (duty)
   {
      motor_ctrl_set_speed(clockwise, duty);
   }
   else
   {
      motor_ctrl_brake();
   }
}

/////////////////////////////////////////////////////////////

void motor_zero_shaft_position(void)
{
   motor_encoder_zero_gearbox_shaft_pos();
}

/////////////////////////////////////////////////////////////

uint16_t motor_get_shaft_position(void)
{
   return motor_encoder_get_gearbox_shaft_pos();
}

/////////////////////////////////////////////////////////////

uint16_t motor_shaft_max_position(void)
{
   return MOTOR_ENCODER_CPR_GEAR_SHAFT;
}
