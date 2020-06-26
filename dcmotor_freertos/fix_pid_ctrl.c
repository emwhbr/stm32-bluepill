#include "fix_pid_ctrl.h"

/*
 * Implementation notes:
 * 1. PID without a PhD, Tim Wescott
 *    http://m.eet.com/media/1112634/f-wescot.pdf
 *
 * 2. Arduino PID library
 *    http://playground.arduino.cc/Code/PIDLibrary
 */

/****************************************************************************
 *               Function prototypes
 ****************************************************************************/

/****************************************************************************
 *               Exported functions
 ****************************************************************************/

/*****************************************************************/

void fix_pid_ctrl_initialize(struct fix_pid_ctrl *pid,
                             fix_t p_gain,
                             fix_t i_gain,
                             fix_t d_gain)
{
   /* default output range: 0 - 100 */
   fix_pid_ctrl_set_command_position(pid, 0);
   fix_pid_ctrl_set_output_limits(pid, Q31_MIN, Q31_MAX);

   pid->m_pos_error = 0;

   pid->m_output = pid->m_output_min;

   pid->m_p_gain = p_gain;

   pid->m_i_gain = i_gain;
   pid->m_i_state = 0;

   pid->m_d_gain = d_gain;
   pid->m_d_state = pid->m_command_position;
}

/*****************************************************************/

void fix_pid_ctrl_set_output_limits(struct fix_pid_ctrl *pid,
                                    fix_t min,
                                    fix_t max)
{
   pid->m_output_min = min;
   pid->m_output_max = max;

   pid->m_i_min = min;
   pid->m_i_max = max;
}

/*****************************************************************/

void fix_pid_ctrl_set_command_position(struct fix_pid_ctrl *pid,
                                       fix_t value)
{
   pid->m_command_position = value;
}

/*****************************************************************/

fix_t fix_pid_ctrl_update(struct fix_pid_ctrl *pid,
                          fix_t position)
{
   fix_t p_term;
   fix_t i_term;
   fix_t d_term;

   fix_t output;

   /* calculate error */
   arm_sub_q31(&pid->m_command_position,
               &position,
               &pid->m_pos_error,
               1);

   /* calculate PROPORTIONAL term */
   arm_mult_q31(&pid->m_p_gain,
                &pid->m_pos_error,
                &p_term,
                1);

   /* Calculate INTEGRAL term */
   arm_add_q31 (&pid->m_pos_error,
                &pid->m_i_state,
                &pid->m_i_state,
                1);

   arm_mult_q31(&pid->m_i_gain,
                &pid->m_i_state,
                &i_term,
                1);

   /* calculate DERIVATE term */
   arm_sub_q31(&pid->m_d_state,
               &position,
               &d_term,
               1);

   arm_mult_q31(&pid->m_d_gain,
                &d_term,
                &d_term,
                1);

   pid->m_d_state = position;

   /* calculate output */
   arm_add_q31 (&p_term,
                &i_term,
                &output,
                1);

   arm_add_q31 (&output,
                &d_term,
                &output,
                1);

   pid->m_output = output;

   return output;
}

/****************************************************************************
 *               Private functions
 ****************************************************************************/
 
/*****************************************************************/
