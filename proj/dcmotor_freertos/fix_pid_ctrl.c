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
   fix_pid_ctrl_set_output_limits(pid, FIX_MIN, FIX_MAX);

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
   pid->m_pos_error = fix_sub_sat(pid->m_command_position, position);

   /* calculate PROPORTIONAL term */
   p_term = fix_mul_sat(pid->m_p_gain, pid->m_pos_error);

   /* Calculate INTEGRAL term */
   pid->m_i_state = fix_add_sat(pid->m_pos_error, pid->m_i_state);
   i_term = fix_mul_sat(pid->m_i_gain, pid->m_i_state);

   /* calculate DERIVATE term */
   d_term = fix_sub_sat(pid->m_d_state, position);
   d_term = fix_mul_sat(pid->m_d_gain, d_term);
   pid->m_d_state = position;

   /* calculate output */
   output = fix_add_sat(p_term, i_term);
   output = fix_add_sat(output, d_term);
   pid->m_output = output;

   return output;
}

/****************************************************************************
 *               Private functions
 ****************************************************************************/
 
/*****************************************************************/
