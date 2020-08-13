#ifndef __FIX_PID_CTRL_H__
#define __FIX_PID_CTRL_H__

#include "fix.h" // custom - fixed point math (Q31)

/****************************************************************************
 *               Types and definitions
 ****************************************************************************/

/* pid controller "class" */
struct fix_pid_ctrl
{
   /* set-point */
   fix_t m_command_position;

   /* error */
   fix_t m_pos_error;

   /* output */  
   fix_t m_output;
   fix_t m_output_min;
   fix_t m_output_max;

   /* proportional */
   fix_t m_p_gain; /* (P)roportional gain */

   /* integral */
   fix_t m_i_gain;  /* (I)ntegral gain                */
   fix_t m_i_state; /* integrator state               */
   fix_t m_i_min;   /* min allowable integrator state */
   fix_t m_i_max;   /* max allowable integrator state */

   /* derivate */
   fix_t m_d_gain;  /* (D)erivative gain   */
   fix_t m_d_state; /* last position input */
};

/****************************************************************************
 *               Exported functions
 ****************************************************************************/

/* define PID parameters */
void fix_pid_ctrl_initialize(struct fix_pid_ctrl *pid,
                             fix_t p_gain,
                             fix_t i_gain,
                             fix_t d_gain);

/* output limits */
void fix_pid_ctrl_set_output_limits(struct fix_pid_ctrl *pid,
                                    fix_t min,
                                    fix_t max);

/* update set-point */
void fix_pid_ctrl_set_command_position(struct fix_pid_ctrl *pid,
                                       fix_t value);

/* performs the PID calculation*/
fix_t fix_pid_ctrl_update(struct fix_pid_ctrl *pid,
                          fix_t position);

#endif /* __FIX_PID_CTRL_H_*/
