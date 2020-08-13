#ifndef __MPC_FSM_H__
#define __MPC_FSM_H__

// state definitions
enum mpc_fsm_state
{
   MPC_FSM_STATE_INIT,  // initialization
   MPC_FSM_STATE_CALIB, // calibration
   MPC_FSM_STATE_POS    // position control
};

// event definitions
enum mpc_fsm_event
{
   MPC_FSM_EVENT_NONE,
   MPC_FSM_EVENT_BUT_INIT,   // button 'initialize' is pressed
   MPC_FSM_EVENT_BUT_ZERO,   // button 'zero' is pressed
   MPC_FSM_EVENT_BUT_CALIB,  // button 'calibration' is pressed
   MPC_FSM_EVENT_BUT_POS     // button 'position' is pressed
};

void mpc_fsm_init(void);

void mpc_fsm_execute(enum mpc_fsm_event event);

#endif
