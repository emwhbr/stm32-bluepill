#include <stdio.h>
#include <stdbool.h>

#include "mpc_fsm.h"
#include "mpc_core.h"

/////////////////////////////////////////////////////////////

// global variables
static enum mpc_fsm_state g_current_state = MPC_FSM_STATE_INIT;

// internal functions
static void mpc_fsm_state_action(enum mpc_fsm_state state,
                                 enum mpc_fsm_event event);

/////////////////////////////////////////////////////////////

void mpc_fsm_init(void)
{
   g_current_state = MPC_FSM_STATE_INIT;
}

/////////////////////////////////////////////////////////////

void mpc_fsm_execute(enum mpc_fsm_event event)
{
   enum mpc_fsm_state next_state = g_current_state;

   // determine next state
   switch (g_current_state)
   {
   case MPC_FSM_STATE_INIT:
      switch (event)
      {
      case MPC_FSM_EVENT_BUT_CALIB:
         printf("INIT-->CALIB\n");
         next_state = MPC_FSM_STATE_CALIB;
         break;
      default:
         break;
      }
      break;
   case MPC_FSM_STATE_CALIB:
      switch (event)
      {
      case MPC_FSM_EVENT_BUT_POS:
         printf("CALIB-->POS\n");
         next_state = MPC_FSM_STATE_POS;
         break;
      case MPC_FSM_EVENT_BUT_INIT:
         printf("CALIB-->INIT\n");
         next_state = MPC_FSM_STATE_INIT;
         break;
      default:
         break;
      }
      break;
   case MPC_FSM_STATE_POS:
      switch (event)
      {
      case MPC_FSM_EVENT_BUT_CALIB:
         printf("POS-->CALIB\n");
         next_state = MPC_FSM_STATE_CALIB;
         break;
      case MPC_FSM_EVENT_BUT_INIT:
         printf("POS-->INIT\n");
         next_state = MPC_FSM_STATE_INIT;
         break;
      default:
         break;
      }
      break;
   }

   // check if transition to new state
   if (next_state != g_current_state)
   {
      g_current_state = next_state;
   }

   // execute state action
   mpc_fsm_state_action(g_current_state, event);
}

/////////////////////////////////////////////////////////////

static void mpc_fsm_state_action(enum mpc_fsm_state state,
                                 enum mpc_fsm_event event)
{
   switch (state)
   {
   case MPC_FSM_STATE_INIT:
      mpc_core_initialize(event == MPC_FSM_EVENT_BUT_ZERO);
      break;
   case MPC_FSM_STATE_CALIB:
      mpc_core_calibrate(event == MPC_FSM_EVENT_BUT_ZERO);
      break;
   case MPC_FSM_STATE_POS:
      mpc_core_position();
      break;
  }
}
