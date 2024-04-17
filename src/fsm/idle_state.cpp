
#include "canzero.h"
#include "states.h"

global_state idle_state_next(global_command cmd, Duration time_since_last_transition) {

  if (global_command_PRECHARGE == cmd) {
    return global_state_PRECHARGE;
  }

  canzero_set_mlu_command(mlu_command_DISCONNECT);
  canzero_set_mgu_command(mgu_command_DISCONNECT);
  canzero_set_motor_command(motor_command_DISCONNECT);

  return global_state_IDLE;
}
