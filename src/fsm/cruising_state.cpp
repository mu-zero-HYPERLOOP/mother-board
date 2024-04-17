
#include "canzero.h"
#include "states.h"

static constexpr Duration STATE_TIMEOUT = 1_min;

global_state cruising_state_next(global_command cmd, Duration time_since_last_transition) {


  if (cmd == global_command_DISCONNECT || cmd == global_command_EMERGENCY) {
    return global_state_DISCONNECTING;
  }

  if (canzero_get_mlu1_state() != mlu_state_CONTROL ||
      canzero_get_mlu2_state() != mlu_state_CONTROL ||
      canzero_get_mlu3_state() != mlu_state_CONTROL ||
      canzero_get_mlu4_state() != mlu_state_CONTROL ||
      canzero_get_mlu5_state() != mlu_state_CONTROL ||
      canzero_get_mlu6_state() != mlu_state_CONTROL ||
      canzero_get_mgu1_state() != mgu_state_CONTROL ||
      canzero_get_mgu2_state() != mgu_state_CONTROL ||
      canzero_get_motor_state() != motor_state_READY) {
    return global_state_DISCONNECTING;
  }

  if (cmd == global_command_STOP_PROPULSION || cmd == global_command_ABORT 
      || global_command_STOP_LEVITATION) {
    return global_state_DECELERATION;
  }

  if (time_since_last_transition > STATE_TIMEOUT) {
    return global_state_DECELERATION;
  }

  return global_state_CRUISING;
}
