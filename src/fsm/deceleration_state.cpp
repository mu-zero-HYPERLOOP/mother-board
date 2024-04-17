
#include "canzero.h"
#include "states.h"


static constexpr Duration STATE_TIMEOUT = 10_s;

global_state deceleration_state_next(global_command cmd, Duration time_since_last_transition) {

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
      canzero_get_mgu2_state() != mgu_state_CONTROL) {
    return global_state_DISCONNECTING;
  }

  if (time_since_last_transition < STATE_TIMEOUT) {
    return global_state_DECELERATION;
  }

  if (canzero_get_motor_state() == motor_state_CONTROL) {
    return global_state_READY;
  }

  return global_state_DECELERATION;
}
