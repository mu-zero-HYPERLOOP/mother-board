#include "canzero.h"
#include "states.h"


static constexpr Duration STATE_TIMEOUT = 10_s;

global_state acceleration_state_next(global_command cmd,
                                     Duration time_since_last_transition) {

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

  if (time_since_last_transition < 500_ms) {
    if (canzero_get_motor_state() != motor_state_READY &&
        canzero_get_motor_state() != motor_state_START &&
        canzero_get_motor_state() != motor_state_CONTROL) {
      return global_state_DISCONNECTING;
    }
  } else {
    if (canzero_get_motor_state() != motor_state_START &&
        canzero_get_motor_state() != motor_state_CONTROL) {
      return global_state_DISCONNECTING;
    }
  }

  if (cmd == global_command_STOP_LEVITATION ||
      cmd == global_command_STOP_PROPULSION || cmd == global_command_ABORT) {
    return global_state_DECELERATION;
  }

  if (time_since_last_transition > STATE_TIMEOUT) {
    return global_state_DECELERATION;
  }


  if (canzero_get_motor_state() == motor_state_CONTROL) {
    return global_state_CRUISING;
  }

  canzero_set_mlu_command(mlu_command_START);
  canzero_set_mgu_command(mgu_command_START);
  canzero_set_motor_command(motor_command_ACCELERATE);

  return global_state_ACCELERATION;
}
