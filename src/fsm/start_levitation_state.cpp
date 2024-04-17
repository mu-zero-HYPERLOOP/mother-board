
#include "canzero.h"
#include "states.h"

static constexpr Duration STATE_TIMEOUT = 5_s;
static constexpr Duration MAX_RESPONSE = 500_ms;

// invariant all mlus start with levitation
//   && mgu and motors are ready!
global_state start_levitation_state_next(global_command cmd,
                                         Duration time_since_last_transition) {

  if (cmd == global_command_DISCONNECT || cmd == global_command_EMERGENCY) {
    return global_state_DISCONNECTING;
  }

  if (cmd == global_command_STOP_LEVITATION || cmd == global_command_ABORT) {
    return global_state_STOP_LEVITATION;
  }

  // check state of all mlus
  if (time_since_last_transition < MAX_RESPONSE) {
    if ((canzero_get_mlu1_state() != mlu_state_READY &&
         canzero_get_mlu1_state() != mlu_state_START &&
         canzero_get_mlu1_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu2_state() != mlu_state_READY &&
         canzero_get_mlu2_state() != mlu_state_START &&
         canzero_get_mlu2_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu3_state() != mlu_state_READY &&
         canzero_get_mlu3_state() != mlu_state_START &&
         canzero_get_mlu3_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu4_state() != mlu_state_READY &&
         canzero_get_mlu4_state() != mlu_state_START &&
         canzero_get_mlu4_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu5_state() != mlu_state_READY &&
         canzero_get_mlu5_state() != mlu_state_START &&
         canzero_get_mlu5_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu6_state() != mlu_state_READY &&
         canzero_get_mlu6_state() != mlu_state_START &&
         canzero_get_mlu6_state() != mlu_state_CONTROL)) {
      return global_state_DISCONNECTING;
    }
  } else {
    if ((canzero_get_mlu1_state() != mlu_state_START &&
         canzero_get_mlu1_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu2_state() != mlu_state_START &&
         canzero_get_mlu2_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu3_state() != mlu_state_START &&
         canzero_get_mlu3_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu4_state() != mlu_state_START &&
         canzero_get_mlu4_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu5_state() != mlu_state_START &&
         canzero_get_mlu5_state() != mlu_state_CONTROL) ||
        (canzero_get_mlu6_state() != mlu_state_START &&
         canzero_get_mlu6_state() != mlu_state_CONTROL)) {
      return global_state_DISCONNECTING;
    }
  }

  // check state of mgus and the motor
  if (canzero_get_mgu1_state() != mgu_state_READY ||
      canzero_get_mgu2_state() != mgu_state_READY ||
      canzero_get_motor_state() != motor_state_READY) {
    return global_state_DISCONNECTING;
  }

  if (time_since_last_transition > STATE_TIMEOUT) {
    return global_state_STOP_LEVITATION;
  }


  if (canzero_get_mlu1_state() == mlu_state_CONTROL &&
      canzero_get_mlu2_state() == mlu_state_CONTROL &&
      canzero_get_mlu3_state() == mlu_state_CONTROL &&
      canzero_get_mlu4_state() == mlu_state_CONTROL &&
      canzero_get_mlu5_state() == mlu_state_CONTROL &&
      canzero_get_mlu6_state() == mlu_state_CONTROL) {
    return global_state_LEVITATION_STABLE;
  }

  canzero_set_mlu_command(mlu_command_START);
  canzero_set_mgu_command(mgu_command_STOP);
  canzero_set_motor_command(motor_command_STOP);

  return global_state_START_LEVITATION;
}
