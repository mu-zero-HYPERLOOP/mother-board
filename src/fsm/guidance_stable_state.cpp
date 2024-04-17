
#include "canzero.h"
#include "states.h"


static constexpr Duration STATE_TIMEOUT = 30_min;

// invariant
// - mlus = control
// - mgus = control
// - motor is ready
global_state guidance_stable_state_next(global_command cmd,
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
      canzero_get_mgu2_state() != mgu_state_CONTROL ||
      canzero_get_motor_state() != motor_state_READY) {
    return global_state_DISCONNECTING;
  }

  if (cmd == global_command_STOP_LEVITATION || cmd == global_command_ABORT) {
    return global_state_STOP_LEVITATION;
  }

  if (time_since_last_transition > STATE_TIMEOUT) {
    return global_state_STOP_LEVITATION;
  }


  if (cmd == global_command_START_PROPULSION) {
    return global_state_ACCELERATION;
  }

  canzero_set_mlu_command(mlu_command_START);
  canzero_set_mgu_command(mgu_command_START);
  canzero_set_motor_command(motor_command_STOP);

  return global_state_GUIDANCE_STABLE;
}
