#include "canzero.h"
#include "states.h"


global_state state_levitation_stable_next(global_command command, Duration time_since_last_transition) {
  if (command == global_command_EMERGENCY || command == global_command_DISCONNECT) {
    return global_state_DISCONNECTING;
  }

  if (command == global_command_STOP_LEVITATION) {
    return global_state_STOP_LEVITATION;
  }

  canzero_set_motor_driver_command(motor_command_STOP);
  canzero_set_levitation_command(levitation_command_START);
  canzero_set_guidance_command(guidance_command_STOP);

  return global_state_START_GUIDANCE;
}
