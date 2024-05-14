#include "canzero.h"
#include "states.h"
#include "system.h"

global_state state_idle_next(global_command command,
                             Duration time_since_last_transition) {

  if (command == global_command_PRECHARGE) {
    return global_state_PRECHARGE;
  }

  canzero_set_motor_driver_command(motor_command_STOP);
  canzero_set_levitation_command(levitation_command_STOP);
  canzero_set_guidance_command(guidance_command_STOP);

  return global_state_IDLE;
}
