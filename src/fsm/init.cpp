#include "canzero.h"
#include "states.h"

#include "systems.h"

global_state state_init_next(global_command command,
                             Duration time_since_last_transition) {

  if (levitation_system.all(levitation_state_IDLE) &&
      guidance_system.all(guidance_state_IDLE) &&
      motor_system.all(motor_state_IDLE) &&
      sensor_system.all(input_board_state_RUNNING) &&
      power_system.all(pdu_state_RUNNING)) {
    return global_state_IDLE;
  }

  canzero_set_motor_driver_command(motor_command_STOP);
  canzero_set_levitation_command(levitation_command_STOP);
  canzero_set_guidance_command(guidance_command_STOP);

  return global_state_INIT;
}
