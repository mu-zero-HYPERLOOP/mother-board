#include "canzero.h"
#include "sdc.h"
#include "states.h"
#include "systems.h"

global_state state_disconnecting_next(global_command command,
                                      Duration time_since_last_transition) {

  if (levitation_system.all(levitation_state_IDLE) &&
      guidance_system.all(guidance_state_IDLE) &&
      motor_system.all(motor_state_IDLE) && sdc_system.any(sdc_status_OPEN)) {
    return global_state_IDLE;
  }

  canzero_set_motor_driver_command(motor_command_DISCONNECT);
  canzero_set_levitation_command(levitation_command_DISCONNECT);
  canzero_set_guidance_command(guidance_command_DISCONNECT);

  return global_state_DISCONNECTING;
}
