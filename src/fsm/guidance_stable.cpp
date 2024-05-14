#include "canzero.h"
#include "states.h"
#include "systems.h"


constexpr Duration state_timeout = 30_min;

global_state state_guidance_stable_next(global_command command, Duration time_since_last_transition) {
  if (command == global_command_EMERGENCY || command == global_command_DISCONNECT) {
    return global_state_DISCONNECTING;
  }

  if (motor_system.any_not(motor_state_READY)) {
    return global_state_DISCONNECTING;   
  }

  if (levitation_system.any_not(levitation_state_CONTROL)) {
    return global_state_DISCONNECTING;   
  }

  if (guidance_system.any_not(guidance_state_CONTROL)) {
    return global_state_DISCONNECTING;   
  }

  if (sensor_system.any_not(input_board_state_RUNNING)) {
    return global_state_DISCONNECTING;
  }

  if (power_system.any_not(pdu_state_RUNNING)) {
    return global_state_DISCONNECTING;
  }

  if (sdc_system.any_not(sdc_status_CLOSED)) {
    return global_state_DISCONNECTING;
  }

  if (time_since_last_transition > state_timeout) {
    return global_state_STOP_LEVITATION;
  }

  if (command == global_command_STOP_LEVITATION) {
    return global_state_STOP_LEVITATION;
  }

  if (command == global_command_START_PROPULSION) {
    return global_state_ACCELERATION;
  }

  canzero_set_motor_driver_command(motor_command_STOP);
  canzero_set_levitation_command(levitation_command_START);
  canzero_set_guidance_command(guidance_command_START);

  return global_state_GUIDANCE_STABLE;

}

