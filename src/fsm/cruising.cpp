#include "canzero.h"
#include "range.h"
#include "states.h"
#include "systems.h"
#include "velocity_profile.h"



constexpr Duration state_timeout = 10_s;

global_state state_cruising_next(global_command command, Duration time_since_last_transition) {


  if (command == global_command_EMERGENCY ||
      command == global_command_DISCONNECT) {
    return global_state_DISCONNECTING;
  }

  if (motor_system.any_not(motor_state_CONTROL)) {
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
    return global_state_DECELERATION;
  }

  if (command == global_command_STOP_PROPULSION ||
      command == global_command_ABORT) {
    return global_state_DECELERATION;
  }


  if (velocity_profile_decelerate()) {
    return global_state_DECELERATION;
  }

  canzero_set_motor_driver_command(motor_command_ACCELERATE);
  canzero_set_levitation_command(levitation_command_START);
  canzero_set_guidance_command(guidance_command_START);

  return global_state_CRUISING;
}
