#include "canzero.h"
#include "range.h"
#include "states.h"
#include "systems.h"
#include "velocity_profile.h"

constexpr Range<motor_state, 3> allowed_motor_states(motor_state_READY,
                                                     motor_state_START,
                                                     motor_state_CONTROL);

constexpr Duration state_timeout = 5_s;

Duration last_vel_to_low = 0_ms;
Duration cruising_velocity_timethresh = 200_ms;

global_state state_acceleration_next(global_command command,
                                     Duration time_since_last_transition) {

  if (command == global_command_EMERGENCY ||
      command == global_command_DISCONNECT) {
    return global_state_DISCONNECTING;
  }

  if (motor_system.any_not_in(allowed_motor_states)) {
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

  if (canzero_get_velocity() >= canzero_get_target_velocity()) {
    if (time_since_last_transition - last_vel_to_low > cruising_velocity_timethresh) {
      return global_state_CRUISING;
    }
  }else{
    last_vel_to_low = time_since_last_transition;
  }

  canzero_set_motor_driver_command(motor_command_ACCELERATE);
  canzero_set_levitation_command(levitation_command_START);
  canzero_set_guidance_command(guidance_command_START);

  return global_state_ACCELERATION;
}
