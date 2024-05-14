#include "canzero.h"
#include "range.h"
#include "states.h"
#include "systems.h"


constexpr Range<levitation_state, 3>
    allowed_levitation_states(levitation_state_READY, levitation_state_START,
                              levitation_state_CONTROL);

constexpr Duration state_timeout = 30_s;

global_state state_start_levitation_next(global_command command, Duration time_since_last_transition) {
  
  if (command == global_command_EMERGENCY
      || command == global_command_DISCONNECT) {
    return global_state_DISCONNECTING;
  }

  if (levitation_system.any_not_in(allowed_levitation_states)) {
    return global_state_DISCONNECTING;
  }
  
  if (guidance_system.any_not(guidance_state_READY)) {
    return global_state_DISCONNECTING;
  }

  if (motor_system.any_not(motor_state_READY)) {
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

  if (command == global_command_STOP_LEVITATION) {
    return global_state_STOP_LEVITATION;
  }

  if (time_since_last_transition > state_timeout) {
    return global_state_STOP_LEVITATION;
  }

  if (levitation_system.all(levitation_state_CONTROL)) {
    return global_state_LEVITATION_STABLE;
  }

  canzero_set_motor_driver_command(motor_command_STOP);
  canzero_set_levitation_command(levitation_command_START);
  canzero_set_guidance_command(guidance_command_STOP);

  return global_state_START_LEVITATION;
}
