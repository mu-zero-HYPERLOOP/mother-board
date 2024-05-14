#include "canzero.h"
#include "states.h"
#include "systems.h"

constexpr Range<levitation_state, 3>
    allowed_levitation_states(levitation_state_IDLE, levitation_state_PRECHARGE,
                              levitation_state_READY);

constexpr Range<guidance_state, 3>
    allowed_guidance_states(guidance_state_IDLE, guidance_state_PRECHARGE,
                            guidance_state_READY);

constexpr Range<motor_state, 3> allowed_motor_state(motor_state_IDLE,
                                                 motor_state_PRECHARGE,
                                                 motor_state_READY);

constexpr Duration state_timeout = 10_s;

global_state state_precharge_next(global_command command,
                                  Duration time_since_last_transition) {

  if (command == global_command_EMERGENCY ||
      command == global_command_DISCONNECT) {
    return global_state_DISCONNECTING;
  }

  if (levitation_system.any_not_in(allowed_levitation_states)) {
    return global_state_DISCONNECTING;
  }

  if (guidance_system.any_not_in(allowed_guidance_states)) {
    return global_state_DISCONNECTING;
  }

  if (motor_system.any_not_in(allowed_motor_state)) {
    return global_state_DISCONNECTING;
  }
  
  if (sensor_system.any_not(input_board_state_RUNNING)) {
    return global_state_DISCONNECTING;
  }

  if (power_system.any_not(pdu_state_RUNNING)) {
    return global_state_DISCONNECTING;
  }

  if (time_since_last_transition > state_timeout) {
    return global_state_DISCONNECTING;
  }

  if (levitation_system.all(levitation_state_READY) 
      && guidance_system.all(guidance_state_READY)
      && motor_system.all(motor_state_READY)) {
    return global_state_READY;
  }

  canzero_set_motor_driver_command(motor_command_PRECHARGE);
  canzero_set_levitation_command(levitation_command_PRECHARGE);
  canzero_set_guidance_command(guidance_command_PRECHARGE);


  return global_state_PRECHARGE;
}
