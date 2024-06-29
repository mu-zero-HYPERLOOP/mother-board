#include "canzero.h"
#include "control/velocity.h"
#include "fsm/states.h"
#include "sdc.h"
#include "subsystems.h"

constexpr Velocity VEL_CONSIDERED_STOPPED = 0.1_mps;

constexpr Duration STATE_TIMEOUT = 5_s;

// Invariants:
// - guidance in control
// - levitation in control
// - motor in control
// - input board is running
// - pdus are running
// - sdc is closed
// Exit condition:
// - velocity is below threshold.
global_state fsm::states::deceleration(global_command cmd,
                                       Duration time_since_last_transition) {

  const input_board_state input_state = canzero_get_input_board_state();

  const pdu_24v_state pdu24_state = canzero_get_power_board24_state();
  const pdu_12v_state pdu12_state = canzero_get_power_board12_state();

  const guidance_state g1_state = canzero_get_guidance_board_front_state();
  const guidance_state g2_state = canzero_get_guidance_board_back_state();

  const levitation_state l1_state = canzero_get_levitation_board1_state();
  const levitation_state l2_state = canzero_get_levitation_board2_state();
  const levitation_state l3_state = canzero_get_levitation_board3_state();

  const motor_state motor_state = canzero_get_motor_driver_state();

  if (global_command_RESTART == cmd){
    return global_state_RESTARTING;
  }

  // ================ TRANSITIONS ================
  if (global_command_SHUTDOWN == cmd){
    return global_state_SHUTDOWN;
  }

  if (global_command_EMERGENCY == cmd){
    return global_state_DISARMING45;
  }

  // Invariant: guidance
  if ((guidance_state_CONTROL != g1_state ||
       guidance_state_CONTROL != g2_state) &&
      !DISABLE_GUIDANCE_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: levitation
  if ((levitation_state_CONTROL != l1_state ||
       levitation_state_CONTROL != l2_state ||
       levitation_state_CONTROL != l3_state) &&
      !DISABLE_LEVITATION_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: motor
  if (motor_state_CONTROL != motor_state && !DISABLE_MOTOR_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: input_board
  if (input_board_state_RUNNING != input_state && !DISABLE_INPUT_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: pdus
  if ((pdu_12v_state_CHANNELS_ON != pdu12_state || pdu_24v_state_CHANNELS_ON != pdu24_state) &&
      !DISABLE_POWER_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: SDC
  if (sdc::status() == sdc_status_OPEN) {
    return global_state_DISARMING45;
  }

  if (time_since_last_transition > STATE_TIMEOUT){
    return global_state_DISARMING45;
  }

  // Pod commands:
  if (global_command_STOP_45 == cmd) {
    return global_state_DISARMING45;
  }

  if (global_command_STOP_LEVITATION == cmd) {
    return global_state_DISARMING45; // invalid command
  }

  // Transition back into ready, when the velocity is below a threshold.
  if (canzero_get_velocity() < static_cast<float>(VEL_CONSIDERED_STOPPED)) {
    return global_state_GUIDANCE_STABLE;
  }

  // =============== OUTPUTS =================
  canzero_set_guidance_command(guidance_command_NONE);
  canzero_set_levitation_command(levitation_command_NONE);
  canzero_set_motor_driver_command(motor_command_NONE);
  canzero_set_pod_grounded(bool_t_FALSE);
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_power_board12_command(pdu_12v_command_NONE);
  canzero_set_power_board24_command(pdu_24v_command_NONE);
  canzero_set_input_board_assert_45V_online(bool_t_TRUE);
  control::velocity::enable();
  control::velocity::target_velocity(0_mps);

  return global_state_DECELERATION;
}
