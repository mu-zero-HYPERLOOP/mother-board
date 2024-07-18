#include "canzero.h"
#include "control/velocity.h"
#include "error_handling.h"
#include "fsm/states.h"
#include "fsm/invariants.h"
#include <iostream>
#include "sdc.h"
#include "subsystems.h"

constexpr std::array<motor_state, 2> ALLOWED_MOTOR_STATES = {
    motor_state_CONTROL, motor_state_READY};

// Invariant:
// - guidance is in control
// - levitation is in control
// - motor is ready
// - input board is running
// - pdus are running
// - sdc is closed.
// Exit condition:
// - All guidance systems in control, motor in ready.
global_state fsm::states::guidance_stable(global_command cmd, Duration time_since_last_transition) {

  using namespace fsm::invariant;

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
  
  // ================= TRANSITIONS ====================
  if (global_command_SHUTDOWN == cmd){
    return global_state_SHUTDOWN;
  }

  if (global_command_EMERGENCY == cmd){
    return global_state_DISARMING45;
  }

  // Invariant: guidance
  if ((guidance_state_CONTROL != g1_state
      || guidance_state_CONTROL != g2_state) && !DISABLE_GUIDANCE_SUBSYSTEM){
    std::cout << "GUIDANCE INVARIANT BROKEN" << std::endl;
    return error_handling::invariant_broken();
  }

  // Invariant: levitation
  if ((levitation_state_CONTROL != l1_state
      || levitation_state_CONTROL != l2_state
      || levitation_state_CONTROL != l3_state) && !DISABLE_LEVITATION_SUBSYSTEM){
    std::cout << "LEVITATION INVARIANT BROKEN" << std::endl;
    return error_handling::invariant_broken();
  }

  // Invariant: motor state
  if (!contains(ALLOWED_MOTOR_STATES, motor_state) &&
      !DISABLE_MOTOR_SUBSYSTEM) {
    return error_handling::invariant_broken();
  }

  // Invariant: input board
  if (input_board_state_RUNNING != input_state && !DISABLE_INPUT_SUBSYSTEM){
    std::cout << "INPUT BOARD INVARIANT BROKEN" << std::endl;
    return error_handling::invariant_broken();
  }

  // Invariant: pdus
  if ((pdu_12v_state_CHANNELS_ON != pdu12_state 
      || pdu_24v_state_CHANNELS_ON != pdu24_state) && !DISABLE_POWER_SUBSYSTEM){
    std::cout << "PDU INVARIANT BROKEN" << std::endl;
    return error_handling::invariant_broken();
  }

  // Invariant: sdc
  if (sdc::status() == sdc_status_OPEN){
    std::cout << "SDC INVARIANT BROKEN" << std::endl;
    return error_handling::invariant_broken();
  }

  if (global_command_STOP_45 == cmd){
    return global_state_DISARMING45;
  }

  if (global_command_STOP_LEVITATION == cmd || global_command_ABORT == cmd){
    return global_state_STOP_GUIDANCE;
  }

  // Transition into propulsion iff.
  // - ACCELERATE command
  // - absolute position known
  if (global_command_START_PROPULSION == cmd){
    canzero_set_command(global_command_NONE);
    if (canzero_get_absolute_position_known() == bool_t_FALSE) {
      return global_state_STOP_GUIDANCE;
    }
    return global_state_ACCELERATION;
  }

  // Transition into controller iff.
  // - START_CONTROLLER command.
  if (global_command_START_CONTROLLER == cmd){
    return global_state_CONTROLLER;
  }
  
  // =================== OUTPUTS ======================

  canzero_set_guidance_command(guidance_command_NONE);
  canzero_set_levitation_command(levitation_command_NONE);
  canzero_set_motor_driver_command(motor_command_NONE);
  canzero_set_pod_grounded(bool_t_FALSE);
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_power_board12_command(pdu_12v_command_NONE);
  canzero_set_power_board24_command(pdu_24v_command_NONE);
  canzero_set_input_board_assert_45V_online(bool_t_TRUE);
  control::velocity::disable();

  return global_state_GUIDANCE_STABLE;
}
