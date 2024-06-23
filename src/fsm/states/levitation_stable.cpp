#include "canzero.h"
#include "control/velocity.h"
#include "fsm/states.h"
#include "fsm/invariants.h"
#include "sdc.h"
#include "subsystems.h"
#include <iostream>


constexpr Duration MIN_STABLE_TIME = 2_s;

constexpr Duration STATE_TIMEOUT = 10_s;

// Invariant:
// - guidance is ready
// - levitation is [control]
// - motor is ready
// - input board is running
// - pdus are running
// - SDC is closed.
// Exit condition:
// - Levitation is stable, other subsystems are ready.
global_state fsm::states::levitation_stable(global_command cmd, Duration time_since_last_transition) {
  
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

  // ==================== TRANSITION ====================
  if (global_command_SHUTDOWN == cmd){
    return global_state_SHUTDOWN;
  }

  if (global_command_EMERGENCY == cmd){
    return global_state_DISARMING45;
  }

  
  // Invariant: guidance
  if ((guidance_state_READY != g1_state
      || guidance_state_READY != g2_state) && !DISABLE_GUIDANCE_SUBSYSTEM){
    return global_state_DISARMING45;
  }

  // Invariant: levitation
  if ((levitation_state_CONTROL != l1_state
      || levitation_state_CONTROL != l2_state
      || levitation_state_CONTROL != l3_state) && !DISABLE_LEVITATION_SUBSYSTEM){
    return global_state_DISARMING45;
  }

  // Invariant: motor
  if (motor_state_READY != motor_state && !DISABLE_MOTOR_SUBSYSTEM){
    return global_state_DISARMING45;
  }

  // Invariant: input board 
  if (input_board_state_RUNNING != input_state && !DISABLE_INPUT_SUBSYSTEM){
    return global_state_DISARMING45;
  }

  // Invariant: pdus 
  if ((pdu_12v_state_CHANNELS_ON != pdu12_state
      || pdu_24v_state_CHANNELS_ON != pdu24_state) && !DISABLE_POWER_SUBSYSTEM){
    return global_state_DISARMING45;
  }

  // Invariant: SDC
  if (sdc::status() == sdc_status_OPEN){
    return global_state_DISARMING45;
  }


  if (global_command_STOP_45 == cmd){
    return global_state_DISARMING45;
  }

  if (time_since_last_transition > STATE_TIMEOUT){
    return global_state_STOP_LEVITATION;
  }

  if (global_command_STOP_LEVITATION == cmd || global_command_ABORT == cmd){
    return global_state_STOP_LEVITATION;
  }

  if (time_since_last_transition > MIN_STABLE_TIME){
    return global_state_START_GUIDANCE;
  }

  // ===================== OUTPUT =======================
  canzero_set_guidance_command(guidance_command_NONE);
  canzero_set_levitation_command(levitation_command_NONE);
  canzero_set_motor_driver_command(motor_command_NONE);
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_power_board12_command(pdu_12v_command_NONE);
  canzero_set_power_board24_command(pdu_24v_command_NONE);
  canzero_set_input_board_assert_45V_online(bool_t_TRUE);
  control::velocity::disable();

  return global_state_LEVITATION_STABLE;
}
