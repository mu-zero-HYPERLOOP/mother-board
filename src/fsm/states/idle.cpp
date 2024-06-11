#include "canzero.h"
#include "control/velocity.h"
#include "fsm/states.h"
#include "subsystems.h"

// Invariants:
// - All system in idle!
global_state fsm::states::idle(global_command cmd,
                               Duration time_since_last_transition) {

  const input_board_state input_state = canzero_get_input_board_state();
  const pdu_state pdu24_state = canzero_get_power_board24_state();
  const pdu_state pdu12_state = canzero_get_power_board12_state();

  const guidance_state g1_state = canzero_get_guidance_board_front_state();
  const guidance_state g2_state = canzero_get_guidance_board_back_state();

  const levitation_state l1_state = canzero_get_levitation_board1_state();
  const levitation_state l2_state = canzero_get_levitation_board2_state();
  const levitation_state l3_state = canzero_get_levitation_board3_state();

  const motor_state motor_state = canzero_get_motor_driver_state();

  if (global_command_SHUTDOWN == cmd){
    return global_state_SHUTDOWN;
  }

  // Invariant: guidance
  if ((guidance_state_IDLE != g1_state || guidance_state_IDLE != g2_state) &&
      !DISABLE_GUIDANCE_SUBSYSTEM) {
    return global_state_SHUTDOWN;
  }

  // Invariant: levitation
  if ((levitation_state_IDLE != l1_state || levitation_state_IDLE != l2_state ||
       levitation_state_IDLE != l3_state) &&
      !DISABLE_LEVITATION_SUBSYSTEM) {
    return global_state_SHUTDOWN;
  }
  // Invariant: motor
  if (motor_state_IDLE != motor_state && !DISABLE_MOTOR_SUBSYSTEM) {
    return global_state_SHUTDOWN;
  }

  // Invariant: input board
  if (input_board_state_RUNNING != input_state && !DISABLE_INPUT_SUBSYSTEM) {
    return global_state_SHUTDOWN;
  }

  if ((pdu_state_RUNNING != pdu12_state || pdu_state_RUNNING != pdu24_state) &&
      !DISABLE_POWER_SUBSYSTEM) {
    return global_state_SHUTDOWN;
  }

  // ================ TRANSITIONS ==================
  if (global_command_START_45 == cmd) {
    return global_state_ARMING45;
  }

  // ================== OUTPUT =====================
  canzero_set_guidance_command(guidance_command_NONE);
  canzero_set_levitation_command(levitation_command_NONE);
  canzero_set_motor_driver_command(motor_command_NONE);
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_input_board_assert_45V_online(bool_t_FALSE);
  control::velocity::disable();

  return global_state_IDLE;
}
