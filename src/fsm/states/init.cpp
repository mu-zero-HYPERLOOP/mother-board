#include "canzero.h"
#include "control/velocity.h"
#include "fsm/states.h"
#include "invariants.h"
#include "subsystems.h"
#include <array>


constexpr std::array<pdu_12v_state, 2> ALLOWED_PDU_12V_STATES = {
    pdu_12v_state_CHANNELS_ON, pdu_12v_state_CHANNELS_TELEMETRY};

// Invariants: none
// Exit condition: All system in idle.
global_state fsm::states::init(global_command cmd,
                               Duration time_since_last_transition) {

  using namespace fsm::invariant;

  const input_board_state input_board_state = canzero_get_input_board_state();
  const pdu_24v_state pdu24_state = canzero_get_power_board24_state();
  const pdu_12v_state pdu12_state = canzero_get_power_board12_state();

  const guidance_state g1_state = canzero_get_guidance_board_front_state();
  const guidance_state g2_state = canzero_get_guidance_board_back_state();

  const levitation_state l1_state = canzero_get_levitation_board1_state();
  const levitation_state l2_state = canzero_get_levitation_board2_state();
  const levitation_state l3_state = canzero_get_levitation_board3_state();

  if (global_command_SHUTDOWN == cmd){
    return global_state_SHUTDOWN;
  }

  if ((input_board_state_RUNNING == input_board_state ||
       DISABLE_INPUT_SUBSYSTEM) &&

      ((pdu_24v_state_CHANNELS_IDLE == pdu24_state && 
        pdu_12v_state_CHANNELS_ON == pdu12_state) ||
       DISABLE_POWER_SUBSYSTEM) &&

      ((g1_state == guidance_state_IDLE && g2_state == guidance_state_IDLE) ||
       DISABLE_GUIDANCE_SUBSYSTEM) &&

      ((l1_state == levitation_state_IDLE &&
        l2_state == levitation_state_IDLE &&
        l3_state == levitation_state_IDLE) ||
       DISABLE_LEVITATION_SUBSYSTEM)) {

    return global_state_IDLE;
  }

  canzero_set_levitation_command(levitation_command_DISARM45);
  canzero_set_guidance_command(guidance_command_DISARM45);
  canzero_set_motor_driver_command(motor_command_DISARM45);
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_power_board12_command(pdu_12v_command_START);
  canzero_set_power_board24_command(pdu_24v_command_IDLE);
  canzero_set_input_board_assert_45V_online(bool_t_FALSE);
  control::velocity::disable();

  return global_state_INIT;
}
