#include "canzero.h"
#include "control/velocity.h"
#include "fsm/states.h"
#include "sdc.h"
#include "subsystems.h"

// if in disarming for this time shutdown the complete pod! (include 24V & 12V).
constexpr Duration SHUTDOWN_PROMOPTION_TIMEOUT = 5_s;

// Invariants:
// - none
// Exit condition:
// - all systems in idle.
global_state fsm::states::disarming45(global_command cmd,
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

  // =================== TRANSITIONS ==================
  if (global_command_SHUTDOWN == cmd){
    return global_state_SHUTDOWN;
  }

  if (time_since_last_transition > SHUTDOWN_PROMOPTION_TIMEOUT){
    return global_state_SHUTDOWN;
  }

  if ((input_board_state_RUNNING == input_state || DISABLE_INPUT_SUBSYSTEM) &&

      ((pdu_12v_state_CHANNELS_ON == pdu12_state && pdu_24v_state_CHANNELS_IDLE == pdu24_state) ||
       DISABLE_POWER_SUBSYSTEM) &&

      ((guidance_state_IDLE == g1_state && guidance_state_IDLE == g2_state) ||
       DISABLE_GUIDANCE_SUBSYSTEM) &&

      ((levitation_state_IDLE == l1_state &&
        levitation_state_IDLE == l2_state &&
        levitation_state_IDLE == l3_state) ||
       DISABLE_LEVITATION_SUBSYSTEM) &&

      (motor_state_IDLE == motor_state || DISABLE_MOTOR_SUBSYSTEM) &&

      sdc::status() == sdc_status_OPEN) {
    return global_state_IDLE;
  }

  // ============== OUTPUTS ===============
  canzero_set_guidance_command(guidance_command_DISARM45);
  canzero_set_levitation_command(levitation_command_DISARM45);
  canzero_set_motor_driver_command(motor_command_DISARM45);
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_power_board12_command(pdu_12v_command_NONE);
  canzero_set_power_board24_command(pdu_24v_command_IDLE);
  canzero_set_input_board_assert_45V_online(bool_t_FALSE);
  control::velocity::disable();

  return global_state_DISARMING45;
}
