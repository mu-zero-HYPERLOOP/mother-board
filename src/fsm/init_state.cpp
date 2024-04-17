
#include "canzero.h"
#include "states.h"

global_state init_state_next(global_command cmd,
                             Duration time_since_last_transition) {

  // transitions
  if (canzero_get_mlu1_state() == mlu_state_IDLE &&
      canzero_get_mlu2_state() == mlu_state_IDLE &&
      canzero_get_mlu3_state() == mlu_state_IDLE &&
      canzero_get_mlu4_state() == mlu_state_IDLE &&
      canzero_get_mlu5_state() == mlu_state_IDLE &&
      canzero_get_mlu6_state() == mlu_state_IDLE &&
      canzero_get_mgu1_state() == mgu_state_IDLE &&
      canzero_get_mgu2_state() == mgu_state_IDLE &&
      canzero_get_motor_state() == motor_state_IDLE &&
      canzero_get_pdu12_state() == pdu_state_RUNNING &&
      canzero_get_pdu24_state() == pdu_state_RUNNING &&
      canzero_get_input_board_state() == input_board_state_RUNNING) {
    return global_state_IDLE;
  }

  // actions
  canzero_set_mlu_command(mlu_command_DISCONNECT);
  canzero_set_mgu_command(mgu_command_DISCONNECT);
  canzero_set_motor_command(motor_command_DISCONNECT);

  return global_state_INIT;
}
