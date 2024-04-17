
#include "canzero.h"
#include "states.h"

global_state disconnecting_state_next(global_command cmd, Duration time_since_last_transition) {
  
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

  return global_state_DISCONNECTING;
}
