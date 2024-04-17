
#include "canzero.h"
#include "states.h"

static constexpr Duration STATE_TIMEOUT = 1_s;

global_state precharge_state_next(global_command cmd,
                                  Duration time_since_last_transition) {

  if (cmd == global_command_ABORT || cmd == global_command_DISCONNECT ||
      cmd == global_command_EMERGENCY) {
    return global_state_DISCONNECTING;
  }
  

  if ((canzero_get_mlu1_state() != mlu_state_IDLE &&
       canzero_get_mlu1_state() != mlu_state_PRECHARGE &&
       canzero_get_mlu1_state() != mlu_state_READY) ||
      (canzero_get_mlu2_state() != mlu_state_IDLE &&
       canzero_get_mlu2_state() != mlu_state_PRECHARGE &&
       canzero_get_mlu2_state() != mlu_state_READY) ||
      (canzero_get_mlu3_state() != mlu_state_IDLE &&
       canzero_get_mlu3_state() != mlu_state_PRECHARGE &&
       canzero_get_mlu3_state() != mlu_state_READY) ||
      (canzero_get_mlu4_state() != mlu_state_IDLE &&
       canzero_get_mlu4_state() != mlu_state_PRECHARGE &&
       canzero_get_mlu4_state() != mlu_state_READY) ||
      (canzero_get_mlu5_state() != mlu_state_IDLE &&
       canzero_get_mlu5_state() != mlu_state_PRECHARGE &&
       canzero_get_mlu5_state() != mlu_state_READY) ||
      (canzero_get_mlu6_state() != mlu_state_IDLE &&
       canzero_get_mlu6_state() != mlu_state_PRECHARGE &&
       canzero_get_mlu6_state() != mlu_state_READY) ||
      (canzero_get_mgu1_state() != mgu_state_IDLE &&
       canzero_get_mgu1_state() != mgu_state_PRECHARGE &&
       canzero_get_mgu1_state() != mgu_state_READY) ||
      (canzero_get_mgu2_state() != mgu_state_IDLE &&
       canzero_get_mgu2_state() != mgu_state_PRECHARGE &&
       canzero_get_mgu2_state() != mgu_state_READY) ||
      (canzero_get_motor_state() != motor_state_IDLE &&
       canzero_get_motor_state() != motor_state_PRECHARGE &&
       canzero_get_motor_state() != motor_state_READY)) {
    return global_state_DISCONNECTING;
  }

  if (time_since_last_transition < STATE_TIMEOUT) {
    return global_state_DISCONNECTING;
  }

  if (canzero_get_mlu1_state() == mlu_state_READY &&
      canzero_get_mlu2_state() == mlu_state_READY &&
      canzero_get_mlu3_state() == mlu_state_READY &&
      canzero_get_mlu4_state() == mlu_state_READY &&
      canzero_get_mlu5_state() == mlu_state_READY &&
      canzero_get_mlu6_state() == mlu_state_READY &&
      canzero_get_mgu1_state() == mgu_state_READY &&
      canzero_get_mgu2_state() == mgu_state_READY &&
      canzero_get_motor_state() == motor_state_READY &&
      canzero_get_pdu12_state() == pdu_state_RUNNING &&
      canzero_get_pdu24_state() == pdu_state_RUNNING &&
      canzero_get_input_board_state() == input_board_state_RUNNING) {
    return global_state_READY;
  }

  canzero_set_mlu_command(mlu_command_PRECHARGE);
  canzero_set_mgu_command(mgu_command_PRECHARGE);
  canzero_set_motor_command(motor_command_PRECHARGE);

  return global_state_PRECHARGE;
}
