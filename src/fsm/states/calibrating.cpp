#include "canzero.h"
#include "fsm/states.h"
#include "subsystems.h"

// NOTE this is a bad hack!
constexpr Duration START_CALIBRATION = 1_s;

global_state fsm::states::calibrating(global_command cmd,
                                   Duration time_since_last_transition) {

  if (DISABLE_INPUT_SUBSYSTEM){
    return global_state_IDLE;
  }

  if (time_since_last_transition < START_CALIBRATION){
    canzero_set_input_board_command(input_board_command_CALIBRATE);
  }else {
    canzero_set_input_board_command(input_board_command_NONE);
    
    if (input_board_state_RUNNING == canzero_get_input_board_state()){
      return global_state_IDLE;
    }

  }

  canzero_set_input_board_assert_45V_online(bool_t_FALSE);

  canzero_set_pod_grounded(bool_t_TRUE);

  return global_state_CALIBRATING;
}
