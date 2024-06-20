#include "canzero.h"
#include "fsm/states.h"
#include "timestamp.h"

constexpr Duration POWER_OFF_TIME = 3_s;

global_state fsm::states::restarting(global_command cmd,
                                Duration time_since_last_transition) {

  if (time_since_last_transition > POWER_OFF_TIME){
    return global_state_INIT;
  }
  // =============== OUTPUT ================

  canzero_set_power_board12_command(pdu_12v_command_TELEMETRY);
  canzero_set_power_board24_command(pdu_24v_command_IDLE);

  return global_state_RESTARTING;
}
