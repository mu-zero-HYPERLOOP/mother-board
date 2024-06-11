#include "canzero.h"
#include "fsm/states.h"
#include "timestamp.h"

constexpr Duration POWER_OF_TIME = 3_s;

global_state fsm::states::restarting(global_command cmd,
                                Duration time_since_last_transition) {

  if (time_since_last_transition > POWER_OF_TIME){
    return global_state_INIT;
  }
  // =============== OUTPUT ================

  // TODO disable all power channels excluding the pi and the anthena!

  return global_state_RESTARTING;
}
