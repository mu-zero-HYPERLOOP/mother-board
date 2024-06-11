#include "canzero.h"
#include "fsm/states.h"

global_state fsm::states::shutdown(global_command cmd, Duration time_since_last_transition) {

  // TODO disable all PDUs channels (not the pi)

  // Once all subsystems are offline shutdown complete pi!

  return global_state_SHUTDOWN;
}
