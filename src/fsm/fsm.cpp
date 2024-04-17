
#include "canzero.h"

#include "error_handling.h"
#include "states.h"
#include "util/timestamp.h"

Timestamp g_fsm_last_transition = Timestamp::now();

void fsm_init() {
  g_fsm_last_transition = Timestamp::now();
  canzero_set_global_state(global_state_INIT);
}

void fsm_next() {

  Timestamp now = Timestamp::now();
  Duration time_since_last_transition = now - g_fsm_last_transition;

  global_state state = canzero_get_global_state();
  global_command cmd =
      handle_errors(state, canzero_get_command(), time_since_last_transition);

  global_state next_state;

  switch (state) {
  case global_state_INIT:
    next_state = init_state_next(cmd, time_since_last_transition);
    break;
  case global_state_IDLE:
    next_state = idle_state_next(cmd, time_since_last_transition);
    break;
  case global_state_DISCONNECTING:
    next_state = disconnecting_state_next(cmd, time_since_last_transition);
    break;
  case global_state_PRECHARGE:
    next_state = precharge_state_next(cmd, time_since_last_transition);
    break;
  case global_state_READY:
    next_state = ready_state_next(cmd, time_since_last_transition);
    break;
  case global_state_START_LEVITATION:
    next_state = start_levitation_state_next(cmd, time_since_last_transition);
    break;
  case global_state_LEVITATION_STABLE:
    next_state = levitation_stable_state_next(cmd, time_since_last_transition);
    break;
  case global_state_START_GUIDANCE:
    next_state = start_guidance_state_next(cmd, time_since_last_transition);
    break;
  case global_state_GUIDANCE_STABLE:
    next_state = guidance_stable_state_next(cmd, time_since_last_transition);
    break;
  case global_state_ACCELERATION:
    next_state = acceleration_state_next(cmd, time_since_last_transition);
    break;
  case global_state_CRUISING:
    next_state = cruising_state_next(cmd, time_since_last_transition);
    break;
  case global_state_DECELERATION:
    next_state = deceleration_state_next(cmd, time_since_last_transition);
    break;
  case global_state_STOP_LEVITATION:
    next_state = stop_levitation_state_next(cmd, time_since_last_transition);
    break;
  }
  if (next_state != state) {
    g_fsm_last_transition = now;
    canzero_set_global_state(next_state);
  }
}
