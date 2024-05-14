#pragma once
#include "canzero/canzero.h"
#include "util/timestamp.h"

global_state state_init_next(global_command command, Duration time_since_last_transition);

global_state state_idle_next(global_command command, Duration time_since_last_transition);

global_state state_precharge_next(global_command command, Duration time_since_last_transition);

global_state state_ready_next(global_command command, Duration time_since_last_transition);

global_state state_disconnecting_next(global_command command, Duration time_since_last_transition);

global_state state_start_levitation_next(global_command command, Duration time_since_last_transition);

global_state state_levitation_stable_next(global_command command, Duration time_since_last_transition);

global_state state_start_guidance_next(global_command command, Duration time_since_last_transition);

global_state state_guidance_stable_next(global_command command, Duration time_since_last_transition);

global_state state_stop_levitation_next(global_command command, Duration time_since_last_transition);

global_state state_acceleration_next(global_command command, Duration time_since_last_transition);

global_state state_cruising_next(global_command command, Duration time_since_last_transition);

global_state state_deceleration_next(global_command command, Duration time_since_last_transition);

