#pragma once


#include "canzero.h"
#include "timestamp.h"


global_state init_state_next(global_command cmd, Duration time_since_last_transition);
global_state idle_state_next(global_command cmd, Duration time_since_last_transition);
global_state disconnecting_state_next(global_command cmd, Duration time_since_last_transition);
global_state precharge_state_next(global_command cmd, Duration time_since_last_transition);
global_state ready_state_next(global_command cmd, Duration time_since_last_transition);
global_state start_levitation_state_next(global_command cmd, Duration time_since_last_transition);
global_state levitation_stable_state_next(global_command cmd, Duration time_since_last_transition);
global_state start_guidance_state_next(global_command cmd, Duration time_since_last_transition);
global_state guidance_stable_state_next(global_command cmd, Duration time_since_last_transition);
global_state acceleration_state_next(global_command cmd, Duration time_since_last_transition);
global_state cruising_state_next(global_command cmd, Duration time_since_last_transition);
global_state deceleration_state_next(global_command cmd, Duration time_since_last_transition);
global_state stop_levitation_state_next(global_command cmd, Duration time_since_last_transition);
