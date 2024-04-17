#pragma once

#include "canzero.h"
#include "timestamp.h"

global_command handle_errors(global_state state, global_command cmd, Duration time_since_last_transition);
