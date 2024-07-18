#pragma once

#include "canzero/canzero.h"

static void can_defaults() {
  canzero_set_error_level_config_consistency(error_level_OK);
  canzero_set_error_heartbeat_miss(error_flag_OK);
  canzero_set_error_any(error_flag_OK);
  canzero_set_last_node_missed(255);



}
