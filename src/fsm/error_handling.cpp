#include "fsm/error_handling.h"
#include "canzero/canzero.h"
#include <algorithm>
#include <array>

global_command fsm::error_handling::approve(global_command cmd) {

  const auto error_flags = std::array<error_flag, 1> {
    canzero_get_error_heartbeat_miss(),
  };

  const auto max_error_flag_it = std::max_element(error_flags.begin(), error_flags.end());
  const error_flag max_error_flag = *max_error_flag_it;
  if (max_error_flag == error_flag_ERROR) {
    return global_command_SHUTDOWN;
  }

  return cmd;
}
