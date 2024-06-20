#include "canzero.h"
#include "fsm/states.h"
#include <chrono>
#include <thread>

constexpr Duration SHUTDOWN_TIME = 5_s;

global_state fsm::states::shutdown(global_command cmd, Duration time_since_last_transition) {


  // shutdown 12V and 24V systems except for cooling and telemetry
  // and wait a bit
  if (SHUTDOWN_TIME > time_since_last_transition) {
    if (canzero_get_power_board12_state() != pdu_12v_state_CHANNELS_TELEMETRY 
          || canzero_get_power_board24_state() != pdu_24v_state_CHANNELS_IDLE) {
      canzero_set_power_board12_command(pdu_12v_command_TELEMETRY);
      canzero_set_power_board24_command(pdu_24v_command_IDLE);
    }
    return global_state_SHUTDOWN;
  }

  while (canzero_get_power_board24_state() != pdu_24v_state_CHANNELS_OFF) {
    canzero_set_power_board24_command(pdu_24v_command_STOP);
    return global_state_SHUTDOWN;
  }

  // commit suicide...
  canzero_set_power_board12_command(pdu_12v_command_STOP);

  return global_state_SHUTDOWN;
}
