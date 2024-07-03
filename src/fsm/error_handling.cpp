#include "fsm/error_handling.h"
#include "canzero/canzero.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>

global_command fsm::error_handling::approve(global_command cmd) {
  // check config hashes of nodes
  const auto config_hashes = std::array<uint64_t, 10> {
    canzero_get_motor_driver_config_hash(),
    canzero_get_guidance_board_front_config_hash(),
    canzero_get_guidance_board_back_config_hash(),
    canzero_get_levitation_board1_config_hash(),
    canzero_get_levitation_board2_config_hash(),
    canzero_get_levitation_board3_config_hash(),
    canzero_get_input_board_config_hash(),
    canzero_get_power_board12_config_hash(),
    canzero_get_power_board24_config_hash(),
    canzero_get_led_board_config_hash(),
  };
  const uint64_t mother_hash = canzero_get_config_hash();
  const bool inconsistent = std::any_of(config_hashes.begin(), config_hashes.end(), [mother_hash](auto x) {
      return x != mother_hash && x != 0;
  });
  canzero_set_error_level_config_consistency(inconsistent ? error_level_INFO : error_level_OK);

  if (canzero_get_error_heartbeat_miss() == error_flag_ERROR){
    std::cout << "ERROR_CMD: RESTART" << std::endl;
    return global_command_RESTART;
  }


  const auto error_flags = std::array<error_flag, 1> {
    canzero_get_error_heartbeat_miss(),
  };

  const auto max_error_flag_it = std::max_element(error_flags.begin(), error_flags.end());
  const error_flag max_error_flag = *max_error_flag_it;
  if (max_error_flag == error_flag_ERROR) {
    std::cout << "ERROR_CMD: EMERGENCY" << std::endl;
    return global_command_EMERGENCY;
  }

  const auto mcu_overtemps = std::array<error_level, 9> {
    canzero_get_power_board12_error_level_mcu_temperature(),
    canzero_get_power_board24_error_level_mcu_temperature(),
    canzero_get_guidance_board_front_error_level_mcu_temperature(),
    canzero_get_guidance_board_back_error_level_mcu_temperature(),
    canzero_get_input_board_error_level_mcu_temperature(),
    canzero_get_levitation_board1_error_level_mcu_temperature(),
    canzero_get_levitation_board2_error_level_mcu_temperature(),
    canzero_get_levitation_board3_error_level_mcu_temperature(),
    canzero_get_motor_driver_error_level_mcu_temperature(),
  };
  const auto mcu_temp_error_level_it = std::max_element(mcu_overtemps.begin(), mcu_overtemps.end());
  const error_level max_mcu_overtemp = *mcu_temp_error_level_it;
  switch (max_mcu_overtemp) {
  case error_level_OK:
  case error_level_INFO:
    return cmd;
  case error_level_WARNING:
    std::cout << "ERROR_CMD: ABORT" << std::endl;
    return global_command_ABORT;
  case error_level_ERROR:
    std::cout << "ERROR_CMD: EMERGENCY" << std::endl;
    return global_command_EMERGENCY;
  }

  const auto heartbeat_misses = std::array<error_flag, 9> {
    canzero_get_power_board12_error_heartbeat_miss(),
    canzero_get_power_board24_error_heartbeat_miss(),
    canzero_get_guidance_board_front_error_heartbeat_miss(),
    canzero_get_guidance_board_back_error_heartbeat_miss(),
    canzero_get_input_board_error_heartbeat_miss(),
    canzero_get_levitation_board1_error_heartbeat_miss(),
    canzero_get_levitation_board2_error_heartbeat_miss(),
    canzero_get_levitation_board3_error_heartbeat_miss(),
    canzero_get_motor_driver_error_heartbeat_miss(),
  };
  const auto heartbeat_miss_it = std::max_element(heartbeat_misses.begin(), heartbeat_misses.end());
  const error_flag heartbeat_miss = *heartbeat_miss_it;
  if (heartbeat_miss == error_flag_ERROR) {
    std::cout << "ERROR_CMD: EMERGENCY" << std::endl;
    return global_command_EMERGENCY;
  }

  return cmd;
}
