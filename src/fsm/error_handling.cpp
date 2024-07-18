#include "fsm/error_handling.h"
#include "canzero/canzero.h"
#include <algorithm>
#include <array>
#include <cstdint>
#include <iostream>

global_command fsm::error_handling::approve(global_command cmd) {
  
  // === Config Consistency ===
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

  // === MCU Over-Temperatures ===
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
  const auto mcu_temp_error_level_it = std::max_element(mcu_overtemps.begin(), 
      mcu_overtemps.end());
  const error_level max_mcu_overtemp = *mcu_temp_error_level_it;

  // === External Temperatures ===
  const auto ext_temps = std::array<error_level, 16> {
    canzero_get_levitation_board1_error_level_magnet_temperature_left(),
    canzero_get_levitation_board1_error_level_magnet_temperature_right(),
    canzero_get_levitation_board2_error_level_magnet_temperature_left(),
    canzero_get_levitation_board2_error_level_magnet_temperature_right(),
    canzero_get_levitation_board3_error_level_magnet_temperature_left(),
    canzero_get_levitation_board3_error_level_magnet_temperature_right(),
    canzero_get_guidance_board_front_error_level_magnet_temperature_left(),
    canzero_get_guidance_board_front_error_level_magnet_temperature_right(),
    canzero_get_guidance_board_back_error_level_magnet_temperature_left(),
    canzero_get_guidance_board_back_error_level_magnet_temperature_right(),
    canzero_get_motor_driver_error_level_lim_temperature(),
    canzero_get_motor_driver_error_level_board_temperature(),
    canzero_get_input_board_error_level_sac_ebox_temperature(),
    canzero_get_input_board_error_level_power_ebox_temperature(),
    canzero_get_input_board_error_level_ambient_temperature(),
    canzero_get_input_board_error_level_supercap_temperature(),
  };
  const auto ext_temp_error_level_it = std::max_element(ext_temps.begin(), ext_temps.end());
  const error_level max_ext_temp = *ext_temp_error_level_it;

  const auto temps_invalid = std::array<error_flag, 34> {
    canzero_get_motor_driver_error_board_temperature1_invalid(),
    canzero_get_motor_driver_error_board_temperature2_invalid(),
    canzero_get_motor_driver_error_board_temperature3_invalid(),
    canzero_get_motor_driver_error_mcu_temperature_invalid(),
    canzero_get_motor_driver_error_lim_temperature1_invalid(),
    canzero_get_motor_driver_error_lim_temperature2_invalid(),
    canzero_get_motor_driver_error_lim_temperature3_invalid(),
    canzero_get_motor_driver_error_lim_temperature4_invalid(),

    canzero_get_guidance_board_front_error_magnet_temperature_left_invalid(),
    canzero_get_guidance_board_front_error_magnet_temperature_right_invalid(),
    canzero_get_guidance_board_front_error_mcu_temperature_invalid(),

    canzero_get_guidance_board_back_error_magnet_temperature_left_invalid(),
    canzero_get_guidance_board_back_error_magnet_temperature_right_invalid(),
    canzero_get_guidance_board_back_error_mcu_temperature_invalid(),

    canzero_get_levitation_board1_error_magnet_temperature_left_invalid(),
    canzero_get_levitation_board1_error_magnet_temperature_right_invalid(),
    canzero_get_levitation_board1_error_mcu_temperature_invalid(),

    canzero_get_levitation_board2_error_magnet_temperature_left_invalid(),
    canzero_get_levitation_board2_error_magnet_temperature_right_invalid(),
    canzero_get_levitation_board2_error_mcu_temperature_invalid(),

    canzero_get_levitation_board3_error_magnet_temperature_left_invalid(),
    canzero_get_levitation_board3_error_magnet_temperature_right_invalid(),
    canzero_get_levitation_board3_error_mcu_temperature_invalid(),

    canzero_get_input_board_error_buck_temperature_invalid(),
    canzero_get_input_board_error_mcu_temperature_invalid(),
    canzero_get_input_board_error_sac_ebox_temperature_invalid(),
    canzero_get_input_board_error_power_ebox_temperature_invalid(),
    canzero_get_input_board_error_bat24_cell_temperature_1_invalid(),
    canzero_get_input_board_error_bat24_cell_temperature_2_invalid(),
    canzero_get_input_board_error_ambient_temperature_1_invalid(),
    canzero_get_input_board_error_ambient_temperature_2_invalid(),
    canzero_get_input_board_error_ambient_temperature_3_invalid(),
    canzero_get_input_board_error_supercap_temperature_invalid(),
    canzero_get_input_board_error_cooling_cycle_temperature_invalid(),
  };
  const auto temps_invalid_it = std::max_element(temps_invalid.begin(), temps_invalid.end());
  const error_flag temp_invalid = *temps_invalid_it;
  if (temp_invalid == error_flag_ERROR) {
    canzero_set_error_level_over_temperature_system(error_level_ERROR);
  } else {
    canzero_set_error_level_over_temperature_system(std::max(max_ext_temp, max_mcu_overtemp));
  }

  // === Heartbeats ===
  const auto heartbeat_misses = std::array<error_flag, 10> {
    canzero_get_error_heartbeat_miss(),
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
  const auto heartbeat_miss_it = std::max_element(heartbeat_misses.begin(), 
      heartbeat_misses.end());
  const error_flag heartbeat_miss = *heartbeat_miss_it;

  // === Remaining Error Flags ===
  const auto error_flags = std::array<error_flag, 70> {
    canzero_get_motor_driver_assertion_fault(),
    canzero_get_motor_driver_error_arming_failed(),
    canzero_get_motor_driver_error_precharge_failed(),
    canzero_get_motor_driver_error_acceleration_out_of_range(),
    canzero_get_motor_driver_error_acceleration_calibration_failed(),
    canzero_get_motor_driver_error_invalid_target_acceleration(),
    canzero_get_motor_driver_error_vdc_voltage_invalid(),

    canzero_get_guidance_board_front_assertion_fault(),
    canzero_get_guidance_board_front_error_arming_failed(),
    canzero_get_guidance_board_front_error_precharge_failed(),
    canzero_get_guidance_board_front_error_outer_airgap_left_invalid(),
    canzero_get_guidance_board_front_error_outer_airgap_right_invalid(),
    canzero_get_guidance_board_front_error_inner_airgap_left_invalid(),
    canzero_get_guidance_board_front_error_inner_airgap_right_invalid(),
    canzero_get_guidance_board_front_error_vdc_voltage_invalid(),
    canzero_get_guidance_board_front_error_magnet_current_left_invalid(),
    canzero_get_guidance_board_front_error_magnet_current_right_invalid(),
    canzero_get_guidance_board_front_error_input_current_invalid(),

    canzero_get_guidance_board_back_assertion_fault(),
    canzero_get_guidance_board_back_error_arming_failed(),
    canzero_get_guidance_board_back_error_precharge_failed(),
    canzero_get_guidance_board_back_error_outer_airgap_left_invalid(),
    canzero_get_guidance_board_back_error_outer_airgap_right_invalid(),
    canzero_get_guidance_board_back_error_inner_airgap_left_invalid(),
    canzero_get_guidance_board_back_error_inner_airgap_right_invalid(),
    canzero_get_guidance_board_back_error_vdc_voltage_invalid(),
    canzero_get_guidance_board_back_error_magnet_current_left_invalid(),
    canzero_get_guidance_board_back_error_magnet_current_right_invalid(),
    canzero_get_guidance_board_back_error_input_current_invalid(),

    canzero_get_levitation_board1_assertion_fault(),
    canzero_get_levitation_board1_error_airgap_left_invalid(),
    canzero_get_levitation_board1_error_airgap_right_invalid(),
    canzero_get_levitation_board1_error_vdc_voltage_invalid(),
    canzero_get_levitation_board1_error_magnet_current_left_invalid(),
    canzero_get_levitation_board1_error_magnet_current_right_invalid(),
    canzero_get_levitation_board1_error_arming_failed(),
    canzero_get_levitation_board1_error_precharge_failed(),

    canzero_get_levitation_board2_assertion_fault(),
    canzero_get_levitation_board2_error_airgap_left_invalid(),
    canzero_get_levitation_board2_error_airgap_right_invalid(),
    canzero_get_levitation_board2_error_vdc_voltage_invalid(),
    canzero_get_levitation_board2_error_magnet_current_left_invalid(),
    canzero_get_levitation_board2_error_magnet_current_right_invalid(),
    canzero_get_levitation_board2_error_arming_failed(),
    canzero_get_levitation_board2_error_precharge_failed(),

    canzero_get_levitation_board3_assertion_fault(),
    canzero_get_levitation_board3_error_airgap_left_invalid(),
    canzero_get_levitation_board3_error_airgap_right_invalid(),
    canzero_get_levitation_board3_error_vdc_voltage_invalid(),
    canzero_get_levitation_board3_error_magnet_current_left_invalid(),
    canzero_get_levitation_board3_error_magnet_current_right_invalid(),
    canzero_get_levitation_board3_error_arming_failed(),
    canzero_get_levitation_board3_error_precharge_failed(),

    canzero_get_input_board_assertion_fault(),
    canzero_get_input_board_error_bat24_current_invalid(),
    canzero_get_input_board_error_bat24_voltage_invalid(),
    canzero_get_input_board_error_link24_current_invalid(),
    canzero_get_input_board_error_link45_current_invalid(),
    canzero_get_input_board_error_link45_voltage_invalid(),
    canzero_get_input_board_error_supercap_voltage_invalid(),
    canzero_get_input_board_error_acceleration_out_of_range(),
    canzero_get_input_board_error_cooling_cycle_pressure_invalid(),
    canzero_get_input_board_error_acceleration_calibration_failed(),
    canzero_get_input_board_error_lateral_acceleration_out_of_range(),
    canzero_get_input_board_error_vertical_acceleration_out_of_range(),

    canzero_get_power_board12_assertion_fault(),
    canzero_get_power_board12_error_any_short(),

    canzero_get_power_board24_assertion_fault(),
    canzero_get_power_board24_error_any_short(),

    canzero_get_led_board_assertion_fault(),
  };
  const auto max_error_flag_it = std::max_element(error_flags.begin(), error_flags.end());
  const error_flag max_error_flag = *max_error_flag_it;

  // === Remaining Error Levels ===
  const auto error_levels = std::array<error_level, 36> {
    canzero_get_motor_driver_error_level_current_u1(),
    canzero_get_motor_driver_error_level_current_u2(),
    canzero_get_motor_driver_error_level_current_v1(),
    canzero_get_motor_driver_error_level_current_v2(),
    canzero_get_motor_driver_error_level_current_w1(),
    canzero_get_motor_driver_error_level_current_w2(),
    canzero_get_motor_driver_error_level_vdc_voltage(),

    canzero_get_guidance_board_front_error_level_vdc_voltage(),
    canzero_get_guidance_board_front_error_level_input_current(),
    canzero_get_guidance_board_front_error_level_magnet_current_left(),
    canzero_get_guidance_board_front_error_level_magnet_current_right(),

    canzero_get_guidance_board_back_error_level_vdc_voltage(),
    canzero_get_guidance_board_back_error_level_magnet_current_left(),
    canzero_get_guidance_board_back_error_level_magnet_current_right(),

    canzero_get_levitation_board1_error_level_vdc_voltage(),
    canzero_get_levitation_board1_error_level_input_current(),
    canzero_get_levitation_board1_error_level_magnet_current_left(),
    canzero_get_levitation_board1_error_level_magnet_current_right(),

    canzero_get_levitation_board2_error_level_vdc_voltage(),
    canzero_get_levitation_board2_error_level_input_current(),
    canzero_get_levitation_board2_error_level_magnet_current_left(),
    canzero_get_levitation_board2_error_level_magnet_current_right(),

    canzero_get_levitation_board3_error_level_vdc_voltage(),
    canzero_get_levitation_board3_error_level_input_current(),
    canzero_get_levitation_board3_error_level_magnet_current_left(),
    canzero_get_levitation_board3_error_level_magnet_current_right(),

    canzero_get_input_board_error_level_bat24_over_current(),
    canzero_get_input_board_error_level_bat24_over_voltage(),
    canzero_get_input_board_error_level_bat24_under_voltage(),
    canzero_get_input_board_error_level_link24_over_current(),
    canzero_get_input_board_error_level_link45_over_current(),
    canzero_get_input_board_error_level_link45_over_voltage(),
    canzero_get_input_board_error_level_link45_under_voltage(),
    canzero_get_input_board_error_level_supercap_over_voltage(),
    canzero_get_input_board_error_level_supercap_under_voltage(),
    canzero_get_input_board_error_level_cooling_cycle_over_pressure(),
  };
  const auto error_level_it = std::max_element(error_levels.begin(), error_levels.end());
  const error_level max_error_level = *error_level_it;

  bool force_abort = false;

  switch (max_mcu_overtemp) {
  case error_level_OK:
  case error_level_INFO:
    break;
  case error_level_WARNING:
    std::cout << "ERROR_CMD: ABORT (MCU_OVERTEMP)" << std::endl;
    canzero_set_command(global_command_NONE);
    force_abort = true;
    break;
  case error_level_ERROR:
    std::cout << "ERROR_CMD: SHUTDOWN (MCU_OVERTEMP)" << std::endl;
    canzero_set_command(global_command_NONE);
    return global_command_EMERGENCY;
  }

  switch (max_ext_temp) {
  case error_level_OK:
  case error_level_INFO:
    break;
  case error_level_WARNING:
    std::cout << "ERROR_CMD: ABORT (EXT_TEMP)" << std::endl;
    canzero_set_command(global_command_NONE);
    force_abort = true;
    break;
  case error_level_ERROR:
    std::cout << "ERROR_CMD: EMERGENCY (EXT_TEMP)" << std::endl;
    canzero_set_command(global_command_NONE);
    return global_command_EMERGENCY;
  }

  if (temp_invalid == error_flag_ERROR) {
    std::cout << "ERROR_CMD: ABORT (TEMP_INVALID)" << std::endl;
    canzero_set_command(global_command_NONE);
    return global_command_ABORT;
  }

  if (heartbeat_miss == error_flag_ERROR) {
    /* std::cout << "ERROR_CMD: EMERGENCY (HEARTBEAT_MISS)" << std::endl; */
    canzero_set_command(global_command_NONE);
    return global_command_EMERGENCY;
  }

  if (max_error_flag == error_flag_ERROR) {
    /* std::cout << "ERROR_CMD: EMERGENCY (FLAG)" << std::endl; */
    canzero_set_command(global_command_NONE);
    return global_command_EMERGENCY;
  }

  switch (max_error_level) {
  case error_level_OK:
  case error_level_INFO:
    if (force_abort){
      return global_command_ABORT;
    }
    return cmd;
  case error_level_WARNING:
    std::cout << "ERROR_CMD: ABORT (LEVEL)" << std::endl;
    canzero_set_command(global_command_NONE);
    return global_command_ABORT;
  case error_level_ERROR:
    std::cout << "ERROR_CMD: EMERGENCY (LEVEL)" << std::endl;
    canzero_set_command(global_command_NONE);
    return global_command_EMERGENCY;
  }


  return cmd;
}

global_state fsm::error_handling::invariant_broken() {
  canzero_set_command(global_command_NONE);
  return global_state_DISARMING45;
}

global_state fsm::error_handling::invariant_broken_idle() {
  canzero_set_command(global_command_NONE);
  return global_state_INIT;
}
