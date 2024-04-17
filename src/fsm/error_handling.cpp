#include "error_handling.h"
#include "canzero.h"

global_command handle_errors(global_state state, global_command cmd,
                             Duration time_since_last_transition) {

  // input-board errors.
  if (canzero_get_error_input_board_mcu_over_temperature()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_invalid_position_estimation()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_invalid_position()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_invalid_acceleration_profile()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_battery_over_voltage()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_cooling_cycle_over_pressure()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_cooling_cycle_low_pressure()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_ebox_over_temperature()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_cooling_cycle_over_temperature()) {
    return global_command_EMERGENCY;
  }

  // motor errors

  if (canzero_get_error_motor_driver_45V_over_voltage()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_motor_driver_45V_under_voltage()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_dslim_over_temperature()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_motor_driver_mosfet_over_temperature()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_motor_control_failure()) {
    return global_command_EMERGENCY;
  }

  // mgu errors
  if (canzero_get_error_mgu1_45V_over_voltage() ||
      canzero_get_error_mgu2_45V_over_voltage()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mgu1_45V_under_voltage() ||
      canzero_get_error_mgu2_45V_under_voltage()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mgu1_control_error() ||
      canzero_get_error_mgu2_control_error()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mgu1_magnet_over_temperature_starboard() ||
      canzero_get_error_mgu1_magnet_over_temperature_port() ||
      canzero_get_error_mgu2_magnet_over_temperature_starboard() ||
      canzero_get_error_mgu2_magnet_over_temperature_port()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mgu1_mosfet_over_temperature() ||
      canzero_get_error_mgu2_mosfet_over_temperature()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mgu1_mcu_over_temperature() ||
      canzero_get_error_mgu2_mcu_over_temperature()) {
    return global_command_EMERGENCY;
  }

  // mlu errors.
  if (canzero_get_error_mlu1_45V_over_voltage() ||
      canzero_get_error_mlu2_45V_over_voltage() ||
      canzero_get_error_mlu3_45V_over_voltage() ||
      canzero_get_error_mlu4_45V_over_voltage() ||
      canzero_get_error_mlu5_45V_over_voltage() ||
      canzero_get_error_mlu6_45V_over_voltage()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mlu1_45V_under_voltage() ||
      canzero_get_error_mlu2_45V_under_voltage() ||
      canzero_get_error_mlu3_45V_under_voltage() ||
      canzero_get_error_mlu4_45V_under_voltage() ||
      canzero_get_error_mlu5_45V_under_voltage() ||
      canzero_get_error_mlu6_45V_under_voltage()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mlu1_control_failure() ||
      canzero_get_error_mlu2_control_failure() ||
      canzero_get_error_mlu3_control_failure() ||
      canzero_get_error_mlu4_control_failure() ||
      canzero_get_error_mlu5_control_failure() ||
      canzero_get_error_mlu6_control_failure()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mlu1_mcu_over_temperature() ||
      canzero_get_error_mlu2_mcu_over_temperature() ||
      canzero_get_error_mlu3_mcu_over_temperature() ||
      canzero_get_error_mlu4_mcu_over_temperature() ||
      canzero_get_error_mlu5_mcu_over_temperature() ||
      canzero_get_error_mlu6_mcu_over_temperature()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mlu1_magnet_over_temperature() ||
      canzero_get_error_mlu2_magnet_over_temperature() ||
      canzero_get_error_mlu3_magnet_over_temperature() ||
      canzero_get_error_mlu4_magnet_over_temperature() ||
      canzero_get_error_mlu5_magnet_over_temperature() ||
      canzero_get_error_mlu6_magnet_over_temperature()) {
    return global_command_EMERGENCY;
  }

  if (canzero_get_error_mlu1_mosfet_over_temperature() ||
      canzero_get_error_mlu2_mosfet_over_temperature() ||
      canzero_get_error_mlu3_mosfet_over_temperature() ||
      canzero_get_error_mlu4_mosfet_over_temperature() ||
      canzero_get_error_mlu5_mosfet_over_temperature() ||
      canzero_get_error_mlu6_mosfet_over_temperature()) {
    return global_command_EMERGENCY;
  }

  // pdu12 shorts
  if (canzero_get_pdu12_lp_channel1_status() ==
      pdu_channel_status_SHORT_CIRCUIT) {
    return global_command_EMERGENCY;
  }
  // ...

  if (canzero_get_pdu24_lp_channel1_status() ==
      pdu_channel_status_SHORT_CIRCUIT) {
    return global_command_EMERGENCY;
  }
  //..

  // WARNINGS
  if (canzero_get_warn_invalid_position_estimation()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_invalid_position()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_invalid_velocity_profile()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_invalid_acceleration_profile()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_battery_over_temperature()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_cooling_cycle_over_pressure()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_cooling_cycle_low_pressure()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_cooling_cycle_over_temperature()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_input_board_mcu_over_temperature()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_ebox_over_temperature()) {
    return global_command_ABORT;
  }

  // Motor warnings.
  if (canzero_get_warn_motor_dslim_over_temperature()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_motor_mosfet_over_temperature()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_motor_mcu_over_temperature()) {
    return global_command_ABORT;
  }

  // MGU warnings.
  if (canzero_get_warn_mgu1_mcu_over_temperature() ||
      canzero_get_warn_mgu2_mcu_over_temperature()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_mgu1_mosfet_over_temperature() ||
      canzero_get_warn_mgu2_mosfet_over_temperature()) {
    return global_command_ABORT;
  }
  if (canzero_get_warn_mgu1_magnet_over_temperature_starboard() ||
      canzero_get_warn_mgu1_magnet_over_temperature_port() ||
      canzero_get_warn_mgu2_magnet_over_temperature_starboard() ||
      canzero_get_warn_mgu2_magnet_over_temperature_port()) {
    return global_command_ABORT;
  }

  // MLU warnings.
  if (canzero_get_warn_mlu1_mcu_over_temperature()
      || canzero_get_warn_mlu2_mcu_over_temperature()
      || canzero_get_warn_mlu3_mcu_over_temperature()
      || canzero_get_warn_mlu4_mcu_over_temperature()
      || canzero_get_warn_mlu5_mcu_over_temperature()
      || canzero_get_warn_mlu6_mcu_over_temperature()) {
    return global_command_ABORT;
  }
  if(canzero_get_warn_mlu1_magnet_over_temperature()
      || canzero_get_warn_mlu2_magnet_over_temperature()
      || canzero_get_warn_mlu3_magnet_over_temperature()
      || canzero_get_warn_mlu4_magnet_over_temperature()
      || canzero_get_warn_mlu5_magnet_over_temperature()
      || canzero_get_warn_mlu6_magnet_over_temperature()) {
    return global_command_ABORT;
  }

  if(canzero_get_warn_mlu1_mosfet_over_temperature()
      || canzero_get_warn_mlu2_mosfet_over_temperature()
      || canzero_get_warn_mlu3_mosfet_over_temperature()
      || canzero_get_warn_mlu4_mosfet_over_temperature()
      || canzero_get_warn_mlu5_mosfet_over_temperature()
      || canzero_get_warn_mlu6_mosfet_over_temperature()) {
    return global_command_ABORT;
  }

  return cmd;
}
