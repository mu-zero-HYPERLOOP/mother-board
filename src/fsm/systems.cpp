#include "systems.h"
#include "canzero.h"

System<levitation_state, 3,
       std::array{canzero_get_levitation_board_front_state,
                  canzero_get_levitation_board_middle_state,
                  canzero_get_levitation_board_back_state}>
    levitation_system;

System<guidance_state, 2,
       std::array{canzero_get_guidance_board_back_state,
                  canzero_get_guidance_board_front_state}>
    guidance_system;

System<motor_state, 1, std::array{canzero_get_motor_driver_state}> motor_system;

System<input_board_state, 1, std::array{canzero_get_input_board_state}>
    sensor_system;

System<pdu_state, 2,
       std::array{canzero_get_power_board12_state,
                  canzero_get_power_board24_state}>
    power_system;

System<sdc_status, 9,
       std::array{canzero_get_motor_driver_sdc_status,
                  canzero_get_levitation_board_back_sdc_status,
                  canzero_get_levitation_board_middle_sdc_status,
                  canzero_get_levitation_board_front_sdc_status,
                  canzero_get_guidance_board_back_sdc_status,
                  canzero_get_guidance_board_front_sdc_status,
                  canzero_get_input_board_sdc_status,
                  canzero_get_power_board12_sdc_status,
                  canzero_get_power_board24_sdc_status}>
    sdc_system;
