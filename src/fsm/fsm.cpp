#include "fsm.h"
#include "canzero/canzero.h"
#include "fsm/error_handling.h"
#include "fsm/states.h"
#include "util/timestamp.h"
#include <iostream>

static Timestamp fsm_last_transition = Timestamp::now();

void fsm::begin() {
  fsm_last_transition = Timestamp::now();
  canzero_set_state(global_state_INIT);
  canzero_set_command(global_command_NONE);
  canzero_set_track_length(10);
  canzero_set_brake_margin(3);
  canzero_set_emergency_brake_margin(2);
  canzero_set_target_acceleration(0);
  canzero_set_pod_grounded(bool_t_TRUE);
  canzero_set_position(0);
  canzero_set_velocity(0);
  canzero_set_acceleration(0);
  canzero_set_motor_driver_state(motor_state_INIT);
  canzero_set_motor_driver_command(motor_command_NONE);
  canzero_set_motor_driver_sdc_status(sdc_status_OPEN);
  canzero_set_guidance_command(guidance_command_NONE);
  canzero_set_guidance_board_front_state(guidance_state_INIT);
  canzero_set_guidance_board_front_sdc_status(sdc_status_OPEN);
  canzero_set_guidance_board_back_state(guidance_state_INIT);
  canzero_set_guidance_board_back_sdc_status(sdc_status_OPEN);
  canzero_set_levitation_command(levitation_command_NONE);
  canzero_set_levitation_board1_state(levitation_state_INIT);
  canzero_set_levitation_board1_sdc_status(sdc_status_OPEN);
  canzero_set_levitation_board2_state(levitation_state_INIT);
  canzero_set_levitation_board2_sdc_status(sdc_status_OPEN);
  canzero_set_levitation_board3_state(levitation_state_INIT);
  canzero_set_levitation_board3_sdc_status(sdc_status_OPEN);
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_input_board_state(input_board_state_INIT);
  canzero_set_input_board_sdc_status(sdc_status_OPEN);
  canzero_set_power_board12_state(pdu_12v_state_CHANNELS_TELEMETRY);
  canzero_set_power_board12_sdc_status(sdc_status_OPEN);
  canzero_set_power_board24_state(pdu_24v_state_INIT);
  canzero_set_power_board24_sdc_status(sdc_status_OPEN);

  canzero_set_acceleration_target_velocity(1);

  canzero_update_continue(canzero_get_time());
}

void fsm::update() {

  global_state state;
  global_state next_state;
  do {

    Timestamp now = Timestamp::now();
    Duration time_since_last_transition = now - fsm_last_transition;

    global_command cmd = error_handling::approve(canzero_get_command());

    state = canzero_get_state();
    switch (state) {
    case global_state_INIT:
      next_state = fsm::states::init(cmd, time_since_last_transition);
      break;
    case global_state_IDLE:
      next_state = fsm::states::idle(cmd, time_since_last_transition);
      break;
    case global_state_ARMING45:
      next_state = fsm::states::arming45(cmd, time_since_last_transition);
      break;
    case global_state_PRECHARGE:
      next_state = fsm::states::precharge(cmd, time_since_last_transition);
      break;
    case global_state_DISARMING45:
      next_state = fsm::states::disarming45(cmd, time_since_last_transition);
      break;
    case global_state_READY:
      next_state = fsm::states::ready(cmd, time_since_last_transition);
      break;
    case global_state_START_LEVITATION:
      next_state = fsm::states::start_levitation(cmd, time_since_last_transition);
      break;
    case global_state_LEVITATION_STABLE:
      next_state = fsm::states::levitation_stable(cmd, time_since_last_transition);
      break;
    case global_state_START_GUIDANCE:
      next_state = fsm::states::start_guidance(cmd, time_since_last_transition);
      break;
    case global_state_GUIDANCE_STABLE:
      next_state = fsm::states::guidance_stable(cmd, time_since_last_transition);
      break;
    case global_state_ACCELERATION:
      next_state = fsm::states::acceleration(cmd, time_since_last_transition);
      break;
    case global_state_CONTROLLER:
      next_state = fsm::states::controller(cmd, time_since_last_transition);
      break;
    case global_state_CRUISING:
      next_state = fsm::states::cruising(cmd, time_since_last_transition);
      break;
    case global_state_DECELERATION:
      next_state = fsm::states::deceleration(cmd, time_since_last_transition);
      break;
    case global_state_STOP_LEVITATION:
      next_state = fsm::states::stop_levitation(cmd, time_since_last_transition);
      break;
    case global_state_STOP_GUIDANCE:
      next_state = fsm::states::stop_guidance(cmd, time_since_last_transition);
      break;
    case global_state_SHUTDOWN:
      next_state = fsm::states::shutdown(cmd, time_since_last_transition);
      break;
    case global_state_RESTARTING:
      next_state = fsm::states::restarting(cmd, time_since_last_transition);
      break;
    case global_state_CALIBRATING:
      next_state = fsm::states::calibrating(cmd, time_since_last_transition);
      break;
    default:
      next_state = fsm::states::disarming45(cmd, time_since_last_transition);
      break;
    }

    if (next_state != state) {
      fsm_last_transition = now;
      canzero_set_state(next_state);
      canzero_update_continue(canzero_get_time());
      switch (next_state){
      case global_state_INIT:
        std::cout << "INIT" << std::endl;
        break;
      case global_state_IDLE:
        std::cout << "IDLE" << std::endl;
        break;
      case global_state_ARMING45:
        std::cout << "ARMING45" << std::endl;
        break;
      case global_state_PRECHARGE:
        std::cout << "PRECHARGE" << std::endl;
        break;
      case global_state_DISARMING45:
        std::cout << "DISARMING45" << std::endl;
        break;
      case global_state_READY:
        std::cout << "READY" << std::endl;
        break;
      case global_state_START_LEVITATION:
        std::cout << "START_LEVI" << std::endl;
        break;
      case global_state_LEVITATION_STABLE:
        std::cout << "LEVI_STABL" << std::endl;
        break;
      case global_state_START_GUIDANCE:
        std::cout << "START_GUI" << std::endl;
        break;
      case global_state_GUIDANCE_STABLE:
        std::cout << "GUI_STABL" << std::endl;
        break;
      case global_state_ACCELERATION:
        std::cout << "ACCEL" << std::endl;
        break;
      case global_state_CONTROLLER:
        std::cout << "CONTROLLER" << std::endl;
        break;
      case global_state_CRUISING:
        std::cout << "CRUISING" << std::endl;
        break;
      case global_state_DECELERATION:
        std::cout << "DECELERATION" << std::endl;
        break;
      case global_state_STOP_LEVITATION:
        std::cout << "STOP_LEVI" << std::endl;
        break;
      case global_state_STOP_GUIDANCE:
        std::cout << "STOP_GUI" << std::endl;
        break;
      case global_state_SHUTDOWN:
        std::cout << "SHUTDOWN" << std::endl;
        break;
      case global_state_RESTARTING:
        std::cout << "RESTARTING" << std::endl;
        break;
      case global_state_CALIBRATING:
        std::cout << "CALIBRATING" << std::endl;
        break;
      default:
        std::cout << "UNDEFINED" << std::endl;
        break;
      }
    }
  } while (next_state != state);
}
