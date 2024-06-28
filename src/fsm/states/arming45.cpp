#include "canzero.h"
#include "control/velocity.h"
#include "fsm/invariants.h"
#include "fsm/states.h"
#include "sdc.h"
#include "subsystems.h"
#include <any>
#include <array>
#include <iostream>

constexpr std::array<guidance_state, 2> ALLOWED_GUIDANCE_STATES = {
    guidance_state_IDLE, guidance_state_ARMING45};

constexpr std::array<levitation_state, 2> ALLOWED_LEVITATION_STATES = {
    levitation_state_IDLE, levitation_state_ARMING45};

constexpr std::array<motor_state, 2> ALLOWED_MOTOR_STATES = {
    motor_state_IDLE, motor_state_ARMING45};

constexpr std::array<pdu_24v_state, 2> ALLOWED_PDU_24V_STATES = {
    pdu_24v_state_CHANNELS_ON, pdu_24v_state_CHANNELS_IDLE};

constexpr Duration STATE_TIMEOUT = 5_s;

// Invariant:
// - guidance is [idle, arming45]
// - levitation is [idle, arming45]
// - motor is [idle, arming45]
// - input board is running
// - all PDU 12V channels are on
// - PDU 24V is in [idle, channels_on]
// Exit condition:
// - The SDC is closed.
// - PDU 24V is in state channels_on
global_state fsm::states::arming45(global_command cmd,
                                   Duration time_since_last_transition) {

  using namespace fsm::invariant;

  const input_board_state input_state = canzero_get_input_board_state();
  const pdu_24v_state pdu24_state = canzero_get_power_board24_state();
  const pdu_12v_state pdu12_state = canzero_get_power_board12_state();

  const guidance_state g1_state = canzero_get_guidance_board_front_state();
  const guidance_state g2_state = canzero_get_guidance_board_back_state();

  const levitation_state l1_state = canzero_get_levitation_board1_state();
  const levitation_state l2_state = canzero_get_levitation_board2_state();
  const levitation_state l3_state = canzero_get_levitation_board3_state();

  const motor_state motor_state = canzero_get_motor_driver_state();


  // Invariant: guidance state
  if ((!contains(ALLOWED_GUIDANCE_STATES, g1_state) ||
       !contains(ALLOWED_GUIDANCE_STATES, g2_state)) &&
      !DISABLE_GUIDANCE_SUBSYSTEM) {
    std::cout << "GUIDANCE INVARIANT BROKEN" << std::endl;
    return global_state_DISARMING45;
  }

  // Invariant: levitation state
  if ((!contains(ALLOWED_LEVITATION_STATES, l1_state) ||
       !contains(ALLOWED_LEVITATION_STATES, l2_state) ||
       !contains(ALLOWED_LEVITATION_STATES, l3_state)) &&
      !DISABLE_LEVITATION_SUBSYSTEM) {
    std::cout << "LEVI INVARIANT BROKEN" << std::endl;
    return global_state_DISARMING45;
  }

  // Invariant: motor state
  if (!contains(ALLOWED_MOTOR_STATES, motor_state) &&
      !DISABLE_MOTOR_SUBSYSTEM) {
    std::cout << "MOTOR INVARIANT BROKEN" << std::endl;
    return global_state_DISARMING45;
  }

  // Invariant: input board
  if (input_board_state_RUNNING != input_state && !DISABLE_INPUT_SUBSYSTEM) {
    std::cout << "INPUT_BOARD INVARIANT BROKEN" << std::endl;
    return global_state_DISARMING45;
  }

  // Invariant: PDUs
  if ((pdu_12v_state_CHANNELS_ON != pdu12_state || !contains(ALLOWED_PDU_24V_STATES, pdu24_state)) 
      && !DISABLE_POWER_SUBSYSTEM) {
    std::cout << "PDU INVARIANT BROKEN" << std::endl;
    return global_state_DISARMING45;
  }

  // ============== TRANSITIONS ==============
  if (global_command_SHUTDOWN == cmd){
    return global_state_SHUTDOWN;
  }

  if (global_command_EMERGENCY == cmd){
    return global_state_DISARMING45;
  }

  if (global_command_STOP_45 == cmd || global_command_ABORT == cmd) {
    std::cout << "ARMING_ABORTED" << std::endl;
    return global_state_DISARMING45;
  }

  if (time_since_last_transition > STATE_TIMEOUT){
    std::cout << "ARMING TIMEOUT" << std::endl;
    return global_state_DISARMING45;
  }

  std::cout << sdc::status() << std::endl;

  // Transition into precharge iff.
  // - input board is running
  // - all pdu channels are on
  // - guidance is in arming45
  // - levitation is in arming45
  // - motor is in arming45
  // - SDC IS CLOSED!!!
  if ((input_board_state_RUNNING == input_state || DISABLE_INPUT_SUBSYSTEM) &&

      ((pdu_12v_state_CHANNELS_ON == pdu12_state && pdu_24v_state_CHANNELS_ON == pdu24_state) ||
       DISABLE_POWER_SUBSYSTEM) &&

      ((guidance_state_ARMING45 == g1_state &&
        guidance_state_ARMING45 == g2_state) ||
       DISABLE_GUIDANCE_SUBSYSTEM) &&

      ((levitation_state_ARMING45 == l1_state &&
        levitation_state_ARMING45 == l2_state &&
        levitation_state_ARMING45 == l3_state) ||
       DISABLE_LEVITATION_SUBSYSTEM) &&

      (motor_state_ARMING45 == motor_state || DISABLE_MOTOR_SUBSYSTEM) &&

      sdc::status() == sdc_status_CLOSED

      && time_since_last_transition > 500_ms) {
    return global_state_PRECHARGE;
  }

  // =========== OUTPUT ===========
  canzero_set_guidance_command(guidance_command_ARM45);
  canzero_set_levitation_command(levitation_command_ARM45);
  canzero_set_motor_driver_command(motor_command_ARM45);
  canzero_set_pod_grounded(bool_t_TRUE);
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_power_board12_command(pdu_12v_command_NONE);
  canzero_set_power_board24_command(pdu_24v_command_START);
  canzero_set_input_board_assert_45V_online(bool_t_FALSE);
  control::velocity::disable();

  return global_state_ARMING45;
}
