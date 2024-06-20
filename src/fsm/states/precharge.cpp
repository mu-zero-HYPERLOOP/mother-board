#include "canzero.h"
#include "control/velocity.h"
#include "fsm/invariants.h"
#include "fsm/states.h"
#include "sdc.h"
#include "subsystems.h"
#include <array>

constexpr std::array<guidance_state, 3> ALLOWED_GUIDANCE_STATES = {
    guidance_state_ARMING45, guidance_state_PRECHARGE, guidance_state_READY};

constexpr std::array<levitation_state, 3> ALLOWED_LEVITATION_STATES = {
    levitation_state_ARMING45, levitation_state_PRECHARGE,
    levitation_state_READY};

constexpr std::array<motor_state, 3> ALLOWED_MOTOR_STATES = {
    motor_state_ARMING45, motor_state_PRECHARGE, motor_state_READY};

constexpr Duration STATE_TIMEOUT = 5_s;

// Invariants:
// - guidance is [arming45, precharge, ready]
// - levitation is [arming45, precharge, ready]
// - motor is [arming45, precharge, ready]
// - input board is running
// - pdus are running
// - sdc is closed
// Exit condition:
// - All systems in ready.
global_state fsm::states::precharge(global_command cmd,
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

  //================= TRANSITIONS ================
  if (global_command_SHUTDOWN == cmd){
    return global_state_SHUTDOWN;
  }

  if (global_command_EMERGENCY == cmd){
    return global_state_DISARMING45;
  }

  // Invariant: guidance
  if ((!contains(ALLOWED_GUIDANCE_STATES, g1_state) ||
       !contains(ALLOWED_GUIDANCE_STATES, g2_state)) &&
      !DISABLE_GUIDANCE_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: levitation
  if ((!contains(ALLOWED_LEVITATION_STATES, l1_state) ||
       !contains(ALLOWED_LEVITATION_STATES, l2_state) ||
       !contains(ALLOWED_LEVITATION_STATES, l3_state)) &&
      !DISABLE_LEVITATION_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: motor
  if (!contains(ALLOWED_MOTOR_STATES, motor_state) &&
      !DISABLE_MOTOR_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: input board
  if (input_board_state_RUNNING != input_state && !DISABLE_INPUT_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: pdus
  if ((pdu_12v_state_CHANNELS_ON != pdu12_state || pdu_24v_state_CHANNELS_ON != pdu24_state) &&
      !DISABLE_POWER_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  // Invariant: sdc
  if (sdc::status() == sdc_status_OPEN) {
    return global_state_DISARMING45;
  }

  if (time_since_last_transition > STATE_TIMEOUT){
    return global_state_DISARMING45;
  }

  // Transition into disarming45 if.
  // - STOP_45 is send
  if (global_command_STOP_45 == cmd || global_command_ABORT == cmd) {
    return global_state_DISARMING45;
  }

  // Transition into ready iff.
  // - input board is running (already checked)
  // - pdus are running (already checked)
  // - levitation is ready
  // - guidance is ready
  // - motor is ready
  // - SDC is closed.
  if (((guidance_state_READY == g1_state && guidance_state_READY == g2_state) ||
       DISABLE_GUIDANCE_SUBSYSTEM) &&

      ((levitation_state_READY == l1_state &&
        levitation_state_READY == l2_state &&
        levitation_state_READY == l3_state) ||
       DISABLE_LEVITATION_SUBSYSTEM) &&

      (motor_state_READY == motor_state || DISABLE_MOTOR_SUBSYSTEM) &&

      sdc::status() == sdc_status_CLOSED) {
    return global_state_READY;
  }

  // ================== OUTPUT ==================
  canzero_set_guidance_command(guidance_command_PRECHARGE);
  canzero_set_levitation_command(levitation_command_PRECHARGE);
  canzero_set_motor_driver_command(motor_command_PRECHARGE);
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_power_board12_command(pdu_12v_command_NONE);
  canzero_set_power_board24_command(pdu_24v_command_NONE);
  canzero_set_input_board_assert_45V_online(bool_t_TRUE);
  control::velocity::disable();

  return global_state_PRECHARGE;
}
