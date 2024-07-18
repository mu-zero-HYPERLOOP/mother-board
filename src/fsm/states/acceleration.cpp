#include "canzero.h"
#include "control/velocity.h"
#include "error_handling.h"
#include "fsm/invariants.h"
#include "fsm/states.h"
#include "sdc.h"
#include "subsystems.h"
#include <array>

constexpr std::array<motor_state, 2> ALLOWED_MOTOR_STATES = {
    motor_state_READY, motor_state_CONTROL};

constexpr Duration MAX_MOTOR_TRANSITION_TIME = 1_s;
constexpr Duration MAX_STATE_TIME = 5_s;

// Invariants:
// - levitation stable
// - guidance stable
// - motor is in [ready, control]
// - input board running
// - pdus runnings
// - sdc is closed
// Exit condition:
// - Pod has reached the target velocity
// - All systems in control
global_state fsm::states::acceleration(global_command cmd,
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

  if (global_command_SHUTDOWN == cmd) {
    return global_state_SHUTDOWN;
  }

  if (global_command_EMERGENCY == cmd) {
    return global_state_DISARMING45;
  }

  // ================= TRANSITIONS ===============
  // Invariant: guidance
  if ((g1_state != guidance_state_CONTROL ||
       g2_state != guidance_state_CONTROL) &&
      !DISABLE_GUIDANCE_SUBSYSTEM) {
    return error_handling::invariant_broken();
  }

  // Invariant: levitation
  if ((l1_state != levitation_state_CONTROL ||
       l2_state != levitation_state_CONTROL ||
       l3_state != levitation_state_CONTROL) &&
      !DISABLE_LEVITATION_SUBSYSTEM) {
    return error_handling::invariant_broken();
  }

  // Invariant: motor state
  if (!contains(ALLOWED_MOTOR_STATES, motor_state) &&
      !DISABLE_MOTOR_SUBSYSTEM) {
    return error_handling::invariant_broken();
  }

  // Invariant: input board
  if ((input_state != input_board_state_RUNNING) && !DISABLE_INPUT_SUBSYSTEM) {
    return error_handling::invariant_broken();
  }

  // Invariant: pdus
  if ((pdu12_state != pdu_12v_state_CHANNELS_ON || pdu24_state != pdu_24v_state_CHANNELS_ON) &&
      !DISABLE_POWER_SUBSYSTEM) {
    return error_handling::invariant_broken();
  }

  // Invariant: sdc
  if (sdc::status() == sdc_status_OPEN) {
    return error_handling::invariant_broken();
  }

  if (time_since_last_transition > MAX_STATE_TIME) {
    canzero_set_command(global_command_NONE);
    return global_state_DISARMING45;
  }

  if (global_command_STOP_45 == cmd) {
    return global_state_DISARMING45;
  }

  if (global_command_STOP_LEVITATION == cmd) {
    return global_state_STOP_LEVITATION; // invalid command!!!
  }

  // Ensures that after MIN_MOTOR_TRANSITION_TIME
  // the motor has transitioned into the control state.
  if (time_since_last_transition > MAX_MOTOR_TRANSITION_TIME &&
      motor_state_CONTROL != motor_state && !DISABLE_MOTOR_SUBSYSTEM) {
    return global_state_DISARMING45;
  }

  if (global_command_STOP_PROPULSION == cmd || global_command_ABORT == cmd) {
    return global_state_STOP_LEVITATION;
  }

  if (canzero_get_position() > 7.0f) {
    return global_state_STOP_LEVITATION;
  }
  if (canzero_get_position() > 8.0f) {
    return global_state_DISARMING45;
  }

  if (canzero_get_velocity() >= canzero_get_acceleration_target_velocity()) {
    return global_state_CRUISING;
  }

  // ============== OUTPUTS ==============
  canzero_set_input_board_command(input_board_command_NONE);
  canzero_set_input_board_assert_45V_online(bool_t_TRUE);
  canzero_set_guidance_command(guidance_command_START);
  canzero_set_levitation_command(levitation_command_START);
  canzero_set_motor_driver_command(motor_command_START);
  canzero_set_pod_grounded(bool_t_FALSE);
  canzero_set_power_board12_command(pdu_12v_command_NONE);
  canzero_set_power_board24_command(pdu_24v_command_NONE);
  control::velocity::enable();

  control::velocity::target_velocity(
      Velocity(canzero_get_acceleration_target_velocity()));


  return global_state_ACCELERATION;
}
