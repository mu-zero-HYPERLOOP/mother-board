#ifndef CANZERO_H
#define CANZERO_H
#include <cinttypes>
#include <cstddef>
#define MAX_DYN_HEARTBEATS 10
typedef enum {
  node_id_gamepad = 0,
  node_id_mother_board = 1,
  node_id_motor_driver = 2,
  node_id_guidance_board_front = 3,
  node_id_guidance_board_back = 4,
  node_id_levitation_board1 = 5,
  node_id_levitation_board2 = 6,
  node_id_levitation_board3 = 7,
  node_id_input_board = 8,
  node_id_power_board12 = 9,
  node_id_power_board24 = 10,
  node_id_led_board = 11,
  node_id_count = 12,
} node_id;
typedef struct {
  uint16_t m_od_index;
  uint8_t m_client_id;
  uint8_t m_server_id;
} get_req_header;
typedef struct {
  uint8_t m_sof;
  uint8_t m_eof;
  uint8_t m_toggle;
  uint16_t m_od_index;
  uint8_t m_client_id;
  uint8_t m_server_id;
} set_req_header;
typedef enum {
  motor_state_INIT = 0,
  motor_state_IDLE = 1,
  motor_state_ARMING45 = 2,
  motor_state_PRECHARGE = 3,
  motor_state_READY = 4,
  motor_state_CONTROL = 5,
  motor_state_DISARMING45 = 6,
} motor_state;
typedef enum {
  sdc_status_OPEN = 0,
  sdc_status_CLOSED = 1,
} sdc_status;
typedef enum {
  motor_command_NONE = 0,
  motor_command_ARM45 = 1,
  motor_command_PRECHARGE = 2,
  motor_command_START = 3,
  motor_command_STOP = 4,
  motor_command_ABORT = 5,
  motor_command_DISARM45 = 6,
} motor_command;
typedef enum {
  bool_t_FALSE = 0,
  bool_t_TRUE = 1,
} bool_t;
typedef enum {
  error_flag_OK = 0,
  error_flag_ERROR = 1,
} error_flag;
typedef enum {
  error_level_OK = 0,
  error_level_INFO = 1,
  error_level_WARNING = 2,
  error_level_ERROR = 3,
} error_level;
typedef enum {
  guidance_state_INIT = 0,
  guidance_state_IDLE = 1,
  guidance_state_ARMING45 = 2,
  guidance_state_PRECHARGE = 3,
  guidance_state_READY = 4,
  guidance_state_CONTROL = 5,
  guidance_state_DISARMING45 = 6,
} guidance_state;
typedef enum {
  guidance_command_NONE = 0,
  guidance_command_ARM45 = 1,
  guidance_command_PRECHARGE = 2,
  guidance_command_START = 3,
  guidance_command_STOP = 4,
  guidance_command_DISARM45 = 5,
} guidance_command;
typedef enum {
  levitation_state_INIT = 0,
  levitation_state_IDLE = 1,
  levitation_state_ARMING45 = 2,
  levitation_state_PRECHARGE = 3,
  levitation_state_READY = 4,
  levitation_state_START = 5,
  levitation_state_CONTROL = 6,
  levitation_state_STOP = 7,
  levitation_state_DISARMING45 = 8,
} levitation_state;
typedef enum {
  levitation_command_NONE = 0,
  levitation_command_ARM45 = 1,
  levitation_command_PRECHARGE = 2,
  levitation_command_START = 3,
  levitation_command_STOP = 4,
  levitation_command_ABORT = 5,
  levitation_command_DISARM45 = 6,
} levitation_command;
typedef enum {
  input_board_state_INIT = 0,
  input_board_state_CALIBRATION = 1,
  input_board_state_RUNNING = 2,
} input_board_state;
typedef enum {
  pdu_12v_state_INIT = 0,
  pdu_12v_state_CHANNELS_OFF = 1,
  pdu_12v_state_CHANNELS_TELEMETRY = 2,
  pdu_12v_state_CHANNELS_ON = 3,
  pdu_12v_state_RESTARTING = 4,
} pdu_12v_state;
typedef enum {
  pdu_24v_state_INIT = 0,
  pdu_24v_state_CHANNELS_OFF = 1,
  pdu_24v_state_CHANNELS_ON = 2,
} pdu_24v_state;
typedef enum {
  led_board_state_INIT = 0,
  led_board_state_STARTUP = 1,
  led_board_state_LIFTOFF = 2,
  led_board_state_BREATHE = 3,
  led_board_state_RAINBOW = 4,
  led_board_state_SHUTDOWN = 5,
} led_board_state;
typedef struct {
  uint8_t m_sof;
  uint8_t m_eof;
  uint8_t m_toggle;
  uint16_t m_od_index;
  uint8_t m_client_id;
  uint8_t m_server_id;
} get_resp_header;
typedef enum {
  set_resp_erno_Success = 0,
  set_resp_erno_Error = 1,
} set_resp_erno;
typedef struct {
  uint16_t m_od_index;
  uint8_t m_client_id;
  uint8_t m_server_id;
  set_resp_erno m_erno;
} set_resp_header;
typedef enum {
  global_state_INIT = 0,
  global_state_IDLE = 1,
  global_state_ARMING45 = 2,
  global_state_PRECHARGE = 3,
  global_state_DISARMING45 = 4,
  global_state_READY = 5,
  global_state_START_LEVITATION = 6,
  global_state_LEVITATION_STABLE = 7,
  global_state_START_GUIDANCE = 8,
  global_state_GUIDANCE_STABLE = 9,
  global_state_ACCELERATION = 10,
  global_state_CONTROLLER = 11,
  global_state_CRUISING = 12,
  global_state_DECELERATION = 13,
  global_state_STOP_LEVITATION = 14,
  global_state_STOP_GUIDANCE = 15,
  global_state_SHUTDOWN = 16,
  global_state_RESTARTING = 17,
  global_state_CALIBRATING = 18,
} global_state;
typedef enum {
  global_command_NONE = 0,
  global_command_START_45 = 1,
  global_command_STOP_45 = 2,
  global_command_START_LEVITATION = 3,
  global_command_STOP_LEVITATION = 4,
  global_command_START_PROPULSION = 5,
  global_command_STOP_PROPULSION = 6,
  global_command_START_CONTROLLER = 7,
  global_command_STOP_CONTROLLER = 8,
  global_command_ABORT = 9,
  global_command_EMERGENCY = 10,
  global_command_SHUTDOWN = 11,
  global_command_RESTART = 12,
  global_command_CALIBRATE = 13,
} global_command;
typedef enum {
  input_board_command_NONE = 0,
  input_board_command_CALIBRATE = 1,
} input_board_command;
typedef enum {
  airgap_transition_mode_LINEAR = 0,
  airgap_transition_mode_SIGMOID = 1,
  airgap_transition_mode_EASE_OUT_SINE = 2,
  airgap_transition_mode_EASE_INOUT_SINE = 3,
} airgap_transition_mode;
typedef enum {
  pdu_12v_command_NONE = 0,
  pdu_12v_command_START = 1,
  pdu_12v_command_STOP = 2,
  pdu_12v_command_TELEMETRY = 3,
  pdu_12v_command_RESTART = 4,
} pdu_12v_command;
typedef enum {
  pdu_24v_command_NONE = 0,
  pdu_24v_command_START = 1,
  pdu_24v_command_STOP = 2,
} pdu_24v_command;
typedef enum {
  led_board_command_NONE = 0,
  led_board_command_DO_SHIT = 1,
} led_board_command;
typedef struct {
  uint16_t m_year;
  uint8_t m_month;
  uint8_t m_day;
  uint8_t m_hour;
  uint8_t m_min;
  uint8_t m_sec;
} date_time;
typedef struct {
  double m_Kp;
  double m_Ki;
  double m_Kd;
} pid_parameters;
static const node_id CANZERO_NODE_ID = node_id_mother_board;
typedef struct {
  uint32_t id;
  uint8_t dlc;
  __attribute__((aligned(alignof(uint64_t)))) uint8_t data[8];
} canzero_frame;
typedef enum : uint32_t {
  CANZERO_FRAME_IDE_BIT = 0x40000000, // 1 << 30
  CANZERO_FRAME_RTR_BIT = 0x80000000, // 1 << 31
} can_frame_id_bits;
typedef struct {
  uint32_t mask;
  uint32_t id;
} canzero_can_filter;
extern void canzero_can0_setup(uint32_t baudrate, canzero_can_filter* filters, int filter_count);
extern void canzero_can0_send(canzero_frame* frame);
extern int canzero_can0_recv(canzero_frame* frame);
extern void canzero_can1_setup(uint32_t baudrate, canzero_can_filter* filters, int filter_count);
extern void canzero_can1_send(canzero_frame* frame);
extern int canzero_can1_recv(canzero_frame* frame);
extern void canzero_request_update(uint32_t time);
extern uint32_t canzero_get_time();
extern void canzero_enter_critical();
extern void canzero_exit_critical();
static inline uint64_t canzero_get_config_hash() {
  extern uint64_t __oe_config_hash;
  return __oe_config_hash;
}
static inline date_time canzero_get_build_time() {
  extern date_time __oe_build_time;
  return __oe_build_time;
}
static inline global_state canzero_get_state() {
  extern global_state __oe_state;
  return __oe_state;
}
static inline global_command canzero_get_command() {
  extern global_command __oe_command;
  return __oe_command;
}
static inline sdc_status canzero_get_system_sdc_status() {
  extern sdc_status __oe_system_sdc_status;
  return __oe_system_sdc_status;
}
static inline float canzero_get_track_length() {
  extern float __oe_track_length;
  return __oe_track_length;
}
static inline float canzero_get_brake_margin() {
  extern float __oe_brake_margin;
  return __oe_brake_margin;
}
static inline float canzero_get_emergency_brake_margin() {
  extern float __oe_emergency_brake_margin;
  return __oe_emergency_brake_margin;
}
static inline float canzero_get_target_acceleration() {
  extern float __oe_target_acceleration;
  return __oe_target_acceleration;
}
static inline bool_t canzero_get_pod_grounded() {
  extern bool_t __oe_pod_grounded;
  return __oe_pod_grounded;
}
static inline float canzero_get_acceleration_target_velocity() {
  extern float __oe_acceleration_target_velocity;
  return __oe_acceleration_target_velocity;
}
static inline pid_parameters canzero_get_velocity_pid() {
  extern pid_parameters __oe_velocity_pid;
  return __oe_velocity_pid;
}
static inline float canzero_get_position() {
  extern float __oe_position;
  return __oe_position;
}
static inline float canzero_get_velocity() {
  extern float __oe_velocity;
  return __oe_velocity;
}
static inline float canzero_get_acceleration() {
  extern float __oe_acceleration;
  return __oe_acceleration;
}
static inline error_level canzero_get_error_level_config_consistency() {
  extern error_level __oe_error_level_config_consistency;
  return __oe_error_level_config_consistency;
}
static inline error_flag canzero_get_error_any() {
  extern error_flag __oe_error_any;
  return __oe_error_any;
}
static inline error_level canzero_get_error_level_over_temperature_system() {
  extern error_level __oe_error_level_over_temperature_system;
  return __oe_error_level_over_temperature_system;
}
static inline motor_state canzero_get_motor_driver_state() {
  extern motor_state __oe_motor_driver_state;
  return __oe_motor_driver_state;
}
static inline motor_command canzero_get_motor_driver_command() {
  extern motor_command __oe_motor_driver_command;
  return __oe_motor_driver_command;
}
static inline sdc_status canzero_get_motor_driver_sdc_status() {
  extern sdc_status __oe_motor_driver_sdc_status;
  return __oe_motor_driver_sdc_status;
}
static inline uint64_t canzero_get_motor_driver_config_hash() {
  extern uint64_t __oe_motor_driver_config_hash;
  return __oe_motor_driver_config_hash;
}
static inline error_flag canzero_get_motor_driver_assertion_fault() {
  extern error_flag __oe_motor_driver_assertion_fault;
  return __oe_motor_driver_assertion_fault;
}
static inline error_flag canzero_get_motor_driver_error_arming_failed() {
  extern error_flag __oe_motor_driver_error_arming_failed;
  return __oe_motor_driver_error_arming_failed;
}
static inline error_flag canzero_get_motor_driver_error_precharge_failed() {
  extern error_flag __oe_motor_driver_error_precharge_failed;
  return __oe_motor_driver_error_precharge_failed;
}
static inline error_flag canzero_get_motor_driver_error_acceleration_out_of_range() {
  extern error_flag __oe_motor_driver_error_acceleration_out_of_range;
  return __oe_motor_driver_error_acceleration_out_of_range;
}
static inline error_flag canzero_get_motor_driver_error_acceleration_calibration_failed() {
  extern error_flag __oe_motor_driver_error_acceleration_calibration_failed;
  return __oe_motor_driver_error_acceleration_calibration_failed;
}
static inline error_flag canzero_get_motor_driver_error_invalid_target_acceleration() {
  extern error_flag __oe_motor_driver_error_invalid_target_acceleration;
  return __oe_motor_driver_error_invalid_target_acceleration;
}
static inline error_flag canzero_get_motor_driver_error_board_temperature1_invalid() {
  extern error_flag __oe_motor_driver_error_board_temperature1_invalid;
  return __oe_motor_driver_error_board_temperature1_invalid;
}
static inline error_flag canzero_get_motor_driver_error_board_temperature2_invalid() {
  extern error_flag __oe_motor_driver_error_board_temperature2_invalid;
  return __oe_motor_driver_error_board_temperature2_invalid;
}
static inline error_flag canzero_get_motor_driver_error_board_temperature3_invalid() {
  extern error_flag __oe_motor_driver_error_board_temperature3_invalid;
  return __oe_motor_driver_error_board_temperature3_invalid;
}
static inline error_flag canzero_get_motor_driver_error_mcu_temperature_invalid() {
  extern error_flag __oe_motor_driver_error_mcu_temperature_invalid;
  return __oe_motor_driver_error_mcu_temperature_invalid;
}
static inline error_flag canzero_get_motor_driver_error_lim_temperature1_invalid() {
  extern error_flag __oe_motor_driver_error_lim_temperature1_invalid;
  return __oe_motor_driver_error_lim_temperature1_invalid;
}
static inline error_flag canzero_get_motor_driver_error_lim_temperature2_invalid() {
  extern error_flag __oe_motor_driver_error_lim_temperature2_invalid;
  return __oe_motor_driver_error_lim_temperature2_invalid;
}
static inline error_flag canzero_get_motor_driver_error_lim_temperature3_invalid() {
  extern error_flag __oe_motor_driver_error_lim_temperature3_invalid;
  return __oe_motor_driver_error_lim_temperature3_invalid;
}
static inline error_flag canzero_get_motor_driver_error_lim_temperature4_invalid() {
  extern error_flag __oe_motor_driver_error_lim_temperature4_invalid;
  return __oe_motor_driver_error_lim_temperature4_invalid;
}
static inline error_flag canzero_get_motor_driver_error_heartbeat_miss() {
  extern error_flag __oe_motor_driver_error_heartbeat_miss;
  return __oe_motor_driver_error_heartbeat_miss;
}
static inline error_level canzero_get_motor_driver_error_level_current_u1() {
  extern error_level __oe_motor_driver_error_level_current_u1;
  return __oe_motor_driver_error_level_current_u1;
}
static inline error_level canzero_get_motor_driver_error_level_current_v1() {
  extern error_level __oe_motor_driver_error_level_current_v1;
  return __oe_motor_driver_error_level_current_v1;
}
static inline error_level canzero_get_motor_driver_error_level_current_w1() {
  extern error_level __oe_motor_driver_error_level_current_w1;
  return __oe_motor_driver_error_level_current_w1;
}
static inline error_level canzero_get_motor_driver_error_level_current_u2() {
  extern error_level __oe_motor_driver_error_level_current_u2;
  return __oe_motor_driver_error_level_current_u2;
}
static inline error_level canzero_get_motor_driver_error_level_current_v2() {
  extern error_level __oe_motor_driver_error_level_current_v2;
  return __oe_motor_driver_error_level_current_v2;
}
static inline error_level canzero_get_motor_driver_error_level_current_w2() {
  extern error_level __oe_motor_driver_error_level_current_w2;
  return __oe_motor_driver_error_level_current_w2;
}
static inline error_level canzero_get_motor_driver_error_level_vdc_voltage() {
  extern error_level __oe_motor_driver_error_level_vdc_voltage;
  return __oe_motor_driver_error_level_vdc_voltage;
}
static inline error_level canzero_get_motor_driver_error_level_board_temperature() {
  extern error_level __oe_motor_driver_error_level_board_temperature;
  return __oe_motor_driver_error_level_board_temperature;
}
static inline error_level canzero_get_motor_driver_error_level_mcu_temperature() {
  extern error_level __oe_motor_driver_error_level_mcu_temperature;
  return __oe_motor_driver_error_level_mcu_temperature;
}
static inline error_level canzero_get_motor_driver_error_level_lim_temperature() {
  extern error_level __oe_motor_driver_error_level_lim_temperature;
  return __oe_motor_driver_error_level_lim_temperature;
}
static inline guidance_command canzero_get_guidance_command() {
  extern guidance_command __oe_guidance_command;
  return __oe_guidance_command;
}
static inline guidance_state canzero_get_guidance_board_front_state() {
  extern guidance_state __oe_guidance_board_front_state;
  return __oe_guidance_board_front_state;
}
static inline sdc_status canzero_get_guidance_board_front_sdc_status() {
  extern sdc_status __oe_guidance_board_front_sdc_status;
  return __oe_guidance_board_front_sdc_status;
}
static inline uint64_t canzero_get_guidance_board_front_config_hash() {
  extern uint64_t __oe_guidance_board_front_config_hash;
  return __oe_guidance_board_front_config_hash;
}
static inline error_flag canzero_get_guidance_board_front_assertion_fault() {
  extern error_flag __oe_guidance_board_front_assertion_fault;
  return __oe_guidance_board_front_assertion_fault;
}
static inline error_flag canzero_get_guidance_board_front_error_arming_failed() {
  extern error_flag __oe_guidance_board_front_error_arming_failed;
  return __oe_guidance_board_front_error_arming_failed;
}
static inline error_flag canzero_get_guidance_board_front_error_precharge_failed() {
  extern error_flag __oe_guidance_board_front_error_precharge_failed;
  return __oe_guidance_board_front_error_precharge_failed;
}
static inline error_flag canzero_get_guidance_board_front_error_heartbeat_miss() {
  extern error_flag __oe_guidance_board_front_error_heartbeat_miss;
  return __oe_guidance_board_front_error_heartbeat_miss;
}
static inline error_flag canzero_get_guidance_board_front_error_outer_airgap_left_invalid() {
  extern error_flag __oe_guidance_board_front_error_outer_airgap_left_invalid;
  return __oe_guidance_board_front_error_outer_airgap_left_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_inner_airgap_left_invalid() {
  extern error_flag __oe_guidance_board_front_error_inner_airgap_left_invalid;
  return __oe_guidance_board_front_error_inner_airgap_left_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_outer_airgap_right_invalid() {
  extern error_flag __oe_guidance_board_front_error_outer_airgap_right_invalid;
  return __oe_guidance_board_front_error_outer_airgap_right_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_inner_airgap_right_invalid() {
  extern error_flag __oe_guidance_board_front_error_inner_airgap_right_invalid;
  return __oe_guidance_board_front_error_inner_airgap_right_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_vdc_voltage_invalid() {
  extern error_flag __oe_guidance_board_front_error_vdc_voltage_invalid;
  return __oe_guidance_board_front_error_vdc_voltage_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_magnet_current_left_invalid() {
  extern error_flag __oe_guidance_board_front_error_magnet_current_left_invalid;
  return __oe_guidance_board_front_error_magnet_current_left_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_magnet_current_right_invalid() {
  extern error_flag __oe_guidance_board_front_error_magnet_current_right_invalid;
  return __oe_guidance_board_front_error_magnet_current_right_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_input_current_invalid() {
  extern error_flag __oe_guidance_board_front_error_input_current_invalid;
  return __oe_guidance_board_front_error_input_current_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_magnet_temperature_left_invalid() {
  extern error_flag __oe_guidance_board_front_error_magnet_temperature_left_invalid;
  return __oe_guidance_board_front_error_magnet_temperature_left_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_magnet_temperature_right_invalid() {
  extern error_flag __oe_guidance_board_front_error_magnet_temperature_right_invalid;
  return __oe_guidance_board_front_error_magnet_temperature_right_invalid;
}
static inline error_flag canzero_get_guidance_board_front_error_mcu_temperature_invalid() {
  extern error_flag __oe_guidance_board_front_error_mcu_temperature_invalid;
  return __oe_guidance_board_front_error_mcu_temperature_invalid;
}
static inline error_level canzero_get_guidance_board_front_error_level_vdc_voltage() {
  extern error_level __oe_guidance_board_front_error_level_vdc_voltage;
  return __oe_guidance_board_front_error_level_vdc_voltage;
}
static inline error_level canzero_get_guidance_board_front_error_level_magnet_current_left() {
  extern error_level __oe_guidance_board_front_error_level_magnet_current_left;
  return __oe_guidance_board_front_error_level_magnet_current_left;
}
static inline error_level canzero_get_guidance_board_front_error_level_magnet_current_right() {
  extern error_level __oe_guidance_board_front_error_level_magnet_current_right;
  return __oe_guidance_board_front_error_level_magnet_current_right;
}
static inline error_level canzero_get_guidance_board_front_error_level_input_current() {
  extern error_level __oe_guidance_board_front_error_level_input_current;
  return __oe_guidance_board_front_error_level_input_current;
}
static inline error_level canzero_get_guidance_board_front_error_level_magnet_temperature_left() {
  extern error_level __oe_guidance_board_front_error_level_magnet_temperature_left;
  return __oe_guidance_board_front_error_level_magnet_temperature_left;
}
static inline error_level canzero_get_guidance_board_front_error_level_magnet_temperature_right() {
  extern error_level __oe_guidance_board_front_error_level_magnet_temperature_right;
  return __oe_guidance_board_front_error_level_magnet_temperature_right;
}
static inline error_level canzero_get_guidance_board_front_error_level_mcu_temperature() {
  extern error_level __oe_guidance_board_front_error_level_mcu_temperature;
  return __oe_guidance_board_front_error_level_mcu_temperature;
}
static inline guidance_state canzero_get_guidance_board_back_state() {
  extern guidance_state __oe_guidance_board_back_state;
  return __oe_guidance_board_back_state;
}
static inline sdc_status canzero_get_guidance_board_back_sdc_status() {
  extern sdc_status __oe_guidance_board_back_sdc_status;
  return __oe_guidance_board_back_sdc_status;
}
static inline uint64_t canzero_get_guidance_board_back_config_hash() {
  extern uint64_t __oe_guidance_board_back_config_hash;
  return __oe_guidance_board_back_config_hash;
}
static inline error_flag canzero_get_guidance_board_back_assertion_fault() {
  extern error_flag __oe_guidance_board_back_assertion_fault;
  return __oe_guidance_board_back_assertion_fault;
}
static inline error_flag canzero_get_guidance_board_back_error_arming_failed() {
  extern error_flag __oe_guidance_board_back_error_arming_failed;
  return __oe_guidance_board_back_error_arming_failed;
}
static inline error_flag canzero_get_guidance_board_back_error_precharge_failed() {
  extern error_flag __oe_guidance_board_back_error_precharge_failed;
  return __oe_guidance_board_back_error_precharge_failed;
}
static inline error_flag canzero_get_guidance_board_back_error_heartbeat_miss() {
  extern error_flag __oe_guidance_board_back_error_heartbeat_miss;
  return __oe_guidance_board_back_error_heartbeat_miss;
}
static inline error_flag canzero_get_guidance_board_back_error_outer_airgap_left_invalid() {
  extern error_flag __oe_guidance_board_back_error_outer_airgap_left_invalid;
  return __oe_guidance_board_back_error_outer_airgap_left_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_inner_airgap_left_invalid() {
  extern error_flag __oe_guidance_board_back_error_inner_airgap_left_invalid;
  return __oe_guidance_board_back_error_inner_airgap_left_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_outer_airgap_right_invalid() {
  extern error_flag __oe_guidance_board_back_error_outer_airgap_right_invalid;
  return __oe_guidance_board_back_error_outer_airgap_right_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_inner_airgap_right_invalid() {
  extern error_flag __oe_guidance_board_back_error_inner_airgap_right_invalid;
  return __oe_guidance_board_back_error_inner_airgap_right_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_vdc_voltage_invalid() {
  extern error_flag __oe_guidance_board_back_error_vdc_voltage_invalid;
  return __oe_guidance_board_back_error_vdc_voltage_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_magnet_current_left_invalid() {
  extern error_flag __oe_guidance_board_back_error_magnet_current_left_invalid;
  return __oe_guidance_board_back_error_magnet_current_left_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_magnet_current_right_invalid() {
  extern error_flag __oe_guidance_board_back_error_magnet_current_right_invalid;
  return __oe_guidance_board_back_error_magnet_current_right_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_input_current_invalid() {
  extern error_flag __oe_guidance_board_back_error_input_current_invalid;
  return __oe_guidance_board_back_error_input_current_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_magnet_temperature_left_invalid() {
  extern error_flag __oe_guidance_board_back_error_magnet_temperature_left_invalid;
  return __oe_guidance_board_back_error_magnet_temperature_left_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_magnet_temperature_right_invalid() {
  extern error_flag __oe_guidance_board_back_error_magnet_temperature_right_invalid;
  return __oe_guidance_board_back_error_magnet_temperature_right_invalid;
}
static inline error_flag canzero_get_guidance_board_back_error_mcu_temperature_invalid() {
  extern error_flag __oe_guidance_board_back_error_mcu_temperature_invalid;
  return __oe_guidance_board_back_error_mcu_temperature_invalid;
}
static inline error_level canzero_get_guidance_board_back_error_level_vdc_voltage() {
  extern error_level __oe_guidance_board_back_error_level_vdc_voltage;
  return __oe_guidance_board_back_error_level_vdc_voltage;
}
static inline error_level canzero_get_guidance_board_back_error_level_magnet_current_left() {
  extern error_level __oe_guidance_board_back_error_level_magnet_current_left;
  return __oe_guidance_board_back_error_level_magnet_current_left;
}
static inline error_level canzero_get_guidance_board_back_error_level_magnet_current_right() {
  extern error_level __oe_guidance_board_back_error_level_magnet_current_right;
  return __oe_guidance_board_back_error_level_magnet_current_right;
}
static inline error_level canzero_get_guidance_board_back_error_level_input_current() {
  extern error_level __oe_guidance_board_back_error_level_input_current;
  return __oe_guidance_board_back_error_level_input_current;
}
static inline error_level canzero_get_guidance_board_back_error_level_magnet_temperature_left() {
  extern error_level __oe_guidance_board_back_error_level_magnet_temperature_left;
  return __oe_guidance_board_back_error_level_magnet_temperature_left;
}
static inline error_level canzero_get_guidance_board_back_error_level_magnet_temperature_right() {
  extern error_level __oe_guidance_board_back_error_level_magnet_temperature_right;
  return __oe_guidance_board_back_error_level_magnet_temperature_right;
}
static inline error_level canzero_get_guidance_board_back_error_level_mcu_temperature() {
  extern error_level __oe_guidance_board_back_error_level_mcu_temperature;
  return __oe_guidance_board_back_error_level_mcu_temperature;
}
static inline levitation_command canzero_get_levitation_command() {
  extern levitation_command __oe_levitation_command;
  return __oe_levitation_command;
}
static inline levitation_state canzero_get_levitation_board1_state() {
  extern levitation_state __oe_levitation_board1_state;
  return __oe_levitation_board1_state;
}
static inline sdc_status canzero_get_levitation_board1_sdc_status() {
  extern sdc_status __oe_levitation_board1_sdc_status;
  return __oe_levitation_board1_sdc_status;
}
static inline uint64_t canzero_get_levitation_board1_config_hash() {
  extern uint64_t __oe_levitation_board1_config_hash;
  return __oe_levitation_board1_config_hash;
}
static inline error_flag canzero_get_levitation_board1_assertion_fault() {
  extern error_flag __oe_levitation_board1_assertion_fault;
  return __oe_levitation_board1_assertion_fault;
}
static inline error_flag canzero_get_levitation_board1_error_arming_failed() {
  extern error_flag __oe_levitation_board1_error_arming_failed;
  return __oe_levitation_board1_error_arming_failed;
}
static inline error_flag canzero_get_levitation_board1_error_precharge_failed() {
  extern error_flag __oe_levitation_board1_error_precharge_failed;
  return __oe_levitation_board1_error_precharge_failed;
}
static inline error_flag canzero_get_levitation_board1_error_heartbeat_miss() {
  extern error_flag __oe_levitation_board1_error_heartbeat_miss;
  return __oe_levitation_board1_error_heartbeat_miss;
}
static inline error_level canzero_get_levitation_board1_error_level_vdc_voltage() {
  extern error_level __oe_levitation_board1_error_level_vdc_voltage;
  return __oe_levitation_board1_error_level_vdc_voltage;
}
static inline error_level canzero_get_levitation_board1_error_level_magnet_current_left() {
  extern error_level __oe_levitation_board1_error_level_magnet_current_left;
  return __oe_levitation_board1_error_level_magnet_current_left;
}
static inline error_level canzero_get_levitation_board1_error_level_magnet_current_right() {
  extern error_level __oe_levitation_board1_error_level_magnet_current_right;
  return __oe_levitation_board1_error_level_magnet_current_right;
}
static inline error_level canzero_get_levitation_board1_error_level_input_current() {
  extern error_level __oe_levitation_board1_error_level_input_current;
  return __oe_levitation_board1_error_level_input_current;
}
static inline error_level canzero_get_levitation_board1_error_level_magnet_temperature_left() {
  extern error_level __oe_levitation_board1_error_level_magnet_temperature_left;
  return __oe_levitation_board1_error_level_magnet_temperature_left;
}
static inline error_level canzero_get_levitation_board1_error_level_magnet_temperature_right() {
  extern error_level __oe_levitation_board1_error_level_magnet_temperature_right;
  return __oe_levitation_board1_error_level_magnet_temperature_right;
}
static inline error_level canzero_get_levitation_board1_error_level_mcu_temperature() {
  extern error_level __oe_levitation_board1_error_level_mcu_temperature;
  return __oe_levitation_board1_error_level_mcu_temperature;
}
static inline levitation_state canzero_get_levitation_board2_state() {
  extern levitation_state __oe_levitation_board2_state;
  return __oe_levitation_board2_state;
}
static inline sdc_status canzero_get_levitation_board2_sdc_status() {
  extern sdc_status __oe_levitation_board2_sdc_status;
  return __oe_levitation_board2_sdc_status;
}
static inline uint64_t canzero_get_levitation_board2_config_hash() {
  extern uint64_t __oe_levitation_board2_config_hash;
  return __oe_levitation_board2_config_hash;
}
static inline error_flag canzero_get_levitation_board2_assertion_fault() {
  extern error_flag __oe_levitation_board2_assertion_fault;
  return __oe_levitation_board2_assertion_fault;
}
static inline error_flag canzero_get_levitation_board2_error_arming_failed() {
  extern error_flag __oe_levitation_board2_error_arming_failed;
  return __oe_levitation_board2_error_arming_failed;
}
static inline error_flag canzero_get_levitation_board2_error_precharge_failed() {
  extern error_flag __oe_levitation_board2_error_precharge_failed;
  return __oe_levitation_board2_error_precharge_failed;
}
static inline error_flag canzero_get_levitation_board2_error_heartbeat_miss() {
  extern error_flag __oe_levitation_board2_error_heartbeat_miss;
  return __oe_levitation_board2_error_heartbeat_miss;
}
static inline error_level canzero_get_levitation_board2_error_level_vdc_voltage() {
  extern error_level __oe_levitation_board2_error_level_vdc_voltage;
  return __oe_levitation_board2_error_level_vdc_voltage;
}
static inline error_level canzero_get_levitation_board2_error_level_magnet_current_left() {
  extern error_level __oe_levitation_board2_error_level_magnet_current_left;
  return __oe_levitation_board2_error_level_magnet_current_left;
}
static inline error_level canzero_get_levitation_board2_error_level_magnet_current_right() {
  extern error_level __oe_levitation_board2_error_level_magnet_current_right;
  return __oe_levitation_board2_error_level_magnet_current_right;
}
static inline error_level canzero_get_levitation_board2_error_level_input_current() {
  extern error_level __oe_levitation_board2_error_level_input_current;
  return __oe_levitation_board2_error_level_input_current;
}
static inline error_level canzero_get_levitation_board2_error_level_magnet_temperature_left() {
  extern error_level __oe_levitation_board2_error_level_magnet_temperature_left;
  return __oe_levitation_board2_error_level_magnet_temperature_left;
}
static inline error_level canzero_get_levitation_board2_error_level_magnet_temperature_right() {
  extern error_level __oe_levitation_board2_error_level_magnet_temperature_right;
  return __oe_levitation_board2_error_level_magnet_temperature_right;
}
static inline error_level canzero_get_levitation_board2_error_level_mcu_temperature() {
  extern error_level __oe_levitation_board2_error_level_mcu_temperature;
  return __oe_levitation_board2_error_level_mcu_temperature;
}
static inline levitation_state canzero_get_levitation_board3_state() {
  extern levitation_state __oe_levitation_board3_state;
  return __oe_levitation_board3_state;
}
static inline sdc_status canzero_get_levitation_board3_sdc_status() {
  extern sdc_status __oe_levitation_board3_sdc_status;
  return __oe_levitation_board3_sdc_status;
}
static inline uint64_t canzero_get_levitation_board3_config_hash() {
  extern uint64_t __oe_levitation_board3_config_hash;
  return __oe_levitation_board3_config_hash;
}
static inline error_flag canzero_get_levitation_board3_assertion_fault() {
  extern error_flag __oe_levitation_board3_assertion_fault;
  return __oe_levitation_board3_assertion_fault;
}
static inline error_flag canzero_get_levitation_board3_error_arming_failed() {
  extern error_flag __oe_levitation_board3_error_arming_failed;
  return __oe_levitation_board3_error_arming_failed;
}
static inline error_flag canzero_get_levitation_board3_error_precharge_failed() {
  extern error_flag __oe_levitation_board3_error_precharge_failed;
  return __oe_levitation_board3_error_precharge_failed;
}
static inline error_flag canzero_get_levitation_board3_error_heartbeat_miss() {
  extern error_flag __oe_levitation_board3_error_heartbeat_miss;
  return __oe_levitation_board3_error_heartbeat_miss;
}
static inline error_level canzero_get_levitation_board3_error_level_vdc_voltage() {
  extern error_level __oe_levitation_board3_error_level_vdc_voltage;
  return __oe_levitation_board3_error_level_vdc_voltage;
}
static inline error_level canzero_get_levitation_board3_error_level_magnet_current_left() {
  extern error_level __oe_levitation_board3_error_level_magnet_current_left;
  return __oe_levitation_board3_error_level_magnet_current_left;
}
static inline error_level canzero_get_levitation_board3_error_level_magnet_current_right() {
  extern error_level __oe_levitation_board3_error_level_magnet_current_right;
  return __oe_levitation_board3_error_level_magnet_current_right;
}
static inline error_level canzero_get_levitation_board3_error_level_input_current() {
  extern error_level __oe_levitation_board3_error_level_input_current;
  return __oe_levitation_board3_error_level_input_current;
}
static inline error_level canzero_get_levitation_board3_error_level_magnet_temperature_left() {
  extern error_level __oe_levitation_board3_error_level_magnet_temperature_left;
  return __oe_levitation_board3_error_level_magnet_temperature_left;
}
static inline error_level canzero_get_levitation_board3_error_level_magnet_temperature_right() {
  extern error_level __oe_levitation_board3_error_level_magnet_temperature_right;
  return __oe_levitation_board3_error_level_magnet_temperature_right;
}
static inline error_level canzero_get_levitation_board3_error_level_mcu_temperature() {
  extern error_level __oe_levitation_board3_error_level_mcu_temperature;
  return __oe_levitation_board3_error_level_mcu_temperature;
}
static inline input_board_state canzero_get_input_board_state() {
  extern input_board_state __oe_input_board_state;
  return __oe_input_board_state;
}
static inline input_board_command canzero_get_input_board_command() {
  extern input_board_command __oe_input_board_command;
  return __oe_input_board_command;
}
static inline bool_t canzero_get_input_board_assert_45V_online() {
  extern bool_t __oe_input_board_assert_45V_online;
  return __oe_input_board_assert_45V_online;
}
static inline sdc_status canzero_get_input_board_sdc_status() {
  extern sdc_status __oe_input_board_sdc_status;
  return __oe_input_board_sdc_status;
}
static inline uint64_t canzero_get_input_board_config_hash() {
  extern uint64_t __oe_input_board_config_hash;
  return __oe_input_board_config_hash;
}
static inline error_flag canzero_get_input_board_assertion_fault() {
  extern error_flag __oe_input_board_assertion_fault;
  return __oe_input_board_assertion_fault;
}
static inline error_flag canzero_get_input_board_error_acceleration_out_of_range() {
  extern error_flag __oe_input_board_error_acceleration_out_of_range;
  return __oe_input_board_error_acceleration_out_of_range;
}
static inline error_flag canzero_get_input_board_error_lateral_acceleration_out_of_range() {
  extern error_flag __oe_input_board_error_lateral_acceleration_out_of_range;
  return __oe_input_board_error_lateral_acceleration_out_of_range;
}
static inline error_flag canzero_get_input_board_error_vertical_acceleration_out_of_range() {
  extern error_flag __oe_input_board_error_vertical_acceleration_out_of_range;
  return __oe_input_board_error_vertical_acceleration_out_of_range;
}
static inline error_flag canzero_get_input_board_error_acceleration_calibration_failed() {
  extern error_flag __oe_input_board_error_acceleration_calibration_failed;
  return __oe_input_board_error_acceleration_calibration_failed;
}
static inline error_flag canzero_get_input_board_error_bat24_voltage_invalid() {
  extern error_flag __oe_input_board_error_bat24_voltage_invalid;
  return __oe_input_board_error_bat24_voltage_invalid;
}
static inline error_flag canzero_get_input_board_error_bat24_current_invalid() {
  extern error_flag __oe_input_board_error_bat24_current_invalid;
  return __oe_input_board_error_bat24_current_invalid;
}
static inline error_flag canzero_get_input_board_error_supercap_voltage_invalid() {
  extern error_flag __oe_input_board_error_supercap_voltage_invalid;
  return __oe_input_board_error_supercap_voltage_invalid;
}
static inline error_flag canzero_get_input_board_error_link24_current_invalid() {
  extern error_flag __oe_input_board_error_link24_current_invalid;
  return __oe_input_board_error_link24_current_invalid;
}
static inline error_flag canzero_get_input_board_error_link45_voltage_invalid() {
  extern error_flag __oe_input_board_error_link45_voltage_invalid;
  return __oe_input_board_error_link45_voltage_invalid;
}
static inline error_flag canzero_get_input_board_error_link45_current_invalid() {
  extern error_flag __oe_input_board_error_link45_current_invalid;
  return __oe_input_board_error_link45_current_invalid;
}
static inline error_flag canzero_get_input_board_error_cooling_cycle_pressure_invalid() {
  extern error_flag __oe_input_board_error_cooling_cycle_pressure_invalid;
  return __oe_input_board_error_cooling_cycle_pressure_invalid;
}
static inline error_flag canzero_get_input_board_error_mcu_temperature_invalid() {
  extern error_flag __oe_input_board_error_mcu_temperature_invalid;
  return __oe_input_board_error_mcu_temperature_invalid;
}
static inline error_flag canzero_get_input_board_error_cooling_cycle_temperature_invalid() {
  extern error_flag __oe_input_board_error_cooling_cycle_temperature_invalid;
  return __oe_input_board_error_cooling_cycle_temperature_invalid;
}
static inline error_flag canzero_get_input_board_error_bat24_cell_temperature_1_invalid() {
  extern error_flag __oe_input_board_error_bat24_cell_temperature_1_invalid;
  return __oe_input_board_error_bat24_cell_temperature_1_invalid;
}
static inline error_flag canzero_get_input_board_error_bat24_cell_temperature_2_invalid() {
  extern error_flag __oe_input_board_error_bat24_cell_temperature_2_invalid;
  return __oe_input_board_error_bat24_cell_temperature_2_invalid;
}
static inline error_flag canzero_get_input_board_error_supercap_temperature_invalid() {
  extern error_flag __oe_input_board_error_supercap_temperature_invalid;
  return __oe_input_board_error_supercap_temperature_invalid;
}
static inline error_flag canzero_get_input_board_error_buck_temperature_invalid() {
  extern error_flag __oe_input_board_error_buck_temperature_invalid;
  return __oe_input_board_error_buck_temperature_invalid;
}
static inline error_flag canzero_get_input_board_error_sac_ebox_temperature_invalid() {
  extern error_flag __oe_input_board_error_sac_ebox_temperature_invalid;
  return __oe_input_board_error_sac_ebox_temperature_invalid;
}
static inline error_flag canzero_get_input_board_error_power_ebox_temperature_invalid() {
  extern error_flag __oe_input_board_error_power_ebox_temperature_invalid;
  return __oe_input_board_error_power_ebox_temperature_invalid;
}
static inline error_flag canzero_get_input_board_error_ambient_temperature_1_invalid() {
  extern error_flag __oe_input_board_error_ambient_temperature_1_invalid;
  return __oe_input_board_error_ambient_temperature_1_invalid;
}
static inline error_flag canzero_get_input_board_error_ambient_temperature_2_invalid() {
  extern error_flag __oe_input_board_error_ambient_temperature_2_invalid;
  return __oe_input_board_error_ambient_temperature_2_invalid;
}
static inline error_flag canzero_get_input_board_error_ambient_temperature_3_invalid() {
  extern error_flag __oe_input_board_error_ambient_temperature_3_invalid;
  return __oe_input_board_error_ambient_temperature_3_invalid;
}
static inline error_flag canzero_get_input_board_error_heartbeat_miss() {
  extern error_flag __oe_input_board_error_heartbeat_miss;
  return __oe_input_board_error_heartbeat_miss;
}
static inline error_level canzero_get_input_board_error_level_bat24_under_voltage() {
  extern error_level __oe_input_board_error_level_bat24_under_voltage;
  return __oe_input_board_error_level_bat24_under_voltage;
}
static inline error_level canzero_get_input_board_error_level_bat24_over_voltage() {
  extern error_level __oe_input_board_error_level_bat24_over_voltage;
  return __oe_input_board_error_level_bat24_over_voltage;
}
static inline error_level canzero_get_input_board_error_level_bat24_over_current() {
  extern error_level __oe_input_board_error_level_bat24_over_current;
  return __oe_input_board_error_level_bat24_over_current;
}
static inline error_level canzero_get_input_board_error_level_supercap_under_voltage() {
  extern error_level __oe_input_board_error_level_supercap_under_voltage;
  return __oe_input_board_error_level_supercap_under_voltage;
}
static inline error_level canzero_get_input_board_error_level_supercap_over_voltage() {
  extern error_level __oe_input_board_error_level_supercap_over_voltage;
  return __oe_input_board_error_level_supercap_over_voltage;
}
static inline error_level canzero_get_input_board_error_level_link24_over_current() {
  extern error_level __oe_input_board_error_level_link24_over_current;
  return __oe_input_board_error_level_link24_over_current;
}
static inline error_level canzero_get_input_board_error_level_link45_under_voltage() {
  extern error_level __oe_input_board_error_level_link45_under_voltage;
  return __oe_input_board_error_level_link45_under_voltage;
}
static inline error_level canzero_get_input_board_error_level_link45_over_voltage() {
  extern error_level __oe_input_board_error_level_link45_over_voltage;
  return __oe_input_board_error_level_link45_over_voltage;
}
static inline error_level canzero_get_input_board_error_level_link45_over_current() {
  extern error_level __oe_input_board_error_level_link45_over_current;
  return __oe_input_board_error_level_link45_over_current;
}
static inline error_level canzero_get_input_board_error_level_cooling_cycle_over_pressure() {
  extern error_level __oe_input_board_error_level_cooling_cycle_over_pressure;
  return __oe_input_board_error_level_cooling_cycle_over_pressure;
}
static inline error_level canzero_get_input_board_error_level_mcu_temperature() {
  extern error_level __oe_input_board_error_level_mcu_temperature;
  return __oe_input_board_error_level_mcu_temperature;
}
static inline error_level canzero_get_input_board_error_level_cooling_cycle_temperature() {
  extern error_level __oe_input_board_error_level_cooling_cycle_temperature;
  return __oe_input_board_error_level_cooling_cycle_temperature;
}
static inline error_level canzero_get_input_board_error_level_bat24_temperature() {
  extern error_level __oe_input_board_error_level_bat24_temperature;
  return __oe_input_board_error_level_bat24_temperature;
}
static inline error_level canzero_get_input_board_error_level_supercap_temperature() {
  extern error_level __oe_input_board_error_level_supercap_temperature;
  return __oe_input_board_error_level_supercap_temperature;
}
static inline error_level canzero_get_input_board_error_level_buck_temperature() {
  extern error_level __oe_input_board_error_level_buck_temperature;
  return __oe_input_board_error_level_buck_temperature;
}
static inline error_level canzero_get_input_board_error_level_sac_ebox_temperature() {
  extern error_level __oe_input_board_error_level_sac_ebox_temperature;
  return __oe_input_board_error_level_sac_ebox_temperature;
}
static inline error_level canzero_get_input_board_error_level_power_ebox_temperature() {
  extern error_level __oe_input_board_error_level_power_ebox_temperature;
  return __oe_input_board_error_level_power_ebox_temperature;
}
static inline error_level canzero_get_input_board_error_level_ambient_temperature() {
  extern error_level __oe_input_board_error_level_ambient_temperature;
  return __oe_input_board_error_level_ambient_temperature;
}
static inline pdu_12v_state canzero_get_power_board12_state() {
  extern pdu_12v_state __oe_power_board12_state;
  return __oe_power_board12_state;
}
static inline pdu_12v_command canzero_get_power_board12_command() {
  extern pdu_12v_command __oe_power_board12_command;
  return __oe_power_board12_command;
}
static inline sdc_status canzero_get_power_board12_sdc_status() {
  extern sdc_status __oe_power_board12_sdc_status;
  return __oe_power_board12_sdc_status;
}
static inline uint64_t canzero_get_power_board12_config_hash() {
  extern uint64_t __oe_power_board12_config_hash;
  return __oe_power_board12_config_hash;
}
static inline error_flag canzero_get_power_board12_assertion_fault() {
  extern error_flag __oe_power_board12_assertion_fault;
  return __oe_power_board12_assertion_fault;
}
static inline error_flag canzero_get_power_board12_error_any_short() {
  extern error_flag __oe_power_board12_error_any_short;
  return __oe_power_board12_error_any_short;
}
static inline error_flag canzero_get_power_board12_error_heartbeat_miss() {
  extern error_flag __oe_power_board12_error_heartbeat_miss;
  return __oe_power_board12_error_heartbeat_miss;
}
static inline error_level canzero_get_power_board12_error_level_mcu_temperature() {
  extern error_level __oe_power_board12_error_level_mcu_temperature;
  return __oe_power_board12_error_level_mcu_temperature;
}
static inline pdu_24v_state canzero_get_power_board24_state() {
  extern pdu_24v_state __oe_power_board24_state;
  return __oe_power_board24_state;
}
static inline pdu_24v_command canzero_get_power_board24_command() {
  extern pdu_24v_command __oe_power_board24_command;
  return __oe_power_board24_command;
}
static inline sdc_status canzero_get_power_board24_sdc_status() {
  extern sdc_status __oe_power_board24_sdc_status;
  return __oe_power_board24_sdc_status;
}
static inline bool_t canzero_get_power_board24_cooling_pump_channel_ctrl() {
  extern bool_t __oe_power_board24_cooling_pump_channel_ctrl;
  return __oe_power_board24_cooling_pump_channel_ctrl;
}
static inline uint64_t canzero_get_power_board24_config_hash() {
  extern uint64_t __oe_power_board24_config_hash;
  return __oe_power_board24_config_hash;
}
static inline error_flag canzero_get_power_board24_assertion_fault() {
  extern error_flag __oe_power_board24_assertion_fault;
  return __oe_power_board24_assertion_fault;
}
static inline error_flag canzero_get_power_board24_error_any_short() {
  extern error_flag __oe_power_board24_error_any_short;
  return __oe_power_board24_error_any_short;
}
static inline error_flag canzero_get_power_board24_error_heartbeat_miss() {
  extern error_flag __oe_power_board24_error_heartbeat_miss;
  return __oe_power_board24_error_heartbeat_miss;
}
static inline error_level canzero_get_power_board24_error_level_mcu_temperature() {
  extern error_level __oe_power_board24_error_level_mcu_temperature;
  return __oe_power_board24_error_level_mcu_temperature;
}
static inline led_board_state canzero_get_led_board_state() {
  extern led_board_state __oe_led_board_state;
  return __oe_led_board_state;
}
static inline led_board_command canzero_get_led_board_command() {
  extern led_board_command __oe_led_board_command;
  return __oe_led_board_command;
}
static inline sdc_status canzero_get_led_board_sdc_status() {
  extern sdc_status __oe_led_board_sdc_status;
  return __oe_led_board_sdc_status;
}
static inline uint64_t canzero_get_led_board_config_hash() {
  extern uint64_t __oe_led_board_config_hash;
  return __oe_led_board_config_hash;
}
static inline error_flag canzero_get_led_board_assertion_fault() {
  extern error_flag __oe_led_board_assertion_fault;
  return __oe_led_board_assertion_fault;
}
static inline error_flag canzero_get_led_board_error_heartbeat_miss() {
  extern error_flag __oe_led_board_error_heartbeat_miss;
  return __oe_led_board_error_heartbeat_miss;
}
static inline error_level canzero_get_led_board_error_level_mcu_temperature() {
  extern error_level __oe_led_board_error_level_mcu_temperature;
  return __oe_led_board_error_level_mcu_temperature;
}
static inline float canzero_get_gamepad_max_acceleration() {
  extern float __oe_gamepad_max_acceleration;
  return __oe_gamepad_max_acceleration;
}
static inline float canzero_get_gamepad_lt2() {
  extern float __oe_gamepad_lt2;
  return __oe_gamepad_lt2;
}
static inline float canzero_get_gamepad_rt2() {
  extern float __oe_gamepad_rt2;
  return __oe_gamepad_rt2;
}
static inline error_flag canzero_get_error_heartbeat_miss() {
  extern error_flag __oe_error_heartbeat_miss;
  return __oe_error_heartbeat_miss;
}
static inline uint8_t canzero_get_last_node_missed() {
  extern uint8_t __oe_last_node_missed;
  return __oe_last_node_missed;
}
static inline float canzero_get_target_airgap() {
  extern float __oe_target_airgap;
  return __oe_target_airgap;
}
static inline float canzero_get_airgap_transition_duration() {
  extern float __oe_airgap_transition_duration;
  return __oe_airgap_transition_duration;
}
static inline airgap_transition_mode canzero_get_airgap_transition_mode() {
  extern airgap_transition_mode __oe_airgap_transition_mode;
  return __oe_airgap_transition_mode;
}
static inline bool_t canzero_get_ignore_45v() {
  extern bool_t __oe_ignore_45v;
  return __oe_ignore_45v;
}
typedef struct {
  get_resp_header m_header;
  uint32_t m_data;
} canzero_message_get_resp;
static const uint32_t canzero_message_get_resp_id = 0x1BD;
typedef struct {
  set_resp_header m_header;
} canzero_message_set_resp;
static const uint32_t canzero_message_set_resp_id = 0x1DD;
typedef struct {
  global_state m_state;
  global_command m_command;
  sdc_status m_system_sdc_status;
} canzero_message_mother_board_stream_state;
static const uint32_t canzero_message_mother_board_stream_state_id = 0x90;
typedef struct {
  float m_target_acceleration;
  motor_command m_motor_driver_command;
  bool_t m_pod_grounded;
} canzero_message_mother_board_stream_motor_command;
static const uint32_t canzero_message_mother_board_stream_motor_command_id = 0x4C;
typedef struct {
  input_board_command m_input_board_command;
  bool_t m_input_board_assert_45V_online;
} canzero_message_mother_board_stream_input_board_command;
static const uint32_t canzero_message_mother_board_stream_input_board_command_id = 0x4E;
typedef struct {
  guidance_command m_guidance_command;
} canzero_message_mother_board_stream_guidance_command;
static const uint32_t canzero_message_mother_board_stream_guidance_command_id = 0x4F;
typedef struct {
  levitation_command m_levitation_command;
} canzero_message_mother_board_stream_levitation_command;
static const uint32_t canzero_message_mother_board_stream_levitation_command_id = 0x4D;
typedef struct {
  float m_target_airgap;
  float m_airgap_transition_duration;
  airgap_transition_mode m_airgap_transition_mode;
} canzero_message_mother_board_stream_levitation_conifg;
static const uint32_t canzero_message_mother_board_stream_levitation_conifg_id = 0x15A;
typedef struct {
  pdu_12v_command m_power_board12_command;
} canzero_message_mother_board_stream_pdu_12v_command;
static const uint32_t canzero_message_mother_board_stream_pdu_12v_command_id = 0x4B;
typedef struct {
  pdu_24v_command m_power_board24_command;
  bool_t m_power_board24_cooling_pump_channel_ctrl;
} canzero_message_mother_board_stream_pdu_24v_command;
static const uint32_t canzero_message_mother_board_stream_pdu_24v_command_id = 0x4A;
typedef struct {
  error_flag m_error_heartbeat_miss;
  error_flag m_error_any;
  error_level m_error_level_over_temperature_system;
  uint8_t m_last_node_missed;
} canzero_message_mother_board_stream_errors;
static const uint32_t canzero_message_mother_board_stream_errors_id = 0x50;
typedef struct {
  bool_t m_ignore_45v;
} canzero_message_mother_board_stream_debug_settings;
static const uint32_t canzero_message_mother_board_stream_debug_settings_id = 0x51;
typedef struct {
  float m_target_airgap;
  float m_airgap_transition_duration;
  airgap_transition_mode m_airgap_transition_mode;
} canzero_message_mother_board_stream_levitation_config;
static const uint32_t canzero_message_mother_board_stream_levitation_config_id = 0x6D;
typedef struct {
  led_board_command m_led_board_command;
} canzero_message_mother_board_stream_led_board_command;
static const uint32_t canzero_message_mother_board_stream_led_board_command_id = 0x70;
typedef struct {
  uint8_t m_node_id;
  uint8_t m_unregister;
  uint8_t m_ticks_next;
} canzero_message_heartbeat_can0;
static const uint32_t canzero_message_heartbeat_can0_id = 0x1F4;
typedef struct {
  uint8_t m_node_id;
  uint8_t m_unregister;
  uint8_t m_ticks_next;
} canzero_message_heartbeat_can1;
static const uint32_t canzero_message_heartbeat_can1_id = 0x1F3;
typedef struct {
  get_req_header m_header;
} canzero_message_get_req;
static const uint32_t canzero_message_get_req_id = 0x1BE;
typedef struct {
  set_req_header m_header;
  uint32_t m_data;
} canzero_message_set_req;
static const uint32_t canzero_message_set_req_id = 0x1DE;
typedef struct {
  motor_state m_state;
  sdc_status m_sdc_status;
  motor_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_motor_driver_stream_state;
static const uint32_t canzero_message_motor_driver_stream_state_id = 0x95;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_motor_driver_stream_config_hash;
static const uint32_t canzero_message_motor_driver_stream_config_hash_id = 0x55;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_arming_failed;
  error_flag m_error_precharge_failed;
  error_flag m_error_acceleration_out_of_range;
  error_flag m_error_acceleration_calibration_failed;
  error_flag m_error_invalid_target_acceleration;
  error_flag m_error_board_temperature1_invalid;
  error_flag m_error_board_temperature2_invalid;
  error_flag m_error_board_temperature3_invalid;
  error_flag m_error_mcu_temperature_invalid;
  error_flag m_error_lim_temperature1_invalid;
  error_flag m_error_lim_temperature2_invalid;
  error_flag m_error_lim_temperature3_invalid;
  error_flag m_error_lim_temperature4_invalid;
  error_flag m_error_heartbeat_miss;
  error_level m_error_level_current_u1;
  error_level m_error_level_current_v1;
  error_level m_error_level_current_w1;
  error_level m_error_level_current_u2;
  error_level m_error_level_current_v2;
  error_level m_error_level_current_w2;
  error_level m_error_level_vdc_voltage;
  error_level m_error_level_board_temperature;
  error_level m_error_level_mcu_temperature;
  error_level m_error_level_lim_temperature;
  error_flag m_error_vdc_voltage_invalid;
  uint8_t m_last_node_missed;
} canzero_message_motor_driver_stream_errors;
static const uint32_t canzero_message_motor_driver_stream_errors_id = 0x75;
typedef struct {
  guidance_state m_state;
  sdc_status m_sdc_status;
  guidance_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_guidance_board_front_stream_state;
static const uint32_t canzero_message_guidance_board_front_stream_state_id = 0xF7;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_guidance_board_front_stream_config_hash;
static const uint32_t canzero_message_guidance_board_front_stream_config_hash_id = 0xB7;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_arming_failed;
  error_flag m_error_precharge_failed;
  error_flag m_error_heartbeat_miss;
  error_flag m_error_outer_airgap_left_invalid;
  error_flag m_error_inner_airgap_left_invalid;
  error_flag m_error_outer_airgap_right_invalid;
  error_flag m_error_inner_airgap_right_invalid;
  error_flag m_error_vdc_voltage_invalid;
  error_flag m_error_magnet_current_left_invalid;
  error_flag m_error_magnet_current_right_invalid;
  error_flag m_error_input_current_invalid;
  error_flag m_error_magnet_temperature_left_invalid;
  error_flag m_error_magnet_temperature_right_invalid;
  error_flag m_error_mcu_temperature_invalid;
  error_level m_error_level_vdc_voltage;
  error_level m_error_level_magnet_current_left;
  error_level m_error_level_magnet_current_right;
  error_level m_error_level_input_current;
  error_level m_error_level_magnet_temperature_left;
  error_level m_error_level_magnet_temperature_right;
  error_level m_error_level_mcu_temperature;
  uint8_t m_last_node_missed;
} canzero_message_guidance_board_front_stream_errors;
static const uint32_t canzero_message_guidance_board_front_stream_errors_id = 0xD7;
typedef struct {
  guidance_state m_state;
  sdc_status m_sdc_status;
  guidance_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_guidance_board_back_stream_state;
static const uint32_t canzero_message_guidance_board_back_stream_state_id = 0x97;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_guidance_board_back_stream_config_hash;
static const uint32_t canzero_message_guidance_board_back_stream_config_hash_id = 0x57;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_arming_failed;
  error_flag m_error_precharge_failed;
  error_flag m_error_heartbeat_miss;
  error_flag m_error_outer_airgap_left_invalid;
  error_flag m_error_inner_airgap_left_invalid;
  error_flag m_error_outer_airgap_right_invalid;
  error_flag m_error_inner_airgap_right_invalid;
  error_flag m_error_vdc_voltage_invalid;
  error_flag m_error_magnet_current_left_invalid;
  error_flag m_error_magnet_current_right_invalid;
  error_flag m_error_input_current_invalid;
  error_flag m_error_magnet_temperature_left_invalid;
  error_flag m_error_magnet_temperature_right_invalid;
  error_flag m_error_mcu_temperature_invalid;
  error_level m_error_level_vdc_voltage;
  error_level m_error_level_magnet_current_left;
  error_level m_error_level_magnet_current_right;
  error_level m_error_level_input_current;
  error_level m_error_level_magnet_temperature_left;
  error_level m_error_level_magnet_temperature_right;
  error_level m_error_level_mcu_temperature;
  uint8_t m_last_node_missed;
} canzero_message_guidance_board_back_stream_errors;
static const uint32_t canzero_message_guidance_board_back_stream_errors_id = 0x77;
typedef struct {
  levitation_state m_state;
  sdc_status m_sdc_status;
  levitation_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_levitation_board1_stream_state;
static const uint32_t canzero_message_levitation_board1_stream_state_id = 0xD6;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_levitation_board1_stream_config_hash;
static const uint32_t canzero_message_levitation_board1_stream_config_hash_id = 0x96;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_arming_failed;
  error_flag m_error_precharge_failed;
  error_flag m_error_heartbeat_miss;
  error_level m_error_level_vdc_voltage;
  error_level m_error_level_magnet_current_left;
  error_level m_error_level_magnet_current_right;
  error_level m_error_level_input_current;
  error_level m_error_level_magnet_temperature_left;
  error_level m_error_level_magnet_temperature_right;
  error_level m_error_level_mcu_temperature;
  error_flag m_error_airgap_left_invalid;
  error_flag m_error_airgap_right_invalid;
  error_flag m_error_vdc_voltage_invalid;
  error_flag m_error_magnet_current_left_invalid;
  error_flag m_error_magnet_current_right_invalid;
  error_flag m_error_magnet_temperature_left_invalid;
  error_flag m_error_magnet_temperature_right_invalid;
  error_flag m_error_mcu_temperature_invalid;
  uint8_t m_last_node_missed;
} canzero_message_levitation_board1_stream_errors;
static const uint32_t canzero_message_levitation_board1_stream_errors_id = 0xB6;
typedef struct {
  levitation_state m_state;
  sdc_status m_sdc_status;
  levitation_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_levitation_board2_stream_state;
static const uint32_t canzero_message_levitation_board2_stream_state_id = 0x136;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_levitation_board2_stream_config_hash;
static const uint32_t canzero_message_levitation_board2_stream_config_hash_id = 0xF6;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_arming_failed;
  error_flag m_error_precharge_failed;
  error_flag m_error_heartbeat_miss;
  error_level m_error_level_vdc_voltage;
  error_level m_error_level_magnet_current_left;
  error_level m_error_level_magnet_current_right;
  error_level m_error_level_input_current;
  error_level m_error_level_magnet_temperature_left;
  error_level m_error_level_magnet_temperature_right;
  error_level m_error_level_mcu_temperature;
  error_flag m_error_airgap_left_invalid;
  error_flag m_error_airgap_right_invalid;
  error_flag m_error_vdc_voltage_invalid;
  error_flag m_error_magnet_current_left_invalid;
  error_flag m_error_magnet_current_right_invalid;
  error_flag m_error_magnet_temperature_left_invalid;
  error_flag m_error_magnet_temperature_right_invalid;
  error_flag m_error_mcu_temperature_invalid;
  uint8_t m_last_node_missed;
} canzero_message_levitation_board2_stream_errors;
static const uint32_t canzero_message_levitation_board2_stream_errors_id = 0x116;
typedef struct {
  levitation_state m_state;
  sdc_status m_sdc_status;
  levitation_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_levitation_board3_stream_state;
static const uint32_t canzero_message_levitation_board3_stream_state_id = 0x196;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_levitation_board3_stream_config_hash;
static const uint32_t canzero_message_levitation_board3_stream_config_hash_id = 0x156;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_arming_failed;
  error_flag m_error_precharge_failed;
  error_flag m_error_heartbeat_miss;
  error_level m_error_level_vdc_voltage;
  error_level m_error_level_magnet_current_left;
  error_level m_error_level_magnet_current_right;
  error_level m_error_level_input_current;
  error_level m_error_level_magnet_temperature_left;
  error_level m_error_level_magnet_temperature_right;
  error_level m_error_level_mcu_temperature;
  error_flag m_error_airgap_left_invalid;
  error_flag m_error_airgap_right_invalid;
  error_flag m_error_vdc_voltage_invalid;
  error_flag m_error_magnet_current_left_invalid;
  error_flag m_error_magnet_current_right_invalid;
  error_flag m_error_magnet_temperature_left_invalid;
  error_flag m_error_magnet_temperature_right_invalid;
  error_flag m_error_mcu_temperature_invalid;
  uint8_t m_last_node_missed;
} canzero_message_levitation_board3_stream_errors;
static const uint32_t canzero_message_levitation_board3_stream_errors_id = 0x176;
typedef struct {
  input_board_state m_state;
  sdc_status m_sdc_status;
} canzero_message_input_board_stream_state;
static const uint32_t canzero_message_input_board_stream_state_id = 0x177;
typedef struct {
  float m_position;
  float m_velocity;
  float m_acceleration;
} canzero_message_input_board_stream_position_estimation;
static const uint32_t canzero_message_input_board_stream_position_estimation_id = 0x157;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_input_board_stream_config_hash;
static const uint32_t canzero_message_input_board_stream_config_hash_id = 0x117;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_acceleration_out_of_range;
  error_flag m_error_lateral_acceleration_out_of_range;
  error_flag m_error_vertical_acceleration_out_of_range;
  error_flag m_error_acceleration_calibration_failed;
  error_flag m_error_bat24_voltage_invalid;
  error_flag m_error_bat24_current_invalid;
  error_flag m_error_supercap_voltage_invalid;
  error_flag m_error_link24_current_invalid;
  error_flag m_error_link45_voltage_invalid;
  error_flag m_error_link45_current_invalid;
  error_flag m_error_mcu_temperature_invalid;
  error_flag m_error_sac_ebox_temperature_invalid;
  error_flag m_error_power_ebox_temperature_invalid;
  error_flag m_error_bat24_cell_temperature_1_invalid;
  error_flag m_error_bat24_cell_temperature_2_invalid;
  error_flag m_error_supercap_temperature_invalid;
  error_flag m_error_ambient_temperature_1_invalid;
  error_flag m_error_ambient_temperature_2_invalid;
  error_flag m_error_ambient_temperature_3_invalid;
  error_flag m_error_heartbeat_miss;
  error_level m_error_level_bat24_under_voltage;
  error_level m_error_level_bat24_over_voltage;
  error_level m_error_level_bat24_over_current;
} canzero_message_input_board_stream_errors;
static const uint32_t canzero_message_input_board_stream_errors_id = 0x52;
typedef struct {
  error_level m_error_level_supercap_under_voltage;
  error_level m_error_level_supercap_over_voltage;
  error_level m_error_level_link24_over_current;
  error_level m_error_level_link45_under_voltage;
  error_level m_error_level_link45_over_voltage;
  error_level m_error_level_link45_over_current;
  error_level m_error_level_mcu_temperature;
  error_level m_error_level_bat24_temperature;
  error_level m_error_level_supercap_temperature;
  error_level m_error_level_sac_ebox_temperature;
  error_level m_error_level_power_ebox_temperature;
  error_level m_error_level_ambient_temperature;
  uint8_t m_last_node_missed;
} canzero_message_input_board_stream_errors2;
static const uint32_t canzero_message_input_board_stream_errors2_id = 0x137;
typedef struct {
  pdu_12v_state m_state;
  sdc_status m_sdc_status;
} canzero_message_power_board12_stream_state;
static const uint32_t canzero_message_power_board12_stream_state_id = 0xF5;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_power_board12_stream_config_hash;
static const uint32_t canzero_message_power_board12_stream_config_hash_id = 0xB5;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_any_short;
  error_flag m_error_heartbeat_miss;
  error_level m_error_level_mcu_temperature;
  error_flag m_error_mcu_temperature_invalid;
  uint8_t m_last_node_missed;
} canzero_message_power_board12_stream_errors;
static const uint32_t canzero_message_power_board12_stream_errors_id = 0xD5;
typedef struct {
  pdu_24v_state m_state;
  sdc_status m_sdc_status;
} canzero_message_power_board24_stream_state;
static const uint32_t canzero_message_power_board24_stream_state_id = 0x155;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_power_board24_stream_config_hash;
static const uint32_t canzero_message_power_board24_stream_config_hash_id = 0x115;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_any_short;
  error_flag m_error_heartbeat_miss;
  error_level m_error_level_mcu_temperature;
  error_flag m_error_mcu_temperature_invalid;
  uint8_t m_last_node_missed;
} canzero_message_power_board24_stream_errors;
static const uint32_t canzero_message_power_board24_stream_errors_id = 0x135;
typedef struct {
  led_board_state m_state;
  sdc_status m_sdc_status;
} canzero_message_led_board_stream_state;
static const uint32_t canzero_message_led_board_stream_state_id = 0x76;
typedef struct {
  uint64_t m_config_hash;
} canzero_message_led_board_stream_config_hash;
static const uint32_t canzero_message_led_board_stream_config_hash_id = 0x197;
typedef struct {
  error_flag m_assertion_fault;
  error_flag m_error_heartbeat_miss;
  error_level m_error_level_mcu_temperature;
  uint8_t m_last_node_missed;
} canzero_message_led_board_stream_errors;
static const uint32_t canzero_message_led_board_stream_errors_id = 0x56;
typedef struct {
  float m_lt2;
  float m_rt2;
  float m_lsb_x;
  float m_lsb_y;
  float m_rsb_x;
  float m_rsb_y;
  bool_t m_lt1_down;
  bool_t m_rt1_down;
  bool_t m_x_down;
  bool_t m_y_down;
  bool_t m_b_down;
  bool_t m_a_down;
  bool_t m_lsb_down;
  bool_t m_rsb_down;
} canzero_message_gamepad_stream_input;
static const uint32_t canzero_message_gamepad_stream_input_id = 0x5F;
void canzero_can0_poll();
void canzero_can1_poll();
uint32_t canzero_update_continue(uint32_t delta_time);
void canzero_init();
static inline void canzero_set_config_hash(uint64_t value){
  extern uint64_t __oe_config_hash;
  __oe_config_hash = value;
}

static inline void canzero_set_build_time(date_time value){
  extern date_time __oe_build_time;
  __oe_build_time = value;
}

void canzero_set_state(global_state value);

void canzero_set_command(global_command value);

void canzero_set_system_sdc_status(sdc_status value);

static inline void canzero_set_track_length(float value){
  extern float __oe_track_length;
  __oe_track_length = value;
}

static inline void canzero_set_brake_margin(float value){
  extern float __oe_brake_margin;
  __oe_brake_margin = value;
}

static inline void canzero_set_emergency_brake_margin(float value){
  extern float __oe_emergency_brake_margin;
  __oe_emergency_brake_margin = value;
}

void canzero_set_target_acceleration(float value);

void canzero_set_pod_grounded(bool_t value);

static inline void canzero_set_acceleration_target_velocity(float value){
  extern float __oe_acceleration_target_velocity;
  __oe_acceleration_target_velocity = value;
}

static inline void canzero_set_velocity_pid(pid_parameters value){
  extern pid_parameters __oe_velocity_pid;
  __oe_velocity_pid = value;
}

static inline void canzero_set_position(float value){
  extern float __oe_position;
  __oe_position = value;
}

static inline void canzero_set_velocity(float value){
  extern float __oe_velocity;
  __oe_velocity = value;
}

static inline void canzero_set_acceleration(float value){
  extern float __oe_acceleration;
  __oe_acceleration = value;
}

static inline void canzero_set_error_level_config_consistency(error_level value){
  extern error_level __oe_error_level_config_consistency;
  __oe_error_level_config_consistency = value;
}

void canzero_set_error_any(error_flag value);

void canzero_set_error_level_over_temperature_system(error_level value);

static inline void canzero_set_motor_driver_state(motor_state value){
  extern motor_state __oe_motor_driver_state;
  __oe_motor_driver_state = value;
}

void canzero_set_motor_driver_command(motor_command value);

static inline void canzero_set_motor_driver_sdc_status(sdc_status value){
  extern sdc_status __oe_motor_driver_sdc_status;
  __oe_motor_driver_sdc_status = value;
}

static inline void canzero_set_motor_driver_config_hash(uint64_t value){
  extern uint64_t __oe_motor_driver_config_hash;
  __oe_motor_driver_config_hash = value;
}

static inline void canzero_set_motor_driver_assertion_fault(error_flag value){
  extern error_flag __oe_motor_driver_assertion_fault;
  __oe_motor_driver_assertion_fault = value;
}

static inline void canzero_set_motor_driver_error_arming_failed(error_flag value){
  extern error_flag __oe_motor_driver_error_arming_failed;
  __oe_motor_driver_error_arming_failed = value;
}

static inline void canzero_set_motor_driver_error_precharge_failed(error_flag value){
  extern error_flag __oe_motor_driver_error_precharge_failed;
  __oe_motor_driver_error_precharge_failed = value;
}

static inline void canzero_set_motor_driver_error_acceleration_out_of_range(error_flag value){
  extern error_flag __oe_motor_driver_error_acceleration_out_of_range;
  __oe_motor_driver_error_acceleration_out_of_range = value;
}

static inline void canzero_set_motor_driver_error_acceleration_calibration_failed(error_flag value){
  extern error_flag __oe_motor_driver_error_acceleration_calibration_failed;
  __oe_motor_driver_error_acceleration_calibration_failed = value;
}

static inline void canzero_set_motor_driver_error_invalid_target_acceleration(error_flag value){
  extern error_flag __oe_motor_driver_error_invalid_target_acceleration;
  __oe_motor_driver_error_invalid_target_acceleration = value;
}

static inline void canzero_set_motor_driver_error_board_temperature1_invalid(error_flag value){
  extern error_flag __oe_motor_driver_error_board_temperature1_invalid;
  __oe_motor_driver_error_board_temperature1_invalid = value;
}

static inline void canzero_set_motor_driver_error_board_temperature2_invalid(error_flag value){
  extern error_flag __oe_motor_driver_error_board_temperature2_invalid;
  __oe_motor_driver_error_board_temperature2_invalid = value;
}

static inline void canzero_set_motor_driver_error_board_temperature3_invalid(error_flag value){
  extern error_flag __oe_motor_driver_error_board_temperature3_invalid;
  __oe_motor_driver_error_board_temperature3_invalid = value;
}

static inline void canzero_set_motor_driver_error_mcu_temperature_invalid(error_flag value){
  extern error_flag __oe_motor_driver_error_mcu_temperature_invalid;
  __oe_motor_driver_error_mcu_temperature_invalid = value;
}

static inline void canzero_set_motor_driver_error_lim_temperature1_invalid(error_flag value){
  extern error_flag __oe_motor_driver_error_lim_temperature1_invalid;
  __oe_motor_driver_error_lim_temperature1_invalid = value;
}

static inline void canzero_set_motor_driver_error_lim_temperature2_invalid(error_flag value){
  extern error_flag __oe_motor_driver_error_lim_temperature2_invalid;
  __oe_motor_driver_error_lim_temperature2_invalid = value;
}

static inline void canzero_set_motor_driver_error_lim_temperature3_invalid(error_flag value){
  extern error_flag __oe_motor_driver_error_lim_temperature3_invalid;
  __oe_motor_driver_error_lim_temperature3_invalid = value;
}

static inline void canzero_set_motor_driver_error_lim_temperature4_invalid(error_flag value){
  extern error_flag __oe_motor_driver_error_lim_temperature4_invalid;
  __oe_motor_driver_error_lim_temperature4_invalid = value;
}

static inline void canzero_set_motor_driver_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_motor_driver_error_heartbeat_miss;
  __oe_motor_driver_error_heartbeat_miss = value;
}

static inline void canzero_set_motor_driver_error_level_current_u1(error_level value){
  extern error_level __oe_motor_driver_error_level_current_u1;
  __oe_motor_driver_error_level_current_u1 = value;
}

static inline void canzero_set_motor_driver_error_level_current_v1(error_level value){
  extern error_level __oe_motor_driver_error_level_current_v1;
  __oe_motor_driver_error_level_current_v1 = value;
}

static inline void canzero_set_motor_driver_error_level_current_w1(error_level value){
  extern error_level __oe_motor_driver_error_level_current_w1;
  __oe_motor_driver_error_level_current_w1 = value;
}

static inline void canzero_set_motor_driver_error_level_current_u2(error_level value){
  extern error_level __oe_motor_driver_error_level_current_u2;
  __oe_motor_driver_error_level_current_u2 = value;
}

static inline void canzero_set_motor_driver_error_level_current_v2(error_level value){
  extern error_level __oe_motor_driver_error_level_current_v2;
  __oe_motor_driver_error_level_current_v2 = value;
}

static inline void canzero_set_motor_driver_error_level_current_w2(error_level value){
  extern error_level __oe_motor_driver_error_level_current_w2;
  __oe_motor_driver_error_level_current_w2 = value;
}

static inline void canzero_set_motor_driver_error_level_vdc_voltage(error_level value){
  extern error_level __oe_motor_driver_error_level_vdc_voltage;
  __oe_motor_driver_error_level_vdc_voltage = value;
}

static inline void canzero_set_motor_driver_error_level_board_temperature(error_level value){
  extern error_level __oe_motor_driver_error_level_board_temperature;
  __oe_motor_driver_error_level_board_temperature = value;
}

static inline void canzero_set_motor_driver_error_level_mcu_temperature(error_level value){
  extern error_level __oe_motor_driver_error_level_mcu_temperature;
  __oe_motor_driver_error_level_mcu_temperature = value;
}

static inline void canzero_set_motor_driver_error_level_lim_temperature(error_level value){
  extern error_level __oe_motor_driver_error_level_lim_temperature;
  __oe_motor_driver_error_level_lim_temperature = value;
}

void canzero_set_guidance_command(guidance_command value);

static inline void canzero_set_guidance_board_front_state(guidance_state value){
  extern guidance_state __oe_guidance_board_front_state;
  __oe_guidance_board_front_state = value;
}

static inline void canzero_set_guidance_board_front_sdc_status(sdc_status value){
  extern sdc_status __oe_guidance_board_front_sdc_status;
  __oe_guidance_board_front_sdc_status = value;
}

static inline void canzero_set_guidance_board_front_config_hash(uint64_t value){
  extern uint64_t __oe_guidance_board_front_config_hash;
  __oe_guidance_board_front_config_hash = value;
}

static inline void canzero_set_guidance_board_front_assertion_fault(error_flag value){
  extern error_flag __oe_guidance_board_front_assertion_fault;
  __oe_guidance_board_front_assertion_fault = value;
}

static inline void canzero_set_guidance_board_front_error_arming_failed(error_flag value){
  extern error_flag __oe_guidance_board_front_error_arming_failed;
  __oe_guidance_board_front_error_arming_failed = value;
}

static inline void canzero_set_guidance_board_front_error_precharge_failed(error_flag value){
  extern error_flag __oe_guidance_board_front_error_precharge_failed;
  __oe_guidance_board_front_error_precharge_failed = value;
}

static inline void canzero_set_guidance_board_front_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_guidance_board_front_error_heartbeat_miss;
  __oe_guidance_board_front_error_heartbeat_miss = value;
}

static inline void canzero_set_guidance_board_front_error_outer_airgap_left_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_outer_airgap_left_invalid;
  __oe_guidance_board_front_error_outer_airgap_left_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_inner_airgap_left_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_inner_airgap_left_invalid;
  __oe_guidance_board_front_error_inner_airgap_left_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_outer_airgap_right_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_outer_airgap_right_invalid;
  __oe_guidance_board_front_error_outer_airgap_right_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_inner_airgap_right_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_inner_airgap_right_invalid;
  __oe_guidance_board_front_error_inner_airgap_right_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_vdc_voltage_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_vdc_voltage_invalid;
  __oe_guidance_board_front_error_vdc_voltage_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_magnet_current_left_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_magnet_current_left_invalid;
  __oe_guidance_board_front_error_magnet_current_left_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_magnet_current_right_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_magnet_current_right_invalid;
  __oe_guidance_board_front_error_magnet_current_right_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_input_current_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_input_current_invalid;
  __oe_guidance_board_front_error_input_current_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_magnet_temperature_left_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_magnet_temperature_left_invalid;
  __oe_guidance_board_front_error_magnet_temperature_left_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_magnet_temperature_right_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_magnet_temperature_right_invalid;
  __oe_guidance_board_front_error_magnet_temperature_right_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_mcu_temperature_invalid(error_flag value){
  extern error_flag __oe_guidance_board_front_error_mcu_temperature_invalid;
  __oe_guidance_board_front_error_mcu_temperature_invalid = value;
}

static inline void canzero_set_guidance_board_front_error_level_vdc_voltage(error_level value){
  extern error_level __oe_guidance_board_front_error_level_vdc_voltage;
  __oe_guidance_board_front_error_level_vdc_voltage = value;
}

static inline void canzero_set_guidance_board_front_error_level_magnet_current_left(error_level value){
  extern error_level __oe_guidance_board_front_error_level_magnet_current_left;
  __oe_guidance_board_front_error_level_magnet_current_left = value;
}

static inline void canzero_set_guidance_board_front_error_level_magnet_current_right(error_level value){
  extern error_level __oe_guidance_board_front_error_level_magnet_current_right;
  __oe_guidance_board_front_error_level_magnet_current_right = value;
}

static inline void canzero_set_guidance_board_front_error_level_input_current(error_level value){
  extern error_level __oe_guidance_board_front_error_level_input_current;
  __oe_guidance_board_front_error_level_input_current = value;
}

static inline void canzero_set_guidance_board_front_error_level_magnet_temperature_left(error_level value){
  extern error_level __oe_guidance_board_front_error_level_magnet_temperature_left;
  __oe_guidance_board_front_error_level_magnet_temperature_left = value;
}

static inline void canzero_set_guidance_board_front_error_level_magnet_temperature_right(error_level value){
  extern error_level __oe_guidance_board_front_error_level_magnet_temperature_right;
  __oe_guidance_board_front_error_level_magnet_temperature_right = value;
}

static inline void canzero_set_guidance_board_front_error_level_mcu_temperature(error_level value){
  extern error_level __oe_guidance_board_front_error_level_mcu_temperature;
  __oe_guidance_board_front_error_level_mcu_temperature = value;
}

static inline void canzero_set_guidance_board_back_state(guidance_state value){
  extern guidance_state __oe_guidance_board_back_state;
  __oe_guidance_board_back_state = value;
}

static inline void canzero_set_guidance_board_back_sdc_status(sdc_status value){
  extern sdc_status __oe_guidance_board_back_sdc_status;
  __oe_guidance_board_back_sdc_status = value;
}

static inline void canzero_set_guidance_board_back_config_hash(uint64_t value){
  extern uint64_t __oe_guidance_board_back_config_hash;
  __oe_guidance_board_back_config_hash = value;
}

static inline void canzero_set_guidance_board_back_assertion_fault(error_flag value){
  extern error_flag __oe_guidance_board_back_assertion_fault;
  __oe_guidance_board_back_assertion_fault = value;
}

static inline void canzero_set_guidance_board_back_error_arming_failed(error_flag value){
  extern error_flag __oe_guidance_board_back_error_arming_failed;
  __oe_guidance_board_back_error_arming_failed = value;
}

static inline void canzero_set_guidance_board_back_error_precharge_failed(error_flag value){
  extern error_flag __oe_guidance_board_back_error_precharge_failed;
  __oe_guidance_board_back_error_precharge_failed = value;
}

static inline void canzero_set_guidance_board_back_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_guidance_board_back_error_heartbeat_miss;
  __oe_guidance_board_back_error_heartbeat_miss = value;
}

static inline void canzero_set_guidance_board_back_error_outer_airgap_left_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_outer_airgap_left_invalid;
  __oe_guidance_board_back_error_outer_airgap_left_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_inner_airgap_left_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_inner_airgap_left_invalid;
  __oe_guidance_board_back_error_inner_airgap_left_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_outer_airgap_right_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_outer_airgap_right_invalid;
  __oe_guidance_board_back_error_outer_airgap_right_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_inner_airgap_right_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_inner_airgap_right_invalid;
  __oe_guidance_board_back_error_inner_airgap_right_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_vdc_voltage_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_vdc_voltage_invalid;
  __oe_guidance_board_back_error_vdc_voltage_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_magnet_current_left_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_magnet_current_left_invalid;
  __oe_guidance_board_back_error_magnet_current_left_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_magnet_current_right_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_magnet_current_right_invalid;
  __oe_guidance_board_back_error_magnet_current_right_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_input_current_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_input_current_invalid;
  __oe_guidance_board_back_error_input_current_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_magnet_temperature_left_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_magnet_temperature_left_invalid;
  __oe_guidance_board_back_error_magnet_temperature_left_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_magnet_temperature_right_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_magnet_temperature_right_invalid;
  __oe_guidance_board_back_error_magnet_temperature_right_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_mcu_temperature_invalid(error_flag value){
  extern error_flag __oe_guidance_board_back_error_mcu_temperature_invalid;
  __oe_guidance_board_back_error_mcu_temperature_invalid = value;
}

static inline void canzero_set_guidance_board_back_error_level_vdc_voltage(error_level value){
  extern error_level __oe_guidance_board_back_error_level_vdc_voltage;
  __oe_guidance_board_back_error_level_vdc_voltage = value;
}

static inline void canzero_set_guidance_board_back_error_level_magnet_current_left(error_level value){
  extern error_level __oe_guidance_board_back_error_level_magnet_current_left;
  __oe_guidance_board_back_error_level_magnet_current_left = value;
}

static inline void canzero_set_guidance_board_back_error_level_magnet_current_right(error_level value){
  extern error_level __oe_guidance_board_back_error_level_magnet_current_right;
  __oe_guidance_board_back_error_level_magnet_current_right = value;
}

static inline void canzero_set_guidance_board_back_error_level_input_current(error_level value){
  extern error_level __oe_guidance_board_back_error_level_input_current;
  __oe_guidance_board_back_error_level_input_current = value;
}

static inline void canzero_set_guidance_board_back_error_level_magnet_temperature_left(error_level value){
  extern error_level __oe_guidance_board_back_error_level_magnet_temperature_left;
  __oe_guidance_board_back_error_level_magnet_temperature_left = value;
}

static inline void canzero_set_guidance_board_back_error_level_magnet_temperature_right(error_level value){
  extern error_level __oe_guidance_board_back_error_level_magnet_temperature_right;
  __oe_guidance_board_back_error_level_magnet_temperature_right = value;
}

static inline void canzero_set_guidance_board_back_error_level_mcu_temperature(error_level value){
  extern error_level __oe_guidance_board_back_error_level_mcu_temperature;
  __oe_guidance_board_back_error_level_mcu_temperature = value;
}

void canzero_set_levitation_command(levitation_command value);

static inline void canzero_set_levitation_board1_state(levitation_state value){
  extern levitation_state __oe_levitation_board1_state;
  __oe_levitation_board1_state = value;
}

static inline void canzero_set_levitation_board1_sdc_status(sdc_status value){
  extern sdc_status __oe_levitation_board1_sdc_status;
  __oe_levitation_board1_sdc_status = value;
}

static inline void canzero_set_levitation_board1_config_hash(uint64_t value){
  extern uint64_t __oe_levitation_board1_config_hash;
  __oe_levitation_board1_config_hash = value;
}

static inline void canzero_set_levitation_board1_assertion_fault(error_flag value){
  extern error_flag __oe_levitation_board1_assertion_fault;
  __oe_levitation_board1_assertion_fault = value;
}

static inline void canzero_set_levitation_board1_error_arming_failed(error_flag value){
  extern error_flag __oe_levitation_board1_error_arming_failed;
  __oe_levitation_board1_error_arming_failed = value;
}

static inline void canzero_set_levitation_board1_error_precharge_failed(error_flag value){
  extern error_flag __oe_levitation_board1_error_precharge_failed;
  __oe_levitation_board1_error_precharge_failed = value;
}

static inline void canzero_set_levitation_board1_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_levitation_board1_error_heartbeat_miss;
  __oe_levitation_board1_error_heartbeat_miss = value;
}

static inline void canzero_set_levitation_board1_error_level_vdc_voltage(error_level value){
  extern error_level __oe_levitation_board1_error_level_vdc_voltage;
  __oe_levitation_board1_error_level_vdc_voltage = value;
}

static inline void canzero_set_levitation_board1_error_level_magnet_current_left(error_level value){
  extern error_level __oe_levitation_board1_error_level_magnet_current_left;
  __oe_levitation_board1_error_level_magnet_current_left = value;
}

static inline void canzero_set_levitation_board1_error_level_magnet_current_right(error_level value){
  extern error_level __oe_levitation_board1_error_level_magnet_current_right;
  __oe_levitation_board1_error_level_magnet_current_right = value;
}

static inline void canzero_set_levitation_board1_error_level_input_current(error_level value){
  extern error_level __oe_levitation_board1_error_level_input_current;
  __oe_levitation_board1_error_level_input_current = value;
}

static inline void canzero_set_levitation_board1_error_level_magnet_temperature_left(error_level value){
  extern error_level __oe_levitation_board1_error_level_magnet_temperature_left;
  __oe_levitation_board1_error_level_magnet_temperature_left = value;
}

static inline void canzero_set_levitation_board1_error_level_magnet_temperature_right(error_level value){
  extern error_level __oe_levitation_board1_error_level_magnet_temperature_right;
  __oe_levitation_board1_error_level_magnet_temperature_right = value;
}

static inline void canzero_set_levitation_board1_error_level_mcu_temperature(error_level value){
  extern error_level __oe_levitation_board1_error_level_mcu_temperature;
  __oe_levitation_board1_error_level_mcu_temperature = value;
}

static inline void canzero_set_levitation_board2_state(levitation_state value){
  extern levitation_state __oe_levitation_board2_state;
  __oe_levitation_board2_state = value;
}

static inline void canzero_set_levitation_board2_sdc_status(sdc_status value){
  extern sdc_status __oe_levitation_board2_sdc_status;
  __oe_levitation_board2_sdc_status = value;
}

static inline void canzero_set_levitation_board2_config_hash(uint64_t value){
  extern uint64_t __oe_levitation_board2_config_hash;
  __oe_levitation_board2_config_hash = value;
}

static inline void canzero_set_levitation_board2_assertion_fault(error_flag value){
  extern error_flag __oe_levitation_board2_assertion_fault;
  __oe_levitation_board2_assertion_fault = value;
}

static inline void canzero_set_levitation_board2_error_arming_failed(error_flag value){
  extern error_flag __oe_levitation_board2_error_arming_failed;
  __oe_levitation_board2_error_arming_failed = value;
}

static inline void canzero_set_levitation_board2_error_precharge_failed(error_flag value){
  extern error_flag __oe_levitation_board2_error_precharge_failed;
  __oe_levitation_board2_error_precharge_failed = value;
}

static inline void canzero_set_levitation_board2_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_levitation_board2_error_heartbeat_miss;
  __oe_levitation_board2_error_heartbeat_miss = value;
}

static inline void canzero_set_levitation_board2_error_level_vdc_voltage(error_level value){
  extern error_level __oe_levitation_board2_error_level_vdc_voltage;
  __oe_levitation_board2_error_level_vdc_voltage = value;
}

static inline void canzero_set_levitation_board2_error_level_magnet_current_left(error_level value){
  extern error_level __oe_levitation_board2_error_level_magnet_current_left;
  __oe_levitation_board2_error_level_magnet_current_left = value;
}

static inline void canzero_set_levitation_board2_error_level_magnet_current_right(error_level value){
  extern error_level __oe_levitation_board2_error_level_magnet_current_right;
  __oe_levitation_board2_error_level_magnet_current_right = value;
}

static inline void canzero_set_levitation_board2_error_level_input_current(error_level value){
  extern error_level __oe_levitation_board2_error_level_input_current;
  __oe_levitation_board2_error_level_input_current = value;
}

static inline void canzero_set_levitation_board2_error_level_magnet_temperature_left(error_level value){
  extern error_level __oe_levitation_board2_error_level_magnet_temperature_left;
  __oe_levitation_board2_error_level_magnet_temperature_left = value;
}

static inline void canzero_set_levitation_board2_error_level_magnet_temperature_right(error_level value){
  extern error_level __oe_levitation_board2_error_level_magnet_temperature_right;
  __oe_levitation_board2_error_level_magnet_temperature_right = value;
}

static inline void canzero_set_levitation_board2_error_level_mcu_temperature(error_level value){
  extern error_level __oe_levitation_board2_error_level_mcu_temperature;
  __oe_levitation_board2_error_level_mcu_temperature = value;
}

static inline void canzero_set_levitation_board3_state(levitation_state value){
  extern levitation_state __oe_levitation_board3_state;
  __oe_levitation_board3_state = value;
}

static inline void canzero_set_levitation_board3_sdc_status(sdc_status value){
  extern sdc_status __oe_levitation_board3_sdc_status;
  __oe_levitation_board3_sdc_status = value;
}

static inline void canzero_set_levitation_board3_config_hash(uint64_t value){
  extern uint64_t __oe_levitation_board3_config_hash;
  __oe_levitation_board3_config_hash = value;
}

static inline void canzero_set_levitation_board3_assertion_fault(error_flag value){
  extern error_flag __oe_levitation_board3_assertion_fault;
  __oe_levitation_board3_assertion_fault = value;
}

static inline void canzero_set_levitation_board3_error_arming_failed(error_flag value){
  extern error_flag __oe_levitation_board3_error_arming_failed;
  __oe_levitation_board3_error_arming_failed = value;
}

static inline void canzero_set_levitation_board3_error_precharge_failed(error_flag value){
  extern error_flag __oe_levitation_board3_error_precharge_failed;
  __oe_levitation_board3_error_precharge_failed = value;
}

static inline void canzero_set_levitation_board3_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_levitation_board3_error_heartbeat_miss;
  __oe_levitation_board3_error_heartbeat_miss = value;
}

static inline void canzero_set_levitation_board3_error_level_vdc_voltage(error_level value){
  extern error_level __oe_levitation_board3_error_level_vdc_voltage;
  __oe_levitation_board3_error_level_vdc_voltage = value;
}

static inline void canzero_set_levitation_board3_error_level_magnet_current_left(error_level value){
  extern error_level __oe_levitation_board3_error_level_magnet_current_left;
  __oe_levitation_board3_error_level_magnet_current_left = value;
}

static inline void canzero_set_levitation_board3_error_level_magnet_current_right(error_level value){
  extern error_level __oe_levitation_board3_error_level_magnet_current_right;
  __oe_levitation_board3_error_level_magnet_current_right = value;
}

static inline void canzero_set_levitation_board3_error_level_input_current(error_level value){
  extern error_level __oe_levitation_board3_error_level_input_current;
  __oe_levitation_board3_error_level_input_current = value;
}

static inline void canzero_set_levitation_board3_error_level_magnet_temperature_left(error_level value){
  extern error_level __oe_levitation_board3_error_level_magnet_temperature_left;
  __oe_levitation_board3_error_level_magnet_temperature_left = value;
}

static inline void canzero_set_levitation_board3_error_level_magnet_temperature_right(error_level value){
  extern error_level __oe_levitation_board3_error_level_magnet_temperature_right;
  __oe_levitation_board3_error_level_magnet_temperature_right = value;
}

static inline void canzero_set_levitation_board3_error_level_mcu_temperature(error_level value){
  extern error_level __oe_levitation_board3_error_level_mcu_temperature;
  __oe_levitation_board3_error_level_mcu_temperature = value;
}

static inline void canzero_set_input_board_state(input_board_state value){
  extern input_board_state __oe_input_board_state;
  __oe_input_board_state = value;
}

void canzero_set_input_board_command(input_board_command value);

void canzero_set_input_board_assert_45V_online(bool_t value);

static inline void canzero_set_input_board_sdc_status(sdc_status value){
  extern sdc_status __oe_input_board_sdc_status;
  __oe_input_board_sdc_status = value;
}

static inline void canzero_set_input_board_config_hash(uint64_t value){
  extern uint64_t __oe_input_board_config_hash;
  __oe_input_board_config_hash = value;
}

static inline void canzero_set_input_board_assertion_fault(error_flag value){
  extern error_flag __oe_input_board_assertion_fault;
  __oe_input_board_assertion_fault = value;
}

static inline void canzero_set_input_board_error_acceleration_out_of_range(error_flag value){
  extern error_flag __oe_input_board_error_acceleration_out_of_range;
  __oe_input_board_error_acceleration_out_of_range = value;
}

static inline void canzero_set_input_board_error_lateral_acceleration_out_of_range(error_flag value){
  extern error_flag __oe_input_board_error_lateral_acceleration_out_of_range;
  __oe_input_board_error_lateral_acceleration_out_of_range = value;
}

static inline void canzero_set_input_board_error_vertical_acceleration_out_of_range(error_flag value){
  extern error_flag __oe_input_board_error_vertical_acceleration_out_of_range;
  __oe_input_board_error_vertical_acceleration_out_of_range = value;
}

static inline void canzero_set_input_board_error_acceleration_calibration_failed(error_flag value){
  extern error_flag __oe_input_board_error_acceleration_calibration_failed;
  __oe_input_board_error_acceleration_calibration_failed = value;
}

static inline void canzero_set_input_board_error_bat24_voltage_invalid(error_flag value){
  extern error_flag __oe_input_board_error_bat24_voltage_invalid;
  __oe_input_board_error_bat24_voltage_invalid = value;
}

static inline void canzero_set_input_board_error_bat24_current_invalid(error_flag value){
  extern error_flag __oe_input_board_error_bat24_current_invalid;
  __oe_input_board_error_bat24_current_invalid = value;
}

static inline void canzero_set_input_board_error_supercap_voltage_invalid(error_flag value){
  extern error_flag __oe_input_board_error_supercap_voltage_invalid;
  __oe_input_board_error_supercap_voltage_invalid = value;
}

static inline void canzero_set_input_board_error_link24_current_invalid(error_flag value){
  extern error_flag __oe_input_board_error_link24_current_invalid;
  __oe_input_board_error_link24_current_invalid = value;
}

static inline void canzero_set_input_board_error_link45_voltage_invalid(error_flag value){
  extern error_flag __oe_input_board_error_link45_voltage_invalid;
  __oe_input_board_error_link45_voltage_invalid = value;
}

static inline void canzero_set_input_board_error_link45_current_invalid(error_flag value){
  extern error_flag __oe_input_board_error_link45_current_invalid;
  __oe_input_board_error_link45_current_invalid = value;
}

static inline void canzero_set_input_board_error_cooling_cycle_pressure_invalid(error_flag value){
  extern error_flag __oe_input_board_error_cooling_cycle_pressure_invalid;
  __oe_input_board_error_cooling_cycle_pressure_invalid = value;
}

static inline void canzero_set_input_board_error_mcu_temperature_invalid(error_flag value){
  extern error_flag __oe_input_board_error_mcu_temperature_invalid;
  __oe_input_board_error_mcu_temperature_invalid = value;
}

static inline void canzero_set_input_board_error_cooling_cycle_temperature_invalid(error_flag value){
  extern error_flag __oe_input_board_error_cooling_cycle_temperature_invalid;
  __oe_input_board_error_cooling_cycle_temperature_invalid = value;
}

static inline void canzero_set_input_board_error_bat24_cell_temperature_1_invalid(error_flag value){
  extern error_flag __oe_input_board_error_bat24_cell_temperature_1_invalid;
  __oe_input_board_error_bat24_cell_temperature_1_invalid = value;
}

static inline void canzero_set_input_board_error_bat24_cell_temperature_2_invalid(error_flag value){
  extern error_flag __oe_input_board_error_bat24_cell_temperature_2_invalid;
  __oe_input_board_error_bat24_cell_temperature_2_invalid = value;
}

static inline void canzero_set_input_board_error_supercap_temperature_invalid(error_flag value){
  extern error_flag __oe_input_board_error_supercap_temperature_invalid;
  __oe_input_board_error_supercap_temperature_invalid = value;
}

static inline void canzero_set_input_board_error_buck_temperature_invalid(error_flag value){
  extern error_flag __oe_input_board_error_buck_temperature_invalid;
  __oe_input_board_error_buck_temperature_invalid = value;
}

static inline void canzero_set_input_board_error_sac_ebox_temperature_invalid(error_flag value){
  extern error_flag __oe_input_board_error_sac_ebox_temperature_invalid;
  __oe_input_board_error_sac_ebox_temperature_invalid = value;
}

static inline void canzero_set_input_board_error_power_ebox_temperature_invalid(error_flag value){
  extern error_flag __oe_input_board_error_power_ebox_temperature_invalid;
  __oe_input_board_error_power_ebox_temperature_invalid = value;
}

static inline void canzero_set_input_board_error_ambient_temperature_1_invalid(error_flag value){
  extern error_flag __oe_input_board_error_ambient_temperature_1_invalid;
  __oe_input_board_error_ambient_temperature_1_invalid = value;
}

static inline void canzero_set_input_board_error_ambient_temperature_2_invalid(error_flag value){
  extern error_flag __oe_input_board_error_ambient_temperature_2_invalid;
  __oe_input_board_error_ambient_temperature_2_invalid = value;
}

static inline void canzero_set_input_board_error_ambient_temperature_3_invalid(error_flag value){
  extern error_flag __oe_input_board_error_ambient_temperature_3_invalid;
  __oe_input_board_error_ambient_temperature_3_invalid = value;
}

static inline void canzero_set_input_board_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_input_board_error_heartbeat_miss;
  __oe_input_board_error_heartbeat_miss = value;
}

static inline void canzero_set_input_board_error_level_bat24_under_voltage(error_level value){
  extern error_level __oe_input_board_error_level_bat24_under_voltage;
  __oe_input_board_error_level_bat24_under_voltage = value;
}

static inline void canzero_set_input_board_error_level_bat24_over_voltage(error_level value){
  extern error_level __oe_input_board_error_level_bat24_over_voltage;
  __oe_input_board_error_level_bat24_over_voltage = value;
}

static inline void canzero_set_input_board_error_level_bat24_over_current(error_level value){
  extern error_level __oe_input_board_error_level_bat24_over_current;
  __oe_input_board_error_level_bat24_over_current = value;
}

static inline void canzero_set_input_board_error_level_supercap_under_voltage(error_level value){
  extern error_level __oe_input_board_error_level_supercap_under_voltage;
  __oe_input_board_error_level_supercap_under_voltage = value;
}

static inline void canzero_set_input_board_error_level_supercap_over_voltage(error_level value){
  extern error_level __oe_input_board_error_level_supercap_over_voltage;
  __oe_input_board_error_level_supercap_over_voltage = value;
}

static inline void canzero_set_input_board_error_level_link24_over_current(error_level value){
  extern error_level __oe_input_board_error_level_link24_over_current;
  __oe_input_board_error_level_link24_over_current = value;
}

static inline void canzero_set_input_board_error_level_link45_under_voltage(error_level value){
  extern error_level __oe_input_board_error_level_link45_under_voltage;
  __oe_input_board_error_level_link45_under_voltage = value;
}

static inline void canzero_set_input_board_error_level_link45_over_voltage(error_level value){
  extern error_level __oe_input_board_error_level_link45_over_voltage;
  __oe_input_board_error_level_link45_over_voltage = value;
}

static inline void canzero_set_input_board_error_level_link45_over_current(error_level value){
  extern error_level __oe_input_board_error_level_link45_over_current;
  __oe_input_board_error_level_link45_over_current = value;
}

static inline void canzero_set_input_board_error_level_cooling_cycle_over_pressure(error_level value){
  extern error_level __oe_input_board_error_level_cooling_cycle_over_pressure;
  __oe_input_board_error_level_cooling_cycle_over_pressure = value;
}

static inline void canzero_set_input_board_error_level_mcu_temperature(error_level value){
  extern error_level __oe_input_board_error_level_mcu_temperature;
  __oe_input_board_error_level_mcu_temperature = value;
}

static inline void canzero_set_input_board_error_level_cooling_cycle_temperature(error_level value){
  extern error_level __oe_input_board_error_level_cooling_cycle_temperature;
  __oe_input_board_error_level_cooling_cycle_temperature = value;
}

static inline void canzero_set_input_board_error_level_bat24_temperature(error_level value){
  extern error_level __oe_input_board_error_level_bat24_temperature;
  __oe_input_board_error_level_bat24_temperature = value;
}

static inline void canzero_set_input_board_error_level_supercap_temperature(error_level value){
  extern error_level __oe_input_board_error_level_supercap_temperature;
  __oe_input_board_error_level_supercap_temperature = value;
}

static inline void canzero_set_input_board_error_level_buck_temperature(error_level value){
  extern error_level __oe_input_board_error_level_buck_temperature;
  __oe_input_board_error_level_buck_temperature = value;
}

static inline void canzero_set_input_board_error_level_sac_ebox_temperature(error_level value){
  extern error_level __oe_input_board_error_level_sac_ebox_temperature;
  __oe_input_board_error_level_sac_ebox_temperature = value;
}

static inline void canzero_set_input_board_error_level_power_ebox_temperature(error_level value){
  extern error_level __oe_input_board_error_level_power_ebox_temperature;
  __oe_input_board_error_level_power_ebox_temperature = value;
}

static inline void canzero_set_input_board_error_level_ambient_temperature(error_level value){
  extern error_level __oe_input_board_error_level_ambient_temperature;
  __oe_input_board_error_level_ambient_temperature = value;
}

static inline void canzero_set_power_board12_state(pdu_12v_state value){
  extern pdu_12v_state __oe_power_board12_state;
  __oe_power_board12_state = value;
}

void canzero_set_power_board12_command(pdu_12v_command value);

static inline void canzero_set_power_board12_sdc_status(sdc_status value){
  extern sdc_status __oe_power_board12_sdc_status;
  __oe_power_board12_sdc_status = value;
}

static inline void canzero_set_power_board12_config_hash(uint64_t value){
  extern uint64_t __oe_power_board12_config_hash;
  __oe_power_board12_config_hash = value;
}

static inline void canzero_set_power_board12_assertion_fault(error_flag value){
  extern error_flag __oe_power_board12_assertion_fault;
  __oe_power_board12_assertion_fault = value;
}

static inline void canzero_set_power_board12_error_any_short(error_flag value){
  extern error_flag __oe_power_board12_error_any_short;
  __oe_power_board12_error_any_short = value;
}

static inline void canzero_set_power_board12_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_power_board12_error_heartbeat_miss;
  __oe_power_board12_error_heartbeat_miss = value;
}

static inline void canzero_set_power_board12_error_level_mcu_temperature(error_level value){
  extern error_level __oe_power_board12_error_level_mcu_temperature;
  __oe_power_board12_error_level_mcu_temperature = value;
}

static inline void canzero_set_power_board24_state(pdu_24v_state value){
  extern pdu_24v_state __oe_power_board24_state;
  __oe_power_board24_state = value;
}

void canzero_set_power_board24_command(pdu_24v_command value);

static inline void canzero_set_power_board24_sdc_status(sdc_status value){
  extern sdc_status __oe_power_board24_sdc_status;
  __oe_power_board24_sdc_status = value;
}

void canzero_set_power_board24_cooling_pump_channel_ctrl(bool_t value);

static inline void canzero_set_power_board24_config_hash(uint64_t value){
  extern uint64_t __oe_power_board24_config_hash;
  __oe_power_board24_config_hash = value;
}

static inline void canzero_set_power_board24_assertion_fault(error_flag value){
  extern error_flag __oe_power_board24_assertion_fault;
  __oe_power_board24_assertion_fault = value;
}

static inline void canzero_set_power_board24_error_any_short(error_flag value){
  extern error_flag __oe_power_board24_error_any_short;
  __oe_power_board24_error_any_short = value;
}

static inline void canzero_set_power_board24_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_power_board24_error_heartbeat_miss;
  __oe_power_board24_error_heartbeat_miss = value;
}

static inline void canzero_set_power_board24_error_level_mcu_temperature(error_level value){
  extern error_level __oe_power_board24_error_level_mcu_temperature;
  __oe_power_board24_error_level_mcu_temperature = value;
}

static inline void canzero_set_led_board_state(led_board_state value){
  extern led_board_state __oe_led_board_state;
  __oe_led_board_state = value;
}

void canzero_set_led_board_command(led_board_command value);

static inline void canzero_set_led_board_sdc_status(sdc_status value){
  extern sdc_status __oe_led_board_sdc_status;
  __oe_led_board_sdc_status = value;
}

static inline void canzero_set_led_board_config_hash(uint64_t value){
  extern uint64_t __oe_led_board_config_hash;
  __oe_led_board_config_hash = value;
}

static inline void canzero_set_led_board_assertion_fault(error_flag value){
  extern error_flag __oe_led_board_assertion_fault;
  __oe_led_board_assertion_fault = value;
}

static inline void canzero_set_led_board_error_heartbeat_miss(error_flag value){
  extern error_flag __oe_led_board_error_heartbeat_miss;
  __oe_led_board_error_heartbeat_miss = value;
}

static inline void canzero_set_led_board_error_level_mcu_temperature(error_level value){
  extern error_level __oe_led_board_error_level_mcu_temperature;
  __oe_led_board_error_level_mcu_temperature = value;
}

static inline void canzero_set_gamepad_max_acceleration(float value){
  extern float __oe_gamepad_max_acceleration;
  __oe_gamepad_max_acceleration = value;
}

static inline void canzero_set_gamepad_lt2(float value){
  extern float __oe_gamepad_lt2;
  __oe_gamepad_lt2 = value;
}

static inline void canzero_set_gamepad_rt2(float value){
  extern float __oe_gamepad_rt2;
  __oe_gamepad_rt2 = value;
}

void canzero_set_error_heartbeat_miss(error_flag value);

void canzero_set_last_node_missed(uint8_t value);

void canzero_set_target_airgap(float value);

void canzero_set_airgap_transition_duration(float value);

void canzero_set_airgap_transition_mode(airgap_transition_mode value);

void canzero_set_ignore_45v(bool_t value);

void canzero_send_config_hash();

void canzero_send_build_time();

void canzero_send_state();

void canzero_send_command();

void canzero_send_system_sdc_status();

void canzero_send_track_length();

void canzero_send_brake_margin();

void canzero_send_emergency_brake_margin();

void canzero_send_target_acceleration();

void canzero_send_pod_grounded();

void canzero_send_acceleration_target_velocity();

void canzero_send_velocity_pid();

void canzero_send_position();

void canzero_send_velocity();

void canzero_send_acceleration();

void canzero_send_error_level_config_consistency();

void canzero_send_error_any();

void canzero_send_error_level_over_temperature_system();

void canzero_send_motor_driver_state();

void canzero_send_motor_driver_command();

void canzero_send_motor_driver_sdc_status();

void canzero_send_motor_driver_config_hash();

void canzero_send_motor_driver_assertion_fault();

void canzero_send_motor_driver_error_arming_failed();

void canzero_send_motor_driver_error_precharge_failed();

void canzero_send_motor_driver_error_acceleration_out_of_range();

void canzero_send_motor_driver_error_acceleration_calibration_failed();

void canzero_send_motor_driver_error_invalid_target_acceleration();

void canzero_send_motor_driver_error_board_temperature1_invalid();

void canzero_send_motor_driver_error_board_temperature2_invalid();

void canzero_send_motor_driver_error_board_temperature3_invalid();

void canzero_send_motor_driver_error_mcu_temperature_invalid();

void canzero_send_motor_driver_error_lim_temperature1_invalid();

void canzero_send_motor_driver_error_lim_temperature2_invalid();

void canzero_send_motor_driver_error_lim_temperature3_invalid();

void canzero_send_motor_driver_error_lim_temperature4_invalid();

void canzero_send_motor_driver_error_heartbeat_miss();

void canzero_send_motor_driver_error_level_current_u1();

void canzero_send_motor_driver_error_level_current_v1();

void canzero_send_motor_driver_error_level_current_w1();

void canzero_send_motor_driver_error_level_current_u2();

void canzero_send_motor_driver_error_level_current_v2();

void canzero_send_motor_driver_error_level_current_w2();

void canzero_send_motor_driver_error_level_vdc_voltage();

void canzero_send_motor_driver_error_level_board_temperature();

void canzero_send_motor_driver_error_level_mcu_temperature();

void canzero_send_motor_driver_error_level_lim_temperature();

void canzero_send_guidance_command();

void canzero_send_guidance_board_front_state();

void canzero_send_guidance_board_front_sdc_status();

void canzero_send_guidance_board_front_config_hash();

void canzero_send_guidance_board_front_assertion_fault();

void canzero_send_guidance_board_front_error_arming_failed();

void canzero_send_guidance_board_front_error_precharge_failed();

void canzero_send_guidance_board_front_error_heartbeat_miss();

void canzero_send_guidance_board_front_error_outer_airgap_left_invalid();

void canzero_send_guidance_board_front_error_inner_airgap_left_invalid();

void canzero_send_guidance_board_front_error_outer_airgap_right_invalid();

void canzero_send_guidance_board_front_error_inner_airgap_right_invalid();

void canzero_send_guidance_board_front_error_vdc_voltage_invalid();

void canzero_send_guidance_board_front_error_magnet_current_left_invalid();

void canzero_send_guidance_board_front_error_magnet_current_right_invalid();

void canzero_send_guidance_board_front_error_input_current_invalid();

void canzero_send_guidance_board_front_error_magnet_temperature_left_invalid();

void canzero_send_guidance_board_front_error_magnet_temperature_right_invalid();

void canzero_send_guidance_board_front_error_mcu_temperature_invalid();

void canzero_send_guidance_board_front_error_level_vdc_voltage();

void canzero_send_guidance_board_front_error_level_magnet_current_left();

void canzero_send_guidance_board_front_error_level_magnet_current_right();

void canzero_send_guidance_board_front_error_level_input_current();

void canzero_send_guidance_board_front_error_level_magnet_temperature_left();

void canzero_send_guidance_board_front_error_level_magnet_temperature_right();

void canzero_send_guidance_board_front_error_level_mcu_temperature();

void canzero_send_guidance_board_back_state();

void canzero_send_guidance_board_back_sdc_status();

void canzero_send_guidance_board_back_config_hash();

void canzero_send_guidance_board_back_assertion_fault();

void canzero_send_guidance_board_back_error_arming_failed();

void canzero_send_guidance_board_back_error_precharge_failed();

void canzero_send_guidance_board_back_error_heartbeat_miss();

void canzero_send_guidance_board_back_error_outer_airgap_left_invalid();

void canzero_send_guidance_board_back_error_inner_airgap_left_invalid();

void canzero_send_guidance_board_back_error_outer_airgap_right_invalid();

void canzero_send_guidance_board_back_error_inner_airgap_right_invalid();

void canzero_send_guidance_board_back_error_vdc_voltage_invalid();

void canzero_send_guidance_board_back_error_magnet_current_left_invalid();

void canzero_send_guidance_board_back_error_magnet_current_right_invalid();

void canzero_send_guidance_board_back_error_input_current_invalid();

void canzero_send_guidance_board_back_error_magnet_temperature_left_invalid();

void canzero_send_guidance_board_back_error_magnet_temperature_right_invalid();

void canzero_send_guidance_board_back_error_mcu_temperature_invalid();

void canzero_send_guidance_board_back_error_level_vdc_voltage();

void canzero_send_guidance_board_back_error_level_magnet_current_left();

void canzero_send_guidance_board_back_error_level_magnet_current_right();

void canzero_send_guidance_board_back_error_level_input_current();

void canzero_send_guidance_board_back_error_level_magnet_temperature_left();

void canzero_send_guidance_board_back_error_level_magnet_temperature_right();

void canzero_send_guidance_board_back_error_level_mcu_temperature();

void canzero_send_levitation_command();

void canzero_send_levitation_board1_state();

void canzero_send_levitation_board1_sdc_status();

void canzero_send_levitation_board1_config_hash();

void canzero_send_levitation_board1_assertion_fault();

void canzero_send_levitation_board1_error_arming_failed();

void canzero_send_levitation_board1_error_precharge_failed();

void canzero_send_levitation_board1_error_heartbeat_miss();

void canzero_send_levitation_board1_error_level_vdc_voltage();

void canzero_send_levitation_board1_error_level_magnet_current_left();

void canzero_send_levitation_board1_error_level_magnet_current_right();

void canzero_send_levitation_board1_error_level_input_current();

void canzero_send_levitation_board1_error_level_magnet_temperature_left();

void canzero_send_levitation_board1_error_level_magnet_temperature_right();

void canzero_send_levitation_board1_error_level_mcu_temperature();

void canzero_send_levitation_board2_state();

void canzero_send_levitation_board2_sdc_status();

void canzero_send_levitation_board2_config_hash();

void canzero_send_levitation_board2_assertion_fault();

void canzero_send_levitation_board2_error_arming_failed();

void canzero_send_levitation_board2_error_precharge_failed();

void canzero_send_levitation_board2_error_heartbeat_miss();

void canzero_send_levitation_board2_error_level_vdc_voltage();

void canzero_send_levitation_board2_error_level_magnet_current_left();

void canzero_send_levitation_board2_error_level_magnet_current_right();

void canzero_send_levitation_board2_error_level_input_current();

void canzero_send_levitation_board2_error_level_magnet_temperature_left();

void canzero_send_levitation_board2_error_level_magnet_temperature_right();

void canzero_send_levitation_board2_error_level_mcu_temperature();

void canzero_send_levitation_board3_state();

void canzero_send_levitation_board3_sdc_status();

void canzero_send_levitation_board3_config_hash();

void canzero_send_levitation_board3_assertion_fault();

void canzero_send_levitation_board3_error_arming_failed();

void canzero_send_levitation_board3_error_precharge_failed();

void canzero_send_levitation_board3_error_heartbeat_miss();

void canzero_send_levitation_board3_error_level_vdc_voltage();

void canzero_send_levitation_board3_error_level_magnet_current_left();

void canzero_send_levitation_board3_error_level_magnet_current_right();

void canzero_send_levitation_board3_error_level_input_current();

void canzero_send_levitation_board3_error_level_magnet_temperature_left();

void canzero_send_levitation_board3_error_level_magnet_temperature_right();

void canzero_send_levitation_board3_error_level_mcu_temperature();

void canzero_send_input_board_state();

void canzero_send_input_board_command();

void canzero_send_input_board_assert_45V_online();

void canzero_send_input_board_sdc_status();

void canzero_send_input_board_config_hash();

void canzero_send_input_board_assertion_fault();

void canzero_send_input_board_error_acceleration_out_of_range();

void canzero_send_input_board_error_lateral_acceleration_out_of_range();

void canzero_send_input_board_error_vertical_acceleration_out_of_range();

void canzero_send_input_board_error_acceleration_calibration_failed();

void canzero_send_input_board_error_bat24_voltage_invalid();

void canzero_send_input_board_error_bat24_current_invalid();

void canzero_send_input_board_error_supercap_voltage_invalid();

void canzero_send_input_board_error_link24_current_invalid();

void canzero_send_input_board_error_link45_voltage_invalid();

void canzero_send_input_board_error_link45_current_invalid();

void canzero_send_input_board_error_cooling_cycle_pressure_invalid();

void canzero_send_input_board_error_mcu_temperature_invalid();

void canzero_send_input_board_error_cooling_cycle_temperature_invalid();

void canzero_send_input_board_error_bat24_cell_temperature_1_invalid();

void canzero_send_input_board_error_bat24_cell_temperature_2_invalid();

void canzero_send_input_board_error_supercap_temperature_invalid();

void canzero_send_input_board_error_buck_temperature_invalid();

void canzero_send_input_board_error_sac_ebox_temperature_invalid();

void canzero_send_input_board_error_power_ebox_temperature_invalid();

void canzero_send_input_board_error_ambient_temperature_1_invalid();

void canzero_send_input_board_error_ambient_temperature_2_invalid();

void canzero_send_input_board_error_ambient_temperature_3_invalid();

void canzero_send_input_board_error_heartbeat_miss();

void canzero_send_input_board_error_level_bat24_under_voltage();

void canzero_send_input_board_error_level_bat24_over_voltage();

void canzero_send_input_board_error_level_bat24_over_current();

void canzero_send_input_board_error_level_supercap_under_voltage();

void canzero_send_input_board_error_level_supercap_over_voltage();

void canzero_send_input_board_error_level_link24_over_current();

void canzero_send_input_board_error_level_link45_under_voltage();

void canzero_send_input_board_error_level_link45_over_voltage();

void canzero_send_input_board_error_level_link45_over_current();

void canzero_send_input_board_error_level_cooling_cycle_over_pressure();

void canzero_send_input_board_error_level_mcu_temperature();

void canzero_send_input_board_error_level_cooling_cycle_temperature();

void canzero_send_input_board_error_level_bat24_temperature();

void canzero_send_input_board_error_level_supercap_temperature();

void canzero_send_input_board_error_level_buck_temperature();

void canzero_send_input_board_error_level_sac_ebox_temperature();

void canzero_send_input_board_error_level_power_ebox_temperature();

void canzero_send_input_board_error_level_ambient_temperature();

void canzero_send_power_board12_state();

void canzero_send_power_board12_command();

void canzero_send_power_board12_sdc_status();

void canzero_send_power_board12_config_hash();

void canzero_send_power_board12_assertion_fault();

void canzero_send_power_board12_error_any_short();

void canzero_send_power_board12_error_heartbeat_miss();

void canzero_send_power_board12_error_level_mcu_temperature();

void canzero_send_power_board24_state();

void canzero_send_power_board24_command();

void canzero_send_power_board24_sdc_status();

void canzero_send_power_board24_cooling_pump_channel_ctrl();

void canzero_send_power_board24_config_hash();

void canzero_send_power_board24_assertion_fault();

void canzero_send_power_board24_error_any_short();

void canzero_send_power_board24_error_heartbeat_miss();

void canzero_send_power_board24_error_level_mcu_temperature();

void canzero_send_led_board_state();

void canzero_send_led_board_command();

void canzero_send_led_board_sdc_status();

void canzero_send_led_board_config_hash();

void canzero_send_led_board_assertion_fault();

void canzero_send_led_board_error_heartbeat_miss();

void canzero_send_led_board_error_level_mcu_temperature();

void canzero_send_gamepad_max_acceleration();

void canzero_send_gamepad_lt2();

void canzero_send_gamepad_rt2();

void canzero_send_error_heartbeat_miss();

void canzero_send_last_node_missed();

void canzero_send_target_airgap();

void canzero_send_airgap_transition_duration();

void canzero_send_airgap_transition_mode();

void canzero_send_ignore_45v();

#endif