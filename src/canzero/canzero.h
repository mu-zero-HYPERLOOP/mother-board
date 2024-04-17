#ifndef CANZERO_H
#define CANZERO_H
#include "inttypes.h"
#include "stddef.h"
typedef struct {
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
} get_req_header;
typedef struct {
  uint8_t sof;
  uint8_t eof;
  uint8_t toggle;
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
} set_req_header;
typedef enum {
  input_board_state_INIT = 0,
  input_board_state_RUNNING = 1,
} input_board_state;
typedef enum {
  error_flag_OK = 0,
  error_flag_ERROR = 1,
} error_flag;
typedef enum {
  sdc_status_OPEN = 0,
  sdc_status_CLOSED = 1,
} sdc_status;
typedef enum {
  pdu_state_INIT = 0,
  pdu_state_RUNNING = 1,
} pdu_state;
typedef enum {
  pdu_channel_status_OFF = 0,
  pdu_channel_status_ON = 1,
  pdu_channel_status_SHORT_CIRCUIT = 3,
} pdu_channel_status;
typedef enum {
  motor_state_INIT = 0,
  motor_state_IDLE = 1,
  motor_state_PRECHARGE = 2,
  motor_state_READY = 3,
  motor_state_START = 4,
  motor_state_CONTROL = 5,
  motor_state_STOP = 6,
  motor_state_MANUAL = 7,
  motor_state_ERROR = 8,
} motor_state;
typedef enum {
  mgu_state_INIT = 0,
  mgu_state_IDLE = 1,
  mgu_state_PRECHARGE = 2,
  mgu_state_READY = 3,
  mgu_state_START = 4,
  mgu_state_CONTROL = 5,
  mgu_state_STOP = 6,
} mgu_state;
typedef enum {
  mgu_command_NONE = 0,
  mgu_command_PRECHARGE = 1,
  mgu_command_START = 2,
  mgu_command_STOP = 3,
  mgu_command_DISCONNECT = 4,
} mgu_command;
typedef enum {
  mlu_state_INIT = 0,
  mlu_state_IDLE = 1,
  mlu_state_PRECHARGE = 2,
  mlu_state_READY = 3,
  mlu_state_START = 4,
  mlu_state_CONTROL = 5,
  mlu_state_STOP = 6,
} mlu_state;
typedef enum {
  mlu_command_NONE = 0,
  mlu_command_PRECHARGE = 1,
  mlu_command_START = 2,
  mlu_command_STOP = 3,
  mlu_command_ABORT = 4,
  mlu_command_DISCONNECT = 5,
} mlu_command;
typedef enum {
  node_id_mlu1 = 0,
  node_id_master = 1,
  node_id_mlu2 = 2,
  node_id_mlu3 = 3,
  node_id_mlu4 = 4,
  node_id_mlu5 = 5,
  node_id_mlu6 = 6,
  node_id_mgu1 = 7,
  node_id_mgu2 = 8,
  node_id_motor_driver = 9,
  node_id_pdu24 = 10,
  node_id_pdu12 = 11,
  node_id_input_board = 12,
  node_id_gamepad = 13,
} node_id;
typedef struct {
  uint8_t sof;
  uint8_t eof;
  uint8_t toggle;
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
} get_resp_header;
typedef enum {
  set_resp_erno_Success = 0,
  set_resp_erno_Error = 1,
} set_resp_erno;
typedef struct {
  uint16_t od_index;
  uint8_t client_id;
  uint8_t server_id;
  set_resp_erno erno;
} set_resp_header;
typedef enum {
  motor_command_NONE = 0,
  motor_command_PRECHARGE = 1,
  motor_command_ACCELERATE = 2,
  motor_command_DECELERATE = 3,
  motor_command_STOP = 4,
  motor_command_DISCONNECT = 5,
  motor_command_MANUAL_CONTROL = 6,
} motor_command;
typedef enum {
  pdu_channel_control_OFF = 0,
  pdu_channel_control_ON = 1,
} pdu_channel_control;
typedef enum {
  global_state_INIT = 0,
  global_state_IDLE = 1,
  global_state_DISCONNECTING = 2,
  global_state_PRECHARGE = 3,
  global_state_READY = 4,
  global_state_START_LEVITATION = 5,
  global_state_LEVITATION_STABLE = 6,
  global_state_START_GUIDANCE = 7,
  global_state_GUIDANCE_STABLE = 8,
  global_state_ACCELERATION = 9,
  global_state_CRUISING = 10,
  global_state_DECELERATION = 11,
  global_state_STOP_LEVITATION = 12,
} global_state;
typedef enum {
  global_command_NONE = 0,
  global_command_PRECHARGE = 1,
  global_command_DISCONNECT = 2,
  global_command_START_LEVITATION = 3,
  global_command_START_PROPULSION = 4,
  global_command_STOP_PROPULSION = 5,
  global_command_STOP_LEVITATION = 6,
  global_command_ABORT = 7,
  global_command_MANUAL_CONTROL = 8,
  global_command_EMERGENCY = 9,
} global_command;
typedef struct {
  float p_value;
  float i_value;
  float d_value;
} mlu_pid_values;
typedef enum {
  bool_t_FALSE = 0,
  bool_t_TRUE = 1,
} bool_t;
typedef struct {
  float p_value;
  float i_value;
  float d_value;
} mgu_pid_values;
typedef struct {
  uint32_t id;
  uint8_t dlc;
  uint8_t data[8];
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
static inline mlu_command canzero_get_mlu_command() {
  extern mlu_command __oe_mlu_command;
  return __oe_mlu_command;
}
static inline mgu_command canzero_get_mgu_command() {
  extern mgu_command __oe_mgu_command;
  return __oe_mgu_command;
}
static inline motor_command canzero_get_motor_command() {
  extern motor_command __oe_motor_command;
  return __oe_motor_command;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel1_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel1_control;
  return __oe_pdu24_lp_channel1_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel2_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel2_control;
  return __oe_pdu24_lp_channel2_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel3_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel3_control;
  return __oe_pdu24_lp_channel3_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel4_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel4_control;
  return __oe_pdu24_lp_channel4_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel5_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel5_control;
  return __oe_pdu24_lp_channel5_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel6_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel6_control;
  return __oe_pdu24_lp_channel6_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel7_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel7_control;
  return __oe_pdu24_lp_channel7_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel8_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel8_control;
  return __oe_pdu24_lp_channel8_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel9_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel9_control;
  return __oe_pdu24_lp_channel9_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel10_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel10_control;
  return __oe_pdu24_lp_channel10_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel11_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel11_control;
  return __oe_pdu24_lp_channel11_control;
}
static inline pdu_channel_control canzero_get_pdu24_lp_channel12_control() {
  extern pdu_channel_control __oe_pdu24_lp_channel12_control;
  return __oe_pdu24_lp_channel12_control;
}
static inline pdu_channel_control canzero_get_pdu24_hp_channel1_control() {
  extern pdu_channel_control __oe_pdu24_hp_channel1_control;
  return __oe_pdu24_hp_channel1_control;
}
static inline pdu_channel_control canzero_get_pdu24_hp_channel2_control() {
  extern pdu_channel_control __oe_pdu24_hp_channel2_control;
  return __oe_pdu24_hp_channel2_control;
}
static inline pdu_channel_control canzero_get_pdu24_hp_channel3_control() {
  extern pdu_channel_control __oe_pdu24_hp_channel3_control;
  return __oe_pdu24_hp_channel3_control;
}
static inline pdu_channel_control canzero_get_pdu24_hp_channel4_control() {
  extern pdu_channel_control __oe_pdu24_hp_channel4_control;
  return __oe_pdu24_hp_channel4_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel1_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel1_control;
  return __oe_pdu12_lp_channel1_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel2_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel2_control;
  return __oe_pdu12_lp_channel2_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel3_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel3_control;
  return __oe_pdu12_lp_channel3_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel4_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel4_control;
  return __oe_pdu12_lp_channel4_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel5_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel5_control;
  return __oe_pdu12_lp_channel5_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel6_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel6_control;
  return __oe_pdu12_lp_channel6_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel7_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel7_control;
  return __oe_pdu12_lp_channel7_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel8_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel8_control;
  return __oe_pdu12_lp_channel8_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel9_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel9_control;
  return __oe_pdu12_lp_channel9_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel10_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel10_control;
  return __oe_pdu12_lp_channel10_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel11_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel11_control;
  return __oe_pdu12_lp_channel11_control;
}
static inline pdu_channel_control canzero_get_pdu12_lp_channel12_control() {
  extern pdu_channel_control __oe_pdu12_lp_channel12_control;
  return __oe_pdu12_lp_channel12_control;
}
static inline pdu_channel_control canzero_get_pdu12_hp_channel1_control() {
  extern pdu_channel_control __oe_pdu12_hp_channel1_control;
  return __oe_pdu12_hp_channel1_control;
}
static inline pdu_channel_control canzero_get_pdu12_hp_channel2_control() {
  extern pdu_channel_control __oe_pdu12_hp_channel2_control;
  return __oe_pdu12_hp_channel2_control;
}
static inline pdu_channel_control canzero_get_pdu12_hp_channel3_control() {
  extern pdu_channel_control __oe_pdu12_hp_channel3_control;
  return __oe_pdu12_hp_channel3_control;
}
static inline pdu_channel_control canzero_get_pdu12_hp_channel4_control() {
  extern pdu_channel_control __oe_pdu12_hp_channel4_control;
  return __oe_pdu12_hp_channel4_control;
}
static inline global_state canzero_get_global_state() {
  extern global_state __oe_global_state;
  return __oe_global_state;
}
static inline global_command canzero_get_command() {
  extern global_command __oe_command;
  return __oe_command;
}
static inline bool_t canzero_get_sync_mlu_pid_values() {
  extern bool_t __oe_sync_mlu_pid_values;
  return __oe_sync_mlu_pid_values;
}
static inline mlu_pid_values canzero_get_mlu_pid_values() {
  extern mlu_pid_values __oe_mlu_pid_values;
  return __oe_mlu_pid_values;
}
static inline bool_t canzero_get_sync_mgu_pid_values() {
  extern bool_t __oe_sync_mgu_pid_values;
  return __oe_sync_mgu_pid_values;
}
static inline mgu_pid_values canzero_get_mgu_pid_values() {
  extern mgu_pid_values __oe_mgu_pid_values;
  return __oe_mgu_pid_values;
}
static inline float canzero_get_cpu_temperature() {
  extern float __oe_cpu_temperature;
  return __oe_cpu_temperature;
}
static inline sdc_status canzero_get_sdc_status() {
  extern sdc_status __oe_sdc_status;
  return __oe_sdc_status;
}
static inline float canzero_get_power_estimation() {
  extern float __oe_power_estimation;
  return __oe_power_estimation;
}
static inline float canzero_get_power_estimation24() {
  extern float __oe_power_estimation24;
  return __oe_power_estimation24;
}
static inline float canzero_get_power_estimation48() {
  extern float __oe_power_estimation48;
  return __oe_power_estimation48;
}
static inline pdu_state canzero_get_pdu12_state() {
  extern pdu_state __oe_pdu12_state;
  return __oe_pdu12_state;
}
static inline pdu_state canzero_get_pdu24_state() {
  extern pdu_state __oe_pdu24_state;
  return __oe_pdu24_state;
}
static inline input_board_state canzero_get_input_board_state() {
  extern input_board_state __oe_input_board_state;
  return __oe_input_board_state;
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
static inline error_flag canzero_get_error_input_board_mcu_over_temperature() {
  extern error_flag __oe_error_input_board_mcu_over_temperature;
  return __oe_error_input_board_mcu_over_temperature;
}
static inline error_flag canzero_get_error_invalid_position_estimation() {
  extern error_flag __oe_error_invalid_position_estimation;
  return __oe_error_invalid_position_estimation;
}
static inline error_flag canzero_get_error_invalid_position() {
  extern error_flag __oe_error_invalid_position;
  return __oe_error_invalid_position;
}
static inline error_flag canzero_get_error_invalid_velocity_profile() {
  extern error_flag __oe_error_invalid_velocity_profile;
  return __oe_error_invalid_velocity_profile;
}
static inline error_flag canzero_get_error_invalid_acceleration_profile() {
  extern error_flag __oe_error_invalid_acceleration_profile;
  return __oe_error_invalid_acceleration_profile;
}
static inline error_flag canzero_get_error_battery_over_voltage() {
  extern error_flag __oe_error_battery_over_voltage;
  return __oe_error_battery_over_voltage;
}
static inline error_flag canzero_get_error_cooling_cycle_over_pressure() {
  extern error_flag __oe_error_cooling_cycle_over_pressure;
  return __oe_error_cooling_cycle_over_pressure;
}
static inline error_flag canzero_get_error_cooling_cycle_low_pressure() {
  extern error_flag __oe_error_cooling_cycle_low_pressure;
  return __oe_error_cooling_cycle_low_pressure;
}
static inline error_flag canzero_get_error_ebox_over_temperature() {
  extern error_flag __oe_error_ebox_over_temperature;
  return __oe_error_ebox_over_temperature;
}
static inline error_flag canzero_get_error_cooling_cycle_over_temperature() {
  extern error_flag __oe_error_cooling_cycle_over_temperature;
  return __oe_error_cooling_cycle_over_temperature;
}
static inline error_flag canzero_get_warn_invalid_position_estimation() {
  extern error_flag __oe_warn_invalid_position_estimation;
  return __oe_warn_invalid_position_estimation;
}
static inline error_flag canzero_get_warn_invalid_position() {
  extern error_flag __oe_warn_invalid_position;
  return __oe_warn_invalid_position;
}
static inline error_flag canzero_get_warn_invalid_velocity_profile() {
  extern error_flag __oe_warn_invalid_velocity_profile;
  return __oe_warn_invalid_velocity_profile;
}
static inline error_flag canzero_get_warn_invalid_acceleration_profile() {
  extern error_flag __oe_warn_invalid_acceleration_profile;
  return __oe_warn_invalid_acceleration_profile;
}
static inline error_flag canzero_get_warn_battery_over_temperature() {
  extern error_flag __oe_warn_battery_over_temperature;
  return __oe_warn_battery_over_temperature;
}
static inline error_flag canzero_get_warn_cooling_cycle_over_pressure() {
  extern error_flag __oe_warn_cooling_cycle_over_pressure;
  return __oe_warn_cooling_cycle_over_pressure;
}
static inline error_flag canzero_get_warn_cooling_cycle_low_pressure() {
  extern error_flag __oe_warn_cooling_cycle_low_pressure;
  return __oe_warn_cooling_cycle_low_pressure;
}
static inline error_flag canzero_get_warn_input_board_mcu_over_temperature() {
  extern error_flag __oe_warn_input_board_mcu_over_temperature;
  return __oe_warn_input_board_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_ebox_over_temperature() {
  extern error_flag __oe_warn_ebox_over_temperature;
  return __oe_warn_ebox_over_temperature;
}
static inline error_flag canzero_get_warn_cooling_cycle_over_temperature() {
  extern error_flag __oe_warn_cooling_cycle_over_temperature;
  return __oe_warn_cooling_cycle_over_temperature;
}
static inline sdc_status canzero_get_input_board_sdc_status() {
  extern sdc_status __oe_input_board_sdc_status;
  return __oe_input_board_sdc_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel1_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel1_status;
  return __oe_pdu12_lp_channel1_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel2_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel2_status;
  return __oe_pdu12_lp_channel2_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel3_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel3_status;
  return __oe_pdu12_lp_channel3_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel4_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel4_status;
  return __oe_pdu12_lp_channel4_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel5_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel5_status;
  return __oe_pdu12_lp_channel5_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel6_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel6_status;
  return __oe_pdu12_lp_channel6_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel7_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel7_status;
  return __oe_pdu12_lp_channel7_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel8_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel8_status;
  return __oe_pdu12_lp_channel8_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_chanel9_status() {
  extern pdu_channel_status __oe_pdu12_lp_chanel9_status;
  return __oe_pdu12_lp_chanel9_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel10_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel10_status;
  return __oe_pdu12_lp_channel10_status;
}
static inline pdu_channel_status canzero_get_pdu12_lp_channel11_status() {
  extern pdu_channel_status __oe_pdu12_lp_channel11_status;
  return __oe_pdu12_lp_channel11_status;
}
static inline uint8_t canzero_get_pdu12_lp_channel12_status() {
  extern uint8_t __oe_pdu12_lp_channel12_status;
  return __oe_pdu12_lp_channel12_status;
}
static inline pdu_channel_status canzero_get_pdu12_hp_channel1_status() {
  extern pdu_channel_status __oe_pdu12_hp_channel1_status;
  return __oe_pdu12_hp_channel1_status;
}
static inline pdu_channel_status canzero_get_pdu12_hp_channel2_status() {
  extern pdu_channel_status __oe_pdu12_hp_channel2_status;
  return __oe_pdu12_hp_channel2_status;
}
static inline pdu_channel_status canzero_get_pdu12_hp_channel3_status() {
  extern pdu_channel_status __oe_pdu12_hp_channel3_status;
  return __oe_pdu12_hp_channel3_status;
}
static inline pdu_channel_status canzero_get_pdu12_hp_channel4_status() {
  extern pdu_channel_status __oe_pdu12_hp_channel4_status;
  return __oe_pdu12_hp_channel4_status;
}
static inline sdc_status canzero_get_pdu12_sdc_status() {
  extern sdc_status __oe_pdu12_sdc_status;
  return __oe_pdu12_sdc_status;
}
static inline error_flag canzero_get_error_pdu12_mcu_over_temperature() {
  extern error_flag __oe_error_pdu12_mcu_over_temperature;
  return __oe_error_pdu12_mcu_over_temperature;
}
static inline float canzero_get_pdu12_power_estimation() {
  extern float __oe_pdu12_power_estimation;
  return __oe_pdu12_power_estimation;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel1_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel1_status;
  return __oe_pdu24_lp_channel1_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel2_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel2_status;
  return __oe_pdu24_lp_channel2_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel3_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel3_status;
  return __oe_pdu24_lp_channel3_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel4_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel4_status;
  return __oe_pdu24_lp_channel4_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel5_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel5_status;
  return __oe_pdu24_lp_channel5_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel6_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel6_status;
  return __oe_pdu24_lp_channel6_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel7_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel7_status;
  return __oe_pdu24_lp_channel7_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel8_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel8_status;
  return __oe_pdu24_lp_channel8_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_chanel9_status() {
  extern pdu_channel_status __oe_pdu24_lp_chanel9_status;
  return __oe_pdu24_lp_chanel9_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel10_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel10_status;
  return __oe_pdu24_lp_channel10_status;
}
static inline pdu_channel_status canzero_get_pdu24_lp_channel11_status() {
  extern pdu_channel_status __oe_pdu24_lp_channel11_status;
  return __oe_pdu24_lp_channel11_status;
}
static inline uint8_t canzero_get_pdu24_lp_channel12_status() {
  extern uint8_t __oe_pdu24_lp_channel12_status;
  return __oe_pdu24_lp_channel12_status;
}
static inline pdu_channel_status canzero_get_pdu24_hp_channel1_status() {
  extern pdu_channel_status __oe_pdu24_hp_channel1_status;
  return __oe_pdu24_hp_channel1_status;
}
static inline pdu_channel_status canzero_get_pdu24_hp_channel2_status() {
  extern pdu_channel_status __oe_pdu24_hp_channel2_status;
  return __oe_pdu24_hp_channel2_status;
}
static inline pdu_channel_status canzero_get_pdu24_hp_channel3_status() {
  extern pdu_channel_status __oe_pdu24_hp_channel3_status;
  return __oe_pdu24_hp_channel3_status;
}
static inline pdu_channel_status canzero_get_pdu24_hp_channel4_status() {
  extern pdu_channel_status __oe_pdu24_hp_channel4_status;
  return __oe_pdu24_hp_channel4_status;
}
static inline sdc_status canzero_get_pdu24_sdc_status() {
  extern sdc_status __oe_pdu24_sdc_status;
  return __oe_pdu24_sdc_status;
}
static inline error_flag canzero_get_error_pdu24_mcu_over_temperature() {
  extern error_flag __oe_error_pdu24_mcu_over_temperature;
  return __oe_error_pdu24_mcu_over_temperature;
}
static inline float canzero_get_pdu24_power_estimation() {
  extern float __oe_pdu24_power_estimation;
  return __oe_pdu24_power_estimation;
}
static inline error_flag canzero_get_error_motor_driver_45V_over_voltage() {
  extern error_flag __oe_error_motor_driver_45V_over_voltage;
  return __oe_error_motor_driver_45V_over_voltage;
}
static inline error_flag canzero_get_error_motor_driver_45V_under_voltage() {
  extern error_flag __oe_error_motor_driver_45V_under_voltage;
  return __oe_error_motor_driver_45V_under_voltage;
}
static inline error_flag canzero_get_error_dslim_over_temperature() {
  extern error_flag __oe_error_dslim_over_temperature;
  return __oe_error_dslim_over_temperature;
}
static inline error_flag canzero_get_error_motor_driver_mosfet_over_temperature() {
  extern error_flag __oe_error_motor_driver_mosfet_over_temperature;
  return __oe_error_motor_driver_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_motor_control_failure() {
  extern error_flag __oe_error_motor_control_failure;
  return __oe_error_motor_control_failure;
}
static inline error_flag canzero_get_error_motor_mcu_over_temperature() {
  extern error_flag __oe_error_motor_mcu_over_temperature;
  return __oe_error_motor_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_motor_dslim_over_temperature() {
  extern error_flag __oe_warn_motor_dslim_over_temperature;
  return __oe_warn_motor_dslim_over_temperature;
}
static inline error_flag canzero_get_warn_motor_mosfet_over_temperature() {
  extern error_flag __oe_warn_motor_mosfet_over_temperature;
  return __oe_warn_motor_mosfet_over_temperature;
}
static inline error_flag canzero_get_warn_motor_mcu_over_temperature() {
  extern error_flag __oe_warn_motor_mcu_over_temperature;
  return __oe_warn_motor_mcu_over_temperature;
}
static inline float canzero_get_motor_power_estimation() {
  extern float __oe_motor_power_estimation;
  return __oe_motor_power_estimation;
}
static inline motor_state canzero_get_motor_state() {
  extern motor_state __oe_motor_state;
  return __oe_motor_state;
}
static inline sdc_status canzero_get_motor_sdc_status() {
  extern sdc_status __oe_motor_sdc_status;
  return __oe_motor_sdc_status;
}
static inline error_flag canzero_get_error_mgu1_45V_over_voltage() {
  extern error_flag __oe_error_mgu1_45V_over_voltage;
  return __oe_error_mgu1_45V_over_voltage;
}
static inline error_flag canzero_get_error_mgu1_45V_under_voltage() {
  extern error_flag __oe_error_mgu1_45V_under_voltage;
  return __oe_error_mgu1_45V_under_voltage;
}
static inline error_flag canzero_get_error_mgu1_control_error() {
  extern error_flag __oe_error_mgu1_control_error;
  return __oe_error_mgu1_control_error;
}
static inline error_flag canzero_get_error_mgu1_magnet_over_temperature_starboard() {
  extern error_flag __oe_error_mgu1_magnet_over_temperature_starboard;
  return __oe_error_mgu1_magnet_over_temperature_starboard;
}
static inline error_flag canzero_get_error_mgu1_magnet_over_temperature_port() {
  extern error_flag __oe_error_mgu1_magnet_over_temperature_port;
  return __oe_error_mgu1_magnet_over_temperature_port;
}
static inline error_flag canzero_get_error_mgu1_mosfet_over_temperature() {
  extern error_flag __oe_error_mgu1_mosfet_over_temperature;
  return __oe_error_mgu1_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_mgu1_mcu_over_temperature() {
  extern error_flag __oe_error_mgu1_mcu_over_temperature;
  return __oe_error_mgu1_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_mgu1_magnet_over_temperature_starboard() {
  extern error_flag __oe_warn_mgu1_magnet_over_temperature_starboard;
  return __oe_warn_mgu1_magnet_over_temperature_starboard;
}
static inline error_flag canzero_get_warn_mgu1_magnet_over_temperature_port() {
  extern error_flag __oe_warn_mgu1_magnet_over_temperature_port;
  return __oe_warn_mgu1_magnet_over_temperature_port;
}
static inline error_flag canzero_get_warn_mgu1_mosfet_over_temperature() {
  extern error_flag __oe_warn_mgu1_mosfet_over_temperature;
  return __oe_warn_mgu1_mosfet_over_temperature;
}
static inline error_flag canzero_get_warn_mgu1_mcu_over_temperature() {
  extern error_flag __oe_warn_mgu1_mcu_over_temperature;
  return __oe_warn_mgu1_mcu_over_temperature;
}
static inline mgu_state canzero_get_mgu1_state() {
  extern mgu_state __oe_mgu1_state;
  return __oe_mgu1_state;
}
static inline float canzero_get_mgu1_power_estimation() {
  extern float __oe_mgu1_power_estimation;
  return __oe_mgu1_power_estimation;
}
static inline sdc_status canzero_get_mgu1_sdc_status() {
  extern sdc_status __oe_mgu1_sdc_status;
  return __oe_mgu1_sdc_status;
}
static inline error_flag canzero_get_error_mgu2_45V_over_voltage() {
  extern error_flag __oe_error_mgu2_45V_over_voltage;
  return __oe_error_mgu2_45V_over_voltage;
}
static inline error_flag canzero_get_error_mgu2_45V_under_voltage() {
  extern error_flag __oe_error_mgu2_45V_under_voltage;
  return __oe_error_mgu2_45V_under_voltage;
}
static inline error_flag canzero_get_error_mgu2_control_error() {
  extern error_flag __oe_error_mgu2_control_error;
  return __oe_error_mgu2_control_error;
}
static inline error_flag canzero_get_error_mgu2_magnet_over_temperature_starboard() {
  extern error_flag __oe_error_mgu2_magnet_over_temperature_starboard;
  return __oe_error_mgu2_magnet_over_temperature_starboard;
}
static inline error_flag canzero_get_error_mgu2_magnet_over_temperature_port() {
  extern error_flag __oe_error_mgu2_magnet_over_temperature_port;
  return __oe_error_mgu2_magnet_over_temperature_port;
}
static inline error_flag canzero_get_error_mgu2_mosfet_over_temperature() {
  extern error_flag __oe_error_mgu2_mosfet_over_temperature;
  return __oe_error_mgu2_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_mgu2_mcu_over_temperature() {
  extern error_flag __oe_error_mgu2_mcu_over_temperature;
  return __oe_error_mgu2_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_mgu2_magnet_over_temperature_starboard() {
  extern error_flag __oe_warn_mgu2_magnet_over_temperature_starboard;
  return __oe_warn_mgu2_magnet_over_temperature_starboard;
}
static inline error_flag canzero_get_warn_mgu2_magnet_over_temperature_port() {
  extern error_flag __oe_warn_mgu2_magnet_over_temperature_port;
  return __oe_warn_mgu2_magnet_over_temperature_port;
}
static inline error_flag canzero_get_warn_mgu2_mosfet_over_temperature() {
  extern error_flag __oe_warn_mgu2_mosfet_over_temperature;
  return __oe_warn_mgu2_mosfet_over_temperature;
}
static inline error_flag canzero_get_warn_mgu2_mcu_over_temperature() {
  extern error_flag __oe_warn_mgu2_mcu_over_temperature;
  return __oe_warn_mgu2_mcu_over_temperature;
}
static inline mgu_state canzero_get_mgu2_state() {
  extern mgu_state __oe_mgu2_state;
  return __oe_mgu2_state;
}
static inline float canzero_get_mgu2_power_estimation() {
  extern float __oe_mgu2_power_estimation;
  return __oe_mgu2_power_estimation;
}
static inline sdc_status canzero_get_mgu2_sdc_status() {
  extern sdc_status __oe_mgu2_sdc_status;
  return __oe_mgu2_sdc_status;
}
static inline error_flag canzero_get_error_mlu1_45V_over_voltage() {
  extern error_flag __oe_error_mlu1_45V_over_voltage;
  return __oe_error_mlu1_45V_over_voltage;
}
static inline error_flag canzero_get_error_mlu1_45V_under_voltage() {
  extern error_flag __oe_error_mlu1_45V_under_voltage;
  return __oe_error_mlu1_45V_under_voltage;
}
static inline error_flag canzero_get_error_mlu1_magnet_over_temperature() {
  extern error_flag __oe_error_mlu1_magnet_over_temperature;
  return __oe_error_mlu1_magnet_over_temperature;
}
static inline error_flag canzero_get_error_mlu1_mcu_over_temperature() {
  extern error_flag __oe_error_mlu1_mcu_over_temperature;
  return __oe_error_mlu1_mcu_over_temperature;
}
static inline error_flag canzero_get_error_mlu1_mosfet_over_temperature() {
  extern error_flag __oe_error_mlu1_mosfet_over_temperature;
  return __oe_error_mlu1_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_mlu1_control_failure() {
  extern error_flag __oe_error_mlu1_control_failure;
  return __oe_error_mlu1_control_failure;
}
static inline error_flag canzero_get_warn_mlu1_magnet_over_temperature() {
  extern error_flag __oe_warn_mlu1_magnet_over_temperature;
  return __oe_warn_mlu1_magnet_over_temperature;
}
static inline error_flag canzero_get_warn_mlu1_mcu_over_temperature() {
  extern error_flag __oe_warn_mlu1_mcu_over_temperature;
  return __oe_warn_mlu1_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_mlu1_mosfet_over_temperature() {
  extern error_flag __oe_warn_mlu1_mosfet_over_temperature;
  return __oe_warn_mlu1_mosfet_over_temperature;
}
static inline mlu_state canzero_get_mlu1_state() {
  extern mlu_state __oe_mlu1_state;
  return __oe_mlu1_state;
}
static inline float canzero_get_mlu1_power_estimation() {
  extern float __oe_mlu1_power_estimation;
  return __oe_mlu1_power_estimation;
}
static inline sdc_status canzero_get_mlu1_sdc_status() {
  extern sdc_status __oe_mlu1_sdc_status;
  return __oe_mlu1_sdc_status;
}
static inline error_flag canzero_get_error_mlu2_45V_over_voltage() {
  extern error_flag __oe_error_mlu2_45V_over_voltage;
  return __oe_error_mlu2_45V_over_voltage;
}
static inline error_flag canzero_get_error_mlu2_45V_under_voltage() {
  extern error_flag __oe_error_mlu2_45V_under_voltage;
  return __oe_error_mlu2_45V_under_voltage;
}
static inline error_flag canzero_get_error_mlu2_magnet_over_temperature() {
  extern error_flag __oe_error_mlu2_magnet_over_temperature;
  return __oe_error_mlu2_magnet_over_temperature;
}
static inline error_flag canzero_get_error_mlu2_mcu_over_temperature() {
  extern error_flag __oe_error_mlu2_mcu_over_temperature;
  return __oe_error_mlu2_mcu_over_temperature;
}
static inline error_flag canzero_get_error_mlu2_mosfet_over_temperature() {
  extern error_flag __oe_error_mlu2_mosfet_over_temperature;
  return __oe_error_mlu2_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_mlu2_control_failure() {
  extern error_flag __oe_error_mlu2_control_failure;
  return __oe_error_mlu2_control_failure;
}
static inline error_flag canzero_get_warn_mlu2_magnet_over_temperature() {
  extern error_flag __oe_warn_mlu2_magnet_over_temperature;
  return __oe_warn_mlu2_magnet_over_temperature;
}
static inline error_flag canzero_get_warn_mlu2_mcu_over_temperature() {
  extern error_flag __oe_warn_mlu2_mcu_over_temperature;
  return __oe_warn_mlu2_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_mlu2_mosfet_over_temperature() {
  extern error_flag __oe_warn_mlu2_mosfet_over_temperature;
  return __oe_warn_mlu2_mosfet_over_temperature;
}
static inline mlu_state canzero_get_mlu2_state() {
  extern mlu_state __oe_mlu2_state;
  return __oe_mlu2_state;
}
static inline float canzero_get_mlu2_power_estimation() {
  extern float __oe_mlu2_power_estimation;
  return __oe_mlu2_power_estimation;
}
static inline sdc_status canzero_get_mlu2_sdc_status() {
  extern sdc_status __oe_mlu2_sdc_status;
  return __oe_mlu2_sdc_status;
}
static inline error_flag canzero_get_error_mlu3_45V_over_voltage() {
  extern error_flag __oe_error_mlu3_45V_over_voltage;
  return __oe_error_mlu3_45V_over_voltage;
}
static inline error_flag canzero_get_error_mlu3_45V_under_voltage() {
  extern error_flag __oe_error_mlu3_45V_under_voltage;
  return __oe_error_mlu3_45V_under_voltage;
}
static inline error_flag canzero_get_error_mlu3_magnet_over_temperature() {
  extern error_flag __oe_error_mlu3_magnet_over_temperature;
  return __oe_error_mlu3_magnet_over_temperature;
}
static inline error_flag canzero_get_error_mlu3_mcu_over_temperature() {
  extern error_flag __oe_error_mlu3_mcu_over_temperature;
  return __oe_error_mlu3_mcu_over_temperature;
}
static inline error_flag canzero_get_error_mlu3_mosfet_over_temperature() {
  extern error_flag __oe_error_mlu3_mosfet_over_temperature;
  return __oe_error_mlu3_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_mlu3_control_failure() {
  extern error_flag __oe_error_mlu3_control_failure;
  return __oe_error_mlu3_control_failure;
}
static inline error_flag canzero_get_warn_mlu3_magnet_over_temperature() {
  extern error_flag __oe_warn_mlu3_magnet_over_temperature;
  return __oe_warn_mlu3_magnet_over_temperature;
}
static inline error_flag canzero_get_warn_mlu3_mcu_over_temperature() {
  extern error_flag __oe_warn_mlu3_mcu_over_temperature;
  return __oe_warn_mlu3_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_mlu3_mosfet_over_temperature() {
  extern error_flag __oe_warn_mlu3_mosfet_over_temperature;
  return __oe_warn_mlu3_mosfet_over_temperature;
}
static inline mlu_state canzero_get_mlu3_state() {
  extern mlu_state __oe_mlu3_state;
  return __oe_mlu3_state;
}
static inline float canzero_get_mlu3_power_estimation() {
  extern float __oe_mlu3_power_estimation;
  return __oe_mlu3_power_estimation;
}
static inline sdc_status canzero_get_mlu3_sdc_status() {
  extern sdc_status __oe_mlu3_sdc_status;
  return __oe_mlu3_sdc_status;
}
static inline error_flag canzero_get_error_mlu4_45V_over_voltage() {
  extern error_flag __oe_error_mlu4_45V_over_voltage;
  return __oe_error_mlu4_45V_over_voltage;
}
static inline error_flag canzero_get_error_mlu4_45V_under_voltage() {
  extern error_flag __oe_error_mlu4_45V_under_voltage;
  return __oe_error_mlu4_45V_under_voltage;
}
static inline error_flag canzero_get_error_mlu4_magnet_over_temperature() {
  extern error_flag __oe_error_mlu4_magnet_over_temperature;
  return __oe_error_mlu4_magnet_over_temperature;
}
static inline error_flag canzero_get_error_mlu4_mcu_over_temperature() {
  extern error_flag __oe_error_mlu4_mcu_over_temperature;
  return __oe_error_mlu4_mcu_over_temperature;
}
static inline error_flag canzero_get_error_mlu4_mosfet_over_temperature() {
  extern error_flag __oe_error_mlu4_mosfet_over_temperature;
  return __oe_error_mlu4_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_mlu4_control_failure() {
  extern error_flag __oe_error_mlu4_control_failure;
  return __oe_error_mlu4_control_failure;
}
static inline error_flag canzero_get_warn_mlu4_magnet_over_temperature() {
  extern error_flag __oe_warn_mlu4_magnet_over_temperature;
  return __oe_warn_mlu4_magnet_over_temperature;
}
static inline error_flag canzero_get_warn_mlu4_mcu_over_temperature() {
  extern error_flag __oe_warn_mlu4_mcu_over_temperature;
  return __oe_warn_mlu4_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_mlu4_mosfet_over_temperature() {
  extern error_flag __oe_warn_mlu4_mosfet_over_temperature;
  return __oe_warn_mlu4_mosfet_over_temperature;
}
static inline mlu_state canzero_get_mlu4_state() {
  extern mlu_state __oe_mlu4_state;
  return __oe_mlu4_state;
}
static inline float canzero_get_mlu4_power_estimation() {
  extern float __oe_mlu4_power_estimation;
  return __oe_mlu4_power_estimation;
}
static inline sdc_status canzero_get_mlu4_sdc_status() {
  extern sdc_status __oe_mlu4_sdc_status;
  return __oe_mlu4_sdc_status;
}
static inline error_flag canzero_get_error_mlu5_45V_over_voltage() {
  extern error_flag __oe_error_mlu5_45V_over_voltage;
  return __oe_error_mlu5_45V_over_voltage;
}
static inline error_flag canzero_get_error_mlu5_45V_under_voltage() {
  extern error_flag __oe_error_mlu5_45V_under_voltage;
  return __oe_error_mlu5_45V_under_voltage;
}
static inline error_flag canzero_get_error_mlu5_magnet_over_temperature() {
  extern error_flag __oe_error_mlu5_magnet_over_temperature;
  return __oe_error_mlu5_magnet_over_temperature;
}
static inline error_flag canzero_get_error_mlu5_mcu_over_temperature() {
  extern error_flag __oe_error_mlu5_mcu_over_temperature;
  return __oe_error_mlu5_mcu_over_temperature;
}
static inline error_flag canzero_get_error_mlu5_mosfet_over_temperature() {
  extern error_flag __oe_error_mlu5_mosfet_over_temperature;
  return __oe_error_mlu5_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_mlu5_control_failure() {
  extern error_flag __oe_error_mlu5_control_failure;
  return __oe_error_mlu5_control_failure;
}
static inline error_flag canzero_get_warn_mlu5_magnet_over_temperature() {
  extern error_flag __oe_warn_mlu5_magnet_over_temperature;
  return __oe_warn_mlu5_magnet_over_temperature;
}
static inline error_flag canzero_get_warn_mlu5_mcu_over_temperature() {
  extern error_flag __oe_warn_mlu5_mcu_over_temperature;
  return __oe_warn_mlu5_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_mlu5_mosfet_over_temperature() {
  extern error_flag __oe_warn_mlu5_mosfet_over_temperature;
  return __oe_warn_mlu5_mosfet_over_temperature;
}
static inline mlu_state canzero_get_mlu5_state() {
  extern mlu_state __oe_mlu5_state;
  return __oe_mlu5_state;
}
static inline float canzero_get_mlu5_power_estimation() {
  extern float __oe_mlu5_power_estimation;
  return __oe_mlu5_power_estimation;
}
static inline sdc_status canzero_get_mlu5_sdc_status() {
  extern sdc_status __oe_mlu5_sdc_status;
  return __oe_mlu5_sdc_status;
}
static inline error_flag canzero_get_error_mlu6_45V_over_voltage() {
  extern error_flag __oe_error_mlu6_45V_over_voltage;
  return __oe_error_mlu6_45V_over_voltage;
}
static inline error_flag canzero_get_error_mlu6_45V_under_voltage() {
  extern error_flag __oe_error_mlu6_45V_under_voltage;
  return __oe_error_mlu6_45V_under_voltage;
}
static inline error_flag canzero_get_error_mlu6_magnet_over_temperature() {
  extern error_flag __oe_error_mlu6_magnet_over_temperature;
  return __oe_error_mlu6_magnet_over_temperature;
}
static inline error_flag canzero_get_error_mlu6_mcu_over_temperature() {
  extern error_flag __oe_error_mlu6_mcu_over_temperature;
  return __oe_error_mlu6_mcu_over_temperature;
}
static inline error_flag canzero_get_error_mlu6_mosfet_over_temperature() {
  extern error_flag __oe_error_mlu6_mosfet_over_temperature;
  return __oe_error_mlu6_mosfet_over_temperature;
}
static inline error_flag canzero_get_error_mlu6_control_failure() {
  extern error_flag __oe_error_mlu6_control_failure;
  return __oe_error_mlu6_control_failure;
}
static inline error_flag canzero_get_warn_mlu6_magnet_over_temperature() {
  extern error_flag __oe_warn_mlu6_magnet_over_temperature;
  return __oe_warn_mlu6_magnet_over_temperature;
}
static inline error_flag canzero_get_warn_mlu6_mcu_over_temperature() {
  extern error_flag __oe_warn_mlu6_mcu_over_temperature;
  return __oe_warn_mlu6_mcu_over_temperature;
}
static inline error_flag canzero_get_warn_mlu6_mosfet_over_temperature() {
  extern error_flag __oe_warn_mlu6_mosfet_over_temperature;
  return __oe_warn_mlu6_mosfet_over_temperature;
}
static inline mlu_state canzero_get_mlu6_state() {
  extern mlu_state __oe_mlu6_state;
  return __oe_mlu6_state;
}
static inline float canzero_get_mlu6_power_estimation() {
  extern float __oe_mlu6_power_estimation;
  return __oe_mlu6_power_estimation;
}
static inline sdc_status canzero_get_mlu6_sdc_status() {
  extern sdc_status __oe_mlu6_sdc_status;
  return __oe_mlu6_sdc_status;
}
typedef struct {
  get_resp_header header;
  uint32_t data;
} canzero_message_get_resp;
static const uint32_t canzero_message_get_resp_id = 0x13F;
typedef struct {
  set_resp_header header;
} canzero_message_set_resp;
static const uint32_t canzero_message_set_resp_id = 0x15F;
typedef struct {
  mlu_command mlu_command;
} canzero_message_master_stream_mlu_control;
static const uint32_t canzero_message_master_stream_mlu_control_id = 0x52;
typedef struct {
  mgu_command mgu_command;
} canzero_message_master_stream_mgu_control;
static const uint32_t canzero_message_master_stream_mgu_control_id = 0x53;
typedef struct {
  motor_command motor_command;
} canzero_message_master_stream_motor_control;
static const uint32_t canzero_message_master_stream_motor_control_id = 0x51;
typedef struct {
  pdu_channel_control pdu24_lp_channel1_control;
  pdu_channel_control pdu24_lp_channel2_control;
  pdu_channel_control pdu24_lp_channel3_control;
  pdu_channel_control pdu24_lp_channel4_control;
  pdu_channel_control pdu24_lp_channel5_control;
  pdu_channel_control pdu24_lp_channel6_control;
  pdu_channel_control pdu24_lp_channel7_control;
  pdu_channel_control pdu24_lp_channel8_control;
  pdu_channel_control pdu24_lp_channel9_control;
  pdu_channel_control pdu24_lp_channel10_control;
  pdu_channel_control pdu24_lp_channel11_control;
  pdu_channel_control pdu24_lp_channel12_control;
  pdu_channel_control pdu24_hp_channel1_control;
  pdu_channel_control pdu24_hp_channel2_control;
  pdu_channel_control pdu24_hp_channel3_control;
  pdu_channel_control pdu24_hp_channel4_control;
} canzero_message_master_stream_pdu24_control;
static const uint32_t canzero_message_master_stream_pdu24_control_id = 0x4F;
typedef struct {
  pdu_channel_control pdu12_lp_channel1_control;
  pdu_channel_control pdu12_lp_channel2_control;
  pdu_channel_control pdu12_lp_channel3_control;
  pdu_channel_control pdu12_lp_channel4_control;
  pdu_channel_control pdu12_lp_channel5_control;
  pdu_channel_control pdu12_lp_channel6_control;
  pdu_channel_control pdu12_lp_channel7_control;
  pdu_channel_control pdu12_lp_channel8_control;
  pdu_channel_control pdu12_lp_channel9_control;
  pdu_channel_control pdu12_lp_channel10_control;
  pdu_channel_control pdu12_lp_channel11_control;
  pdu_channel_control pdu12_lp_channel12_control;
  pdu_channel_control pdu12_hp_channel1_control;
  pdu_channel_control pdu12_hp_channel2_control;
  pdu_channel_control pdu12_hp_channel3_control;
  pdu_channel_control pdu12_hp_channel4_control;
} canzero_message_master_stream_pdu12_control;
static const uint32_t canzero_message_master_stream_pdu12_control_id = 0x50;
typedef struct {
  global_state global_state;
  global_command command;
  sdc_status sdc_status;
} canzero_message_master_stream_global_state;
static const uint32_t canzero_message_master_stream_global_state_id = 0x9F;
typedef struct {
  mlu_pid_values mlu_pid_values;
} canzero_message_master_stream_mlu_pid_sync;
static const uint32_t canzero_message_master_stream_mlu_pid_sync_id = 0xBF;
typedef struct {
  node_id node_id;
} canzero_message_heartbeat;
static const uint32_t canzero_message_heartbeat_id = 0x17B;
typedef struct {
  get_req_header header;
} canzero_message_get_req;
static const uint32_t canzero_message_get_req_id = 0x13B;
typedef struct {
  set_req_header header;
  uint32_t data;
} canzero_message_set_req;
static const uint32_t canzero_message_set_req_id = 0x15B;
typedef struct {
  input_board_state state;
} canzero_message_input_board_stream_state;
static const uint32_t canzero_message_input_board_stream_state_id = 0x9A;
typedef struct {
  float position_estimation;
  float velocity_estimation;
  float acceleration_estimation;
} canzero_message_input_board_stream_state_estimation;
static const uint32_t canzero_message_input_board_stream_state_estimation_id = 0xBA;
typedef struct {
  error_flag error_mcu_over_temperature;
  error_flag error_invalid_position_estimation;
  error_flag error_invalid_position;
  error_flag error_invalid_velocity_profile;
  error_flag error_invalid_acceleration_profile;
  error_flag error_battery_over_temperature;
  error_flag error_cooling_cycle_over_pressure;
  error_flag error_cooling_cycle_low_pressure;
  error_flag error_ebox_over_temperature;
  error_flag error_cooling_cycle_over_temperature;
  error_flag warn_invalid_position_estimation;
  error_flag warn_invalid_position;
  error_flag warn_invalid_velocity_profile;
  error_flag warn_invalid_acceleration_profile;
  error_flag warn_battery_over_temperature;
  error_flag warn_cooling_cycle_over_pressure;
  error_flag warn_cooling_cycle_low_pressure;
  error_flag warn_mcu_over_temperature;
  error_flag warn_ebox_over_temperature;
  error_flag warn_cooling_cycle_over_temperature;
} canzero_message_input_board_stream_errors;
static const uint32_t canzero_message_input_board_stream_errors_id = 0x5A;
typedef struct {
  sdc_status sdc_status;
} canzero_message_input_board_stream_sdc_status;
static const uint32_t canzero_message_input_board_stream_sdc_status_id = 0x7A;
typedef struct {
  pdu_state state;
} canzero_message_pdu12_stream_state;
static const uint32_t canzero_message_pdu12_stream_state_id = 0x54;
typedef struct {
  pdu_channel_status lp_channel1_status;
  pdu_channel_status lp_channel2_status;
  pdu_channel_status lp_channel3_status;
  pdu_channel_status lp_channel4_status;
  pdu_channel_status lp_channel5_status;
  pdu_channel_status lp_channel6_status;
  pdu_channel_status lp_channel7_status;
  pdu_channel_status lp_channel8_status;
  pdu_channel_status lp_channel9_status;
  pdu_channel_status lp_channel10_status;
  pdu_channel_status lp_channel11_status;
  uint8_t lp_channel12_status;
  pdu_channel_status hp_channel1_status;
  pdu_channel_status hp_channel2_status;
  pdu_channel_status hp_channel3_status;
  pdu_channel_status hp_channel4_status;
  sdc_status sdc_status;
  error_flag mcu_over_temperature;
} canzero_message_pdu12_stream_channel_status;
static const uint32_t canzero_message_pdu12_stream_channel_status_id = 0xF5;
typedef struct {
  float total_current;
  float power_estimation;
} canzero_message_pdu12_stream_power_estimation;
static const uint32_t canzero_message_pdu12_stream_power_estimation_id = 0x115;
typedef struct {
  pdu_state state;
} canzero_message_pdu24_stream_state;
static const uint32_t canzero_message_pdu24_stream_state_id = 0xB4;
typedef struct {
  pdu_channel_status lp_channel1_status;
  pdu_channel_status lp_channel2_status;
  pdu_channel_status lp_channel3_status;
  pdu_channel_status lp_channel4_status;
  pdu_channel_status lp_channel5_status;
  pdu_channel_status lp_channel6_status;
  pdu_channel_status lp_channel7_status;
  pdu_channel_status lp_channel8_status;
  pdu_channel_status lp_channel9_status;
  pdu_channel_status lp_channel10_status;
  pdu_channel_status lp_channel11_status;
  uint8_t lp_channel12_status;
  pdu_channel_status hp_channel1_status;
  pdu_channel_status hp_channel2_status;
  pdu_channel_status hp_channel3_status;
  pdu_channel_status hp_channel4_status;
  sdc_status sdc_status;
  error_flag mcu_over_temperature;
} canzero_message_pdu24_stream_channel_status;
static const uint32_t canzero_message_pdu24_stream_channel_status_id = 0x74;
typedef struct {
  float total_current;
  float power_estimation;
} canzero_message_pdu24_stream_power_estimation;
static const uint32_t canzero_message_pdu24_stream_power_estimation_id = 0x94;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_dslim_over_temperature;
  error_flag error_mosfet_over_temperature;
  error_flag error_control_failure;
  error_flag error_mcu_over_temperature;
  error_flag warn_dslim_over_temperature;
  error_flag warn_mosfet_over_temperature;
  error_flag warn_mcu_over_temperature;
} canzero_message_motor_driver_stream_errors;
static const uint32_t canzero_message_motor_driver_stream_errors_id = 0x75;
typedef struct {
  float dc_current;
  float power_estimation;
} canzero_message_motor_driver_stream_power_estimation;
static const uint32_t canzero_message_motor_driver_stream_power_estimation_id = 0x95;
typedef struct {
  motor_state state;
  float local_acceleration;
} canzero_message_motor_driver_stream_state;
static const uint32_t canzero_message_motor_driver_stream_state_id = 0xD5;
typedef struct {
  sdc_status sdc_status;
} canzero_message_motor_driver_stream_sdc_status;
static const uint32_t canzero_message_motor_driver_stream_sdc_status_id = 0xB5;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_control_error;
  error_flag error_magnet_over_temperature_starboard;
  error_flag error_magnet_over_temperature_port;
  error_flag error_mosfet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag warn_magnet_over_temperature_starboard;
  error_flag warn_magnet_over_temperature_port;
  error_flag warn_mosfet_over_temperature;
  error_flag warn_mcu_over_temperature;
} canzero_message_mgu1_stream_errors;
static const uint32_t canzero_message_mgu1_stream_errors_id = 0xDA;
typedef struct {
  mgu_state state;
  mgu_command command;
} canzero_message_mgu1_stream_state;
static const uint32_t canzero_message_mgu1_stream_state_id = 0x59;
typedef struct {
  float dc_current_starboard;
  float dc_current_port;
  float power_estimation;
} canzero_message_mgu1_stream_power_estimation;
static const uint32_t canzero_message_mgu1_stream_power_estimation_id = 0xFA;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mgu1_stream_sdc_status;
static const uint32_t canzero_message_mgu1_stream_sdc_status_id = 0x11A;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_control_error;
  error_flag error_magnet_over_temperature_starboard;
  error_flag error_magnet_over_temperature_port;
  error_flag error_mosfet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag warn_magnet_over_temperature_starboard;
  error_flag warn_magnet_over_temperature_port;
  error_flag warn_mosfet_over_temperature;
  error_flag warn_mcu_over_temperature;
} canzero_message_mgu2_stream_errors;
static const uint32_t canzero_message_mgu2_stream_errors_id = 0x79;
typedef struct {
  mgu_state state;
} canzero_message_mgu2_stream_state;
static const uint32_t canzero_message_mgu2_stream_state_id = 0xD9;
typedef struct {
  float dc_current_starboard;
  float dc_current_port;
  float power_estimation;
} canzero_message_mgu2_stream_power_estimation;
static const uint32_t canzero_message_mgu2_stream_power_estimation_id = 0x99;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mgu2_stream_sdc_status;
static const uint32_t canzero_message_mgu2_stream_sdc_status_id = 0xB9;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_magnet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag error_mosfet_over_temperature;
  error_flag error_control_failure;
  error_flag warn_magnet_over_temperature;
  error_flag warn_mcu_over_temperature;
  error_flag warn_mosfet_over_temperature;
} canzero_message_mlu1_stream_errors;
static const uint32_t canzero_message_mlu1_stream_errors_id = 0xF9;
typedef struct {
  mlu_state state;
  mlu_command command;
} canzero_message_mlu1_stream_state;
static const uint32_t canzero_message_mlu1_stream_state_id = 0x78;
typedef struct {
  float dc_current;
  float power_estimation;
} canzero_message_mlu1_stream_power_estimation;
static const uint32_t canzero_message_mlu1_stream_power_estimation_id = 0x119;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mlu1_stream_sdc_status;
static const uint32_t canzero_message_mlu1_stream_sdc_status_id = 0x58;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_magnet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag error_mosfet_over_temperature;
  error_flag error_control_failure;
  error_flag warn_magnet_over_temperature;
  error_flag warn_mcu_over_temperature;
  error_flag warn_mosfet_over_temperature;
} canzero_message_mlu2_stream_errors;
static const uint32_t canzero_message_mlu2_stream_errors_id = 0x98;
typedef struct {
  mlu_state state;
  mlu_command command;
} canzero_message_mlu2_stream_state;
static const uint32_t canzero_message_mlu2_stream_state_id = 0xF8;
typedef struct {
  float dc_current;
  float power_estimation;
} canzero_message_mlu2_stream_power_estimation;
static const uint32_t canzero_message_mlu2_stream_power_estimation_id = 0xB8;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mlu2_stream_sdc_status;
static const uint32_t canzero_message_mlu2_stream_sdc_status_id = 0xD8;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_magnet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag error_mosfet_over_temperature;
  error_flag error_control_failure;
  error_flag warn_magnet_over_temperature;
  error_flag warn_mcu_over_temperature;
  error_flag warn_mosfet_over_temperature;
} canzero_message_mlu3_stream_errors;
static const uint32_t canzero_message_mlu3_stream_errors_id = 0x118;
typedef struct {
  mlu_state state;
  mlu_command command;
} canzero_message_mlu3_stream_state;
static const uint32_t canzero_message_mlu3_stream_state_id = 0x97;
typedef struct {
  float dc_current;
  float power_estimation;
} canzero_message_mlu3_stream_power_estimation;
static const uint32_t canzero_message_mlu3_stream_power_estimation_id = 0x57;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mlu3_stream_sdc_status;
static const uint32_t canzero_message_mlu3_stream_sdc_status_id = 0x77;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_magnet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag error_mosfet_over_temperature;
  error_flag error_control_failure;
  error_flag warn_magnet_over_temperature;
  error_flag warn_mcu_over_temperature;
  error_flag warn_mosfet_over_temperature;
} canzero_message_mlu4_stream_errors;
static const uint32_t canzero_message_mlu4_stream_errors_id = 0xB7;
typedef struct {
  mlu_state state;
  mlu_command command;
} canzero_message_mlu4_stream_state;
static const uint32_t canzero_message_mlu4_stream_state_id = 0x117;
typedef struct {
  float dc_current;
  float power_estimation;
} canzero_message_mlu4_stream_power_estimation;
static const uint32_t canzero_message_mlu4_stream_power_estimation_id = 0xD7;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mlu4_stream_sdc_status;
static const uint32_t canzero_message_mlu4_stream_sdc_status_id = 0xF7;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_magnet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag error_mosfet_over_temperature;
  error_flag error_control_failure;
  error_flag warn_magnet_over_temperature;
  error_flag warn_mcu_over_temperature;
  error_flag warn_mosfet_over_temperature;
} canzero_message_mlu5_stream_errors;
static const uint32_t canzero_message_mlu5_stream_errors_id = 0x56;
typedef struct {
  mlu_state state;
  mlu_command command;
} canzero_message_mlu5_stream_state;
static const uint32_t canzero_message_mlu5_stream_state_id = 0xB6;
typedef struct {
  float dc_current;
  float power_estimation;
} canzero_message_mlu5_stream_power_estimation;
static const uint32_t canzero_message_mlu5_stream_power_estimation_id = 0x76;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mlu5_stream_sdc_status;
static const uint32_t canzero_message_mlu5_stream_sdc_status_id = 0x96;
typedef struct {
  error_flag error_45V_over_voltage;
  error_flag error_45V_under_voltage;
  error_flag error_magnet_over_temperature;
  error_flag error_mcu_over_temperature;
  error_flag error_mosfet_over_temperature;
  error_flag error_control_failure;
  error_flag warn_magnet_over_temperature;
  error_flag warn_mcu_over_temperature;
  error_flag warn_mosfet_over_temperature;
} canzero_message_mlu6_stream_errors;
static const uint32_t canzero_message_mlu6_stream_errors_id = 0xD6;
typedef struct {
  mlu_state state;
  mlu_command command;
} canzero_message_mlu6_stream_state;
static const uint32_t canzero_message_mlu6_stream_state_id = 0x55;
typedef struct {
  float dc_current;
  float power_estimation;
} canzero_message_mlu6_stream_power_estimation;
static const uint32_t canzero_message_mlu6_stream_power_estimation_id = 0xF6;
typedef struct {
  sdc_status sdc_status;
} canzero_message_mlu6_stream_sdc_status;
static const uint32_t canzero_message_mlu6_stream_sdc_status_id = 0x116;
static void canzero_serialize_canzero_message_get_resp(canzero_message_get_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x13F;
  frame->dlc = 8;
  ((uint32_t*)data)[0] = (uint8_t)(msg->header.sof & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.eof & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.toggle & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13))) << 3;
  ((uint32_t*)data)[0] |= msg->header.client_id << 16;
  ((uint32_t*)data)[0] |= msg->header.server_id << 24;
  ((uint32_t*)data)[1] = msg->data;
}
static void canzero_serialize_canzero_message_set_resp(canzero_message_set_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x15F;
  frame->dlc = 4;
  ((uint32_t*)data)[0] = (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13)));
  ((uint32_t*)data)[0] |= msg->header.client_id << 13;
  ((uint32_t*)data)[0] |= msg->header.server_id << 21;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.erno & (0xFF >> (8 - 1))) << 29;
}
static void canzero_serialize_canzero_message_master_stream_mlu_control(canzero_message_master_stream_mlu_control* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x52;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->mlu_command & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_master_stream_mgu_control(canzero_message_master_stream_mgu_control* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x53;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->mgu_command & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_master_stream_motor_control(canzero_message_master_stream_motor_control* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x51;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->motor_command & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_master_stream_pdu24_control(canzero_message_master_stream_pdu24_control* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x4F;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->pdu24_lp_channel1_control & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel2_control & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel3_control & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel4_control & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel5_control & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel6_control & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel7_control & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel8_control & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel9_control & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel10_control & (0xFF >> (8 - 1))) << 9;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel11_control & (0xFF >> (8 - 1))) << 10;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_lp_channel12_control & (0xFF >> (8 - 1))) << 11;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_hp_channel1_control & (0xFF >> (8 - 1))) << 12;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_hp_channel2_control & (0xFF >> (8 - 1))) << 13;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_hp_channel3_control & (0xFF >> (8 - 1))) << 14;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu24_hp_channel4_control & (0xFF >> (8 - 1))) << 15;
}
static void canzero_serialize_canzero_message_master_stream_pdu12_control(canzero_message_master_stream_pdu12_control* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x50;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->pdu12_lp_channel1_control & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel2_control & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel3_control & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel4_control & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel5_control & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel6_control & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel7_control & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel8_control & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel9_control & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel10_control & (0xFF >> (8 - 1))) << 9;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel11_control & (0xFF >> (8 - 1))) << 10;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_lp_channel12_control & (0xFF >> (8 - 1))) << 11;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_hp_channel1_control & (0xFF >> (8 - 1))) << 12;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_hp_channel2_control & (0xFF >> (8 - 1))) << 13;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_hp_channel3_control & (0xFF >> (8 - 1))) << 14;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->pdu12_hp_channel4_control & (0xFF >> (8 - 1))) << 15;
}
static void canzero_serialize_canzero_message_master_stream_global_state(canzero_message_master_stream_global_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x9F;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->global_state & (0xFF >> (8 - 4)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 4))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1))) << 8;
}
static void canzero_serialize_canzero_message_master_stream_mlu_pid_sync(canzero_message_master_stream_mlu_pid_sync* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xBF;
  frame->dlc = 8;
  float p_value_0 = (msg->mlu_pid_values.p_value - -100) / 0.00009536747711538177;
  if (p_value_0 > 4294965248.000) {
    p_value_0 = 4294965248.000;
  }
  ((uint32_t*)data)[0] = (uint32_t)((uint32_t) p_value_0 & (0xFFFFFFFF >> (32 - 21)));
  float i_value_21 = (msg->mlu_pid_values.i_value - -100) / 0.00009536747711538177;
  if (i_value_21 > 4294965248.000) {
    i_value_21 = 4294965248.000;
  }
  ((uint64_t*)data)[0] |= ((uint64_t)(uint32_t)((uint32_t) i_value_21 & (0xFFFFFFFF >> (32 - 21)))) << 21 ;
  float d_value_42 = (msg->mlu_pid_values.d_value - -100) / 0.00009536747711538177;
  if (d_value_42 > 4294965248.000) {
    d_value_42 = 4294965248.000;
  }
  ((uint32_t*)data)[1] |= (uint32_t)((uint32_t) d_value_42 & (0xFFFFFFFF >> (32 - 21))) << 10;
}
static void canzero_serialize_canzero_message_heartbeat(canzero_message_heartbeat* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x17B;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->node_id & (0xFF >> (8 - 4)));
}
static void canzero_serialize_canzero_message_get_req(canzero_message_get_req* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x13B;
  frame->dlc = 4;
  ((uint32_t*)data)[0] = (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13)));
  ((uint32_t*)data)[0] |= msg->header.client_id << 13;
  ((uint32_t*)data)[0] |= msg->header.server_id << 21;
}
static void canzero_serialize_canzero_message_set_req(canzero_message_set_req* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x15B;
  frame->dlc = 8;
  ((uint32_t*)data)[0] = (uint8_t)(msg->header.sof & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.eof & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->header.toggle & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint16_t)(msg->header.od_index & (0xFFFF >> (16 - 13))) << 3;
  ((uint32_t*)data)[0] |= msg->header.client_id << 16;
  ((uint32_t*)data)[0] |= msg->header.server_id << 24;
  ((uint32_t*)data)[1] = msg->data;
}
static void canzero_serialize_canzero_message_input_board_stream_state(canzero_message_input_board_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x9A;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_input_board_stream_state_estimation(canzero_message_input_board_stream_state_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xBA;
  frame->dlc = 6;
  float position_estimation_0 = (msg->position_estimation - -100) / 0.0030518043793392844;
  if (position_estimation_0 > 4294901760.000) {
    position_estimation_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) position_estimation_0;
  float velocity_estimation_16 = (msg->velocity_estimation - -10) / 0.00030518043793392844;
  if (velocity_estimation_16 > 4294901760.000) {
    velocity_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) velocity_estimation_16 << 16;
  float acceleration_estimation_32 = (msg->acceleration_estimation - -5) / 0.00008392462043183032;
  if (acceleration_estimation_32 > 4294901760.000) {
    acceleration_estimation_32 = 4294901760.000;
  }
  ((uint32_t*)data)[1] = (uint32_t) acceleration_estimation_32;
}
static void canzero_serialize_canzero_message_input_board_stream_errors(canzero_message_input_board_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x5A;
  frame->dlc = 3;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_invalid_position_estimation & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_invalid_position & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_invalid_velocity_profile & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_invalid_acceleration_profile & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_battery_over_temperature & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_cooling_cycle_over_pressure & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_cooling_cycle_low_pressure & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_ebox_over_temperature & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_cooling_cycle_over_temperature & (0xFF >> (8 - 1))) << 9;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_invalid_position_estimation & (0xFF >> (8 - 1))) << 10;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_invalid_position & (0xFF >> (8 - 1))) << 11;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_invalid_velocity_profile & (0xFF >> (8 - 1))) << 12;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_invalid_acceleration_profile & (0xFF >> (8 - 1))) << 13;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_battery_over_temperature & (0xFF >> (8 - 1))) << 14;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_cooling_cycle_over_pressure & (0xFF >> (8 - 1))) << 15;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_cooling_cycle_low_pressure & (0xFF >> (8 - 1))) << 16;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 17;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_ebox_over_temperature & (0xFF >> (8 - 1))) << 18;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_cooling_cycle_over_temperature & (0xFF >> (8 - 1))) << 19;
}
static void canzero_serialize_canzero_message_input_board_stream_sdc_status(canzero_message_input_board_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x7A;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_pdu12_stream_state(canzero_message_pdu12_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x54;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_pdu12_stream_channel_status(canzero_message_pdu12_stream_channel_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xF5;
  frame->dlc = 5;
  ((uint32_t*)data)[0] = (uint8_t)(msg->lp_channel1_status & (0xFF >> (8 - 2)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel2_status & (0xFF >> (8 - 2))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel3_status & (0xFF >> (8 - 2))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel4_status & (0xFF >> (8 - 2))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel5_status & (0xFF >> (8 - 2))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel6_status & (0xFF >> (8 - 2))) << 10;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel7_status & (0xFF >> (8 - 2))) << 12;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel8_status & (0xFF >> (8 - 2))) << 14;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel9_status & (0xFF >> (8 - 2))) << 16;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel10_status & (0xFF >> (8 - 2))) << 18;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel11_status & (0xFF >> (8 - 2))) << 20;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel12_status & (0xFF >> (8 - 1))) << 22;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->hp_channel1_status & (0xFF >> (8 - 2))) << 23;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->hp_channel2_status & (0xFF >> (8 - 2))) << 25;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->hp_channel3_status & (0xFF >> (8 - 2))) << 27;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->hp_channel4_status & (0xFF >> (8 - 2))) << 29;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1))) << 31;
  ((uint32_t*)data)[1] = (uint8_t)(msg->mcu_over_temperature & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_pdu12_stream_power_estimation(canzero_message_pdu12_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x115;
  frame->dlc = 4;
  float total_current_0 = (msg->total_current - 0) / 0.0012207217517357137;
  if (total_current_0 > 4294901760.000) {
    total_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) total_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.030518043793392843;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_pdu24_stream_state(canzero_message_pdu24_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xB4;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_pdu24_stream_channel_status(canzero_message_pdu24_stream_channel_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x74;
  frame->dlc = 5;
  ((uint32_t*)data)[0] = (uint8_t)(msg->lp_channel1_status & (0xFF >> (8 - 2)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel2_status & (0xFF >> (8 - 2))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel3_status & (0xFF >> (8 - 2))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel4_status & (0xFF >> (8 - 2))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel5_status & (0xFF >> (8 - 2))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel6_status & (0xFF >> (8 - 2))) << 10;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel7_status & (0xFF >> (8 - 2))) << 12;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel8_status & (0xFF >> (8 - 2))) << 14;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel9_status & (0xFF >> (8 - 2))) << 16;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel10_status & (0xFF >> (8 - 2))) << 18;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel11_status & (0xFF >> (8 - 2))) << 20;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->lp_channel12_status & (0xFF >> (8 - 1))) << 22;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->hp_channel1_status & (0xFF >> (8 - 2))) << 23;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->hp_channel2_status & (0xFF >> (8 - 2))) << 25;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->hp_channel3_status & (0xFF >> (8 - 2))) << 27;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->hp_channel4_status & (0xFF >> (8 - 2))) << 29;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1))) << 31;
  ((uint32_t*)data)[1] = (uint8_t)(msg->mcu_over_temperature & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_pdu24_stream_power_estimation(canzero_message_pdu24_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x94;
  frame->dlc = 4;
  float total_current_0 = (msg->total_current - 0) / 0.0012207217517357137;
  if (total_current_0 > 4294901760.000) {
    total_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) total_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.030518043793392843;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_motor_driver_stream_errors(canzero_message_motor_driver_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x75;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_dslim_over_temperature & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_failure & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_dslim_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 8;
}
static void canzero_serialize_canzero_message_motor_driver_stream_power_estimation(canzero_message_motor_driver_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x95;
  frame->dlc = 4;
  float dc_current_0 = (msg->dc_current - -100) / 0.0030518043793392844;
  if (dc_current_0 > 4294901760.000) {
    dc_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.030518043793392843;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_motor_driver_stream_state(canzero_message_motor_driver_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xD5;
  frame->dlc = 3;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 4)));
  float local_acceleration_4 = (msg->local_acceleration - -10) / 0.00030518043793392844;
  if (local_acceleration_4 > 4294901760.000) {
    local_acceleration_4 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) local_acceleration_4 << 4;
}
static void canzero_serialize_canzero_message_motor_driver_stream_sdc_status(canzero_message_motor_driver_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xB5;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mgu1_stream_errors(canzero_message_mgu1_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xDA;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_error & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature_starboard & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature_port & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature_starboard & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature_port & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 9;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 10;
}
static void canzero_serialize_canzero_message_mgu1_stream_state(canzero_message_mgu1_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x59;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 3))) << 3;
}
static void canzero_serialize_canzero_message_mgu1_stream_power_estimation(canzero_message_mgu1_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xFA;
  frame->dlc = 6;
  float dc_current_starboard_0 = (msg->dc_current_starboard - -10) / 0.0009155413138017853;
  if (dc_current_starboard_0 > 4294901760.000) {
    dc_current_starboard_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_starboard_0;
  float dc_current_port_16 = (msg->dc_current_port - -10) / 0.0009155413138017853;
  if (dc_current_port_16 > 4294901760.000) {
    dc_current_port_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) dc_current_port_16 << 16;
  float power_estimation_32 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_32 > 4294901760.000) {
    power_estimation_32 = 4294901760.000;
  }
  ((uint32_t*)data)[1] = (uint32_t) power_estimation_32;
}
static void canzero_serialize_canzero_message_mgu1_stream_sdc_status(canzero_message_mgu1_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x11A;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mgu2_stream_errors(canzero_message_mgu2_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x79;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_error & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature_starboard & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature_port & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature_starboard & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature_port & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 9;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 10;
}
static void canzero_serialize_canzero_message_mgu2_stream_state(canzero_message_mgu2_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xD9;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_mgu2_stream_power_estimation(canzero_message_mgu2_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x99;
  frame->dlc = 6;
  float dc_current_starboard_0 = (msg->dc_current_starboard - -10) / 0.0009155413138017853;
  if (dc_current_starboard_0 > 4294901760.000) {
    dc_current_starboard_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_starboard_0;
  float dc_current_port_16 = (msg->dc_current_port - -10) / 0.0009155413138017853;
  if (dc_current_port_16 > 4294901760.000) {
    dc_current_port_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) dc_current_port_16 << 16;
  float power_estimation_32 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_32 > 4294901760.000) {
    power_estimation_32 = 4294901760.000;
  }
  ((uint32_t*)data)[1] = (uint32_t) power_estimation_32;
}
static void canzero_serialize_canzero_message_mgu2_stream_sdc_status(canzero_message_mgu2_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xB9;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mlu1_stream_errors(canzero_message_mlu1_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xF9;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_failure & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 8;
}
static void canzero_serialize_canzero_message_mlu1_stream_state(canzero_message_mlu1_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x78;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 3))) << 3;
}
static void canzero_serialize_canzero_message_mlu1_stream_power_estimation(canzero_message_mlu1_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x119;
  frame->dlc = 4;
  float dc_current_0 = (msg->dc_current - -10) / 0.0009155413138017853;
  if (dc_current_0 > 4294901760.000) {
    dc_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_mlu1_stream_sdc_status(canzero_message_mlu1_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x58;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mlu2_stream_errors(canzero_message_mlu2_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x98;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_failure & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 8;
}
static void canzero_serialize_canzero_message_mlu2_stream_state(canzero_message_mlu2_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xF8;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 3))) << 3;
}
static void canzero_serialize_canzero_message_mlu2_stream_power_estimation(canzero_message_mlu2_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xB8;
  frame->dlc = 4;
  float dc_current_0 = (msg->dc_current - -10) / 0.0009155413138017853;
  if (dc_current_0 > 4294901760.000) {
    dc_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_mlu2_stream_sdc_status(canzero_message_mlu2_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xD8;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mlu3_stream_errors(canzero_message_mlu3_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x118;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_failure & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 8;
}
static void canzero_serialize_canzero_message_mlu3_stream_state(canzero_message_mlu3_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x97;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 3))) << 3;
}
static void canzero_serialize_canzero_message_mlu3_stream_power_estimation(canzero_message_mlu3_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x57;
  frame->dlc = 4;
  float dc_current_0 = (msg->dc_current - -10) / 0.0009155413138017853;
  if (dc_current_0 > 4294901760.000) {
    dc_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_mlu3_stream_sdc_status(canzero_message_mlu3_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x77;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mlu4_stream_errors(canzero_message_mlu4_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xB7;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_failure & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 8;
}
static void canzero_serialize_canzero_message_mlu4_stream_state(canzero_message_mlu4_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x117;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 3))) << 3;
}
static void canzero_serialize_canzero_message_mlu4_stream_power_estimation(canzero_message_mlu4_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xD7;
  frame->dlc = 4;
  float dc_current_0 = (msg->dc_current - -10) / 0.0009155413138017853;
  if (dc_current_0 > 4294901760.000) {
    dc_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_mlu4_stream_sdc_status(canzero_message_mlu4_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xF7;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mlu5_stream_errors(canzero_message_mlu5_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x56;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_failure & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 8;
}
static void canzero_serialize_canzero_message_mlu5_stream_state(canzero_message_mlu5_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xB6;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 3))) << 3;
}
static void canzero_serialize_canzero_message_mlu5_stream_power_estimation(canzero_message_mlu5_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x76;
  frame->dlc = 4;
  float dc_current_0 = (msg->dc_current - -10) / 0.0009155413138017853;
  if (dc_current_0 > 4294901760.000) {
    dc_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_mlu5_stream_sdc_status(canzero_message_mlu5_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x96;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_mlu6_stream_errors(canzero_message_mlu6_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xD6;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->error_45V_over_voltage & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_45V_under_voltage & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_magnet_over_temperature & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mcu_over_temperature & (0xFF >> (8 - 1))) << 3;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_mosfet_over_temperature & (0xFF >> (8 - 1))) << 4;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->error_control_failure & (0xFF >> (8 - 1))) << 5;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_magnet_over_temperature & (0xFF >> (8 - 1))) << 6;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mcu_over_temperature & (0xFF >> (8 - 1))) << 7;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->warn_mosfet_over_temperature & (0xFF >> (8 - 1))) << 8;
}
static void canzero_serialize_canzero_message_mlu6_stream_state(canzero_message_mlu6_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x55;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->state & (0xFF >> (8 - 3)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->command & (0xFF >> (8 - 3))) << 3;
}
static void canzero_serialize_canzero_message_mlu6_stream_power_estimation(canzero_message_mlu6_stream_power_estimation* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xF6;
  frame->dlc = 4;
  float dc_current_0 = (msg->dc_current - -10) / 0.0009155413138017853;
  if (dc_current_0 > 4294901760.000) {
    dc_current_0 = 4294901760.000;
  }
  ((uint32_t*)data)[0] = (uint32_t) dc_current_0;
  float power_estimation_16 = (msg->power_estimation - 0) / 0.015259021896696421;
  if (power_estimation_16 > 4294901760.000) {
    power_estimation_16 = 4294901760.000;
  }
  ((uint32_t*)data)[0] |= (uint32_t) power_estimation_16 << 16;
}
static void canzero_serialize_canzero_message_mlu6_stream_sdc_status(canzero_message_mlu6_stream_sdc_status* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x116;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->sdc_status & (0xFF >> (8 - 1)));
}
static void canzero_deserialize_canzero_message_get_resp(canzero_frame* frame, canzero_message_get_resp* msg) {
  uint8_t* data = frame->data;
  msg->header.sof = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->header.eof = ((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.toggle = ((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.od_index = ((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8)));
  msg->data = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 32)));
}
static void canzero_deserialize_canzero_message_set_resp(canzero_frame* frame, canzero_message_set_resp* msg) {
  uint8_t* data = frame->data;
  msg->header.od_index = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 21) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.erno = (set_resp_erno)((((uint32_t*)data)[0] >> 29) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_master_stream_mlu_control(canzero_frame* frame, canzero_message_master_stream_mlu_control* msg) {
  uint8_t* data = frame->data;
  msg->mlu_command = (mlu_command)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_master_stream_mgu_control(canzero_frame* frame, canzero_message_master_stream_mgu_control* msg) {
  uint8_t* data = frame->data;
  msg->mgu_command = (mgu_command)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_master_stream_motor_control(canzero_frame* frame, canzero_message_master_stream_motor_control* msg) {
  uint8_t* data = frame->data;
  msg->motor_command = (motor_command)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_master_stream_pdu24_control(canzero_frame* frame, canzero_message_master_stream_pdu24_control* msg) {
  uint8_t* data = frame->data;
  msg->pdu24_lp_channel1_control = (pdu_channel_control)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel2_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel3_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel4_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel5_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel6_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel7_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel8_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel9_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel10_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel11_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_lp_channel12_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 11) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_hp_channel1_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 12) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_hp_channel2_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_hp_channel3_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 14) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu24_hp_channel4_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 15) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_master_stream_pdu12_control(canzero_frame* frame, canzero_message_master_stream_pdu12_control* msg) {
  uint8_t* data = frame->data;
  msg->pdu12_lp_channel1_control = (pdu_channel_control)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel2_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel3_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel4_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel5_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel6_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel7_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel8_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel9_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel10_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel11_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_lp_channel12_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 11) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_hp_channel1_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 12) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_hp_channel2_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_hp_channel3_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 14) & (0xFFFFFFFF >> (32 - 1)));
  msg->pdu12_hp_channel4_control = (pdu_channel_control)((((uint32_t*)data)[0] >> 15) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_master_stream_global_state(canzero_frame* frame, canzero_message_master_stream_global_state* msg) {
  uint8_t* data = frame->data;
  msg->global_state = (global_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
  msg->command = (global_command)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 4)));
  msg->sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_master_stream_mlu_pid_sync(canzero_frame* frame, canzero_message_master_stream_mlu_pid_sync* msg) {
  uint8_t* data = frame->data;
  msg->mlu_pid_values.p_value = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 21))) * 0.00009536747711538177 + -100;
  msg->mlu_pid_values.i_value = ((((uint64_t*)data)[0] >> 21) & (0xFFFFFFFFFFFFFFFF >> (64 - 21))) * 0.00009536747711538177 + -100;
  msg->mlu_pid_values.d_value = ((((uint32_t*)data)[1] >> 10) & (0xFFFFFFFF >> (32 - 21))) * 0.00009536747711538177 + -100;
}
static void canzero_deserialize_canzero_message_heartbeat(canzero_frame* frame, canzero_message_heartbeat* msg) {
  uint8_t* data = frame->data;
  msg->node_id = (node_id)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
}
static void canzero_deserialize_canzero_message_get_req(canzero_frame* frame, canzero_message_get_req* msg) {
  uint8_t* data = frame->data;
  msg->header.od_index = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 21) & (0xFFFFFFFF >> (32 - 8)));
}
static void canzero_deserialize_canzero_message_set_req(canzero_frame* frame, canzero_message_set_req* msg) {
  uint8_t* data = frame->data;
  msg->header.sof = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->header.eof = ((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.toggle = ((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->header.od_index = ((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 13)));
  msg->header.client_id = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8)));
  msg->header.server_id = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8)));
  msg->data = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 32)));
}
static void canzero_deserialize_canzero_message_input_board_stream_state(canzero_frame* frame, canzero_message_input_board_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (input_board_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_input_board_stream_state_estimation(canzero_frame* frame, canzero_message_input_board_stream_state_estimation* msg) {
  uint8_t* data = frame->data;
  msg->position_estimation = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0030518043793392844 + -100;
  msg->velocity_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + -10;
  msg->acceleration_estimation = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 16))) * 0.00008392462043183032 + -5;
}
static void canzero_deserialize_canzero_message_input_board_stream_errors(canzero_frame* frame, canzero_message_input_board_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_mcu_over_temperature = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_invalid_position_estimation = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_invalid_position = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_invalid_velocity_profile = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_invalid_acceleration_profile = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_battery_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_cooling_cycle_over_pressure = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_cooling_cycle_low_pressure = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_ebox_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_cooling_cycle_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_invalid_position_estimation = (error_flag)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_invalid_position = (error_flag)((((uint32_t*)data)[0] >> 11) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_invalid_velocity_profile = (error_flag)((((uint32_t*)data)[0] >> 12) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_invalid_acceleration_profile = (error_flag)((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_battery_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 14) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_cooling_cycle_over_pressure = (error_flag)((((uint32_t*)data)[0] >> 15) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_cooling_cycle_low_pressure = (error_flag)((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 17) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_ebox_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 18) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_cooling_cycle_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 19) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_input_board_stream_sdc_status(canzero_frame* frame, canzero_message_input_board_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_pdu12_stream_state(canzero_frame* frame, canzero_message_pdu12_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (pdu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_pdu12_stream_channel_status(canzero_frame* frame, canzero_message_pdu12_stream_channel_status* msg) {
  uint8_t* data = frame->data;
  msg->lp_channel1_status = (pdu_channel_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel2_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel3_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel4_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel5_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel6_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel7_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 12) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel8_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 14) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel9_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel10_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 18) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel11_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 20) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel12_status = ((((uint32_t*)data)[0] >> 22) & (0xFFFFFFFF >> (32 - 1)));
  msg->hp_channel1_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 23) & (0xFFFFFFFF >> (32 - 2)));
  msg->hp_channel2_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 25) & (0xFFFFFFFF >> (32 - 2)));
  msg->hp_channel3_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 27) & (0xFFFFFFFF >> (32 - 2)));
  msg->hp_channel4_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 29) & (0xFFFFFFFF >> (32 - 2)));
  msg->sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 31) & (0xFFFFFFFF >> (32 - 1)));
  msg->mcu_over_temperature = (error_flag)(((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_pdu12_stream_power_estimation(canzero_frame* frame, canzero_message_pdu12_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->total_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0012207217517357137 + 0;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.030518043793392843 + 0;
}
static void canzero_deserialize_canzero_message_pdu24_stream_state(canzero_frame* frame, canzero_message_pdu24_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (pdu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_pdu24_stream_channel_status(canzero_frame* frame, canzero_message_pdu24_stream_channel_status* msg) {
  uint8_t* data = frame->data;
  msg->lp_channel1_status = (pdu_channel_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel2_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel3_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel4_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel5_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel6_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel7_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 12) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel8_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 14) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel9_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel10_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 18) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel11_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 20) & (0xFFFFFFFF >> (32 - 2)));
  msg->lp_channel12_status = ((((uint32_t*)data)[0] >> 22) & (0xFFFFFFFF >> (32 - 1)));
  msg->hp_channel1_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 23) & (0xFFFFFFFF >> (32 - 2)));
  msg->hp_channel2_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 25) & (0xFFFFFFFF >> (32 - 2)));
  msg->hp_channel3_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 27) & (0xFFFFFFFF >> (32 - 2)));
  msg->hp_channel4_status = (pdu_channel_status)((((uint32_t*)data)[0] >> 29) & (0xFFFFFFFF >> (32 - 2)));
  msg->sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 31) & (0xFFFFFFFF >> (32 - 1)));
  msg->mcu_over_temperature = (error_flag)(((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_pdu24_stream_power_estimation(canzero_frame* frame, canzero_message_pdu24_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->total_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0012207217517357137 + 0;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.030518043793392843 + 0;
}
static void canzero_deserialize_canzero_message_motor_driver_stream_errors(canzero_frame* frame, canzero_message_motor_driver_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_dslim_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_failure = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_dslim_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_motor_driver_stream_power_estimation(canzero_frame* frame, canzero_message_motor_driver_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0030518043793392844 + -100;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.030518043793392843 + 0;
}
static void canzero_deserialize_canzero_message_motor_driver_stream_state(canzero_frame* frame, canzero_message_motor_driver_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (motor_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
  msg->local_acceleration = ((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + -10;
}
static void canzero_deserialize_canzero_message_motor_driver_stream_sdc_status(canzero_frame* frame, canzero_message_motor_driver_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mgu1_stream_errors(canzero_frame* frame, canzero_message_mgu1_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_error = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature_starboard = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature_port = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature_starboard = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature_port = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mgu1_stream_state(canzero_frame* frame, canzero_message_mgu1_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mgu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->command = (mgu_command)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mgu1_stream_power_estimation(canzero_frame* frame, canzero_message_mgu1_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current_starboard = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->dc_current_port = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mgu1_stream_sdc_status(canzero_frame* frame, canzero_message_mgu1_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mgu2_stream_errors(canzero_frame* frame, canzero_message_mgu2_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_error = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature_starboard = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature_port = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature_starboard = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature_port = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mgu2_stream_state(canzero_frame* frame, canzero_message_mgu2_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mgu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mgu2_stream_power_estimation(canzero_frame* frame, canzero_message_mgu2_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current_starboard = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->dc_current_port = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mgu2_stream_sdc_status(canzero_frame* frame, canzero_message_mgu2_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu1_stream_errors(canzero_frame* frame, canzero_message_mlu1_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_failure = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu1_stream_state(canzero_frame* frame, canzero_message_mlu1_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mlu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->command = (mlu_command)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mlu1_stream_power_estimation(canzero_frame* frame, canzero_message_mlu1_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mlu1_stream_sdc_status(canzero_frame* frame, canzero_message_mlu1_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu2_stream_errors(canzero_frame* frame, canzero_message_mlu2_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_failure = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu2_stream_state(canzero_frame* frame, canzero_message_mlu2_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mlu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->command = (mlu_command)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mlu2_stream_power_estimation(canzero_frame* frame, canzero_message_mlu2_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mlu2_stream_sdc_status(canzero_frame* frame, canzero_message_mlu2_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu3_stream_errors(canzero_frame* frame, canzero_message_mlu3_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_failure = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu3_stream_state(canzero_frame* frame, canzero_message_mlu3_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mlu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->command = (mlu_command)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mlu3_stream_power_estimation(canzero_frame* frame, canzero_message_mlu3_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mlu3_stream_sdc_status(canzero_frame* frame, canzero_message_mlu3_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu4_stream_errors(canzero_frame* frame, canzero_message_mlu4_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_failure = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu4_stream_state(canzero_frame* frame, canzero_message_mlu4_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mlu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->command = (mlu_command)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mlu4_stream_power_estimation(canzero_frame* frame, canzero_message_mlu4_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mlu4_stream_sdc_status(canzero_frame* frame, canzero_message_mlu4_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu5_stream_errors(canzero_frame* frame, canzero_message_mlu5_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_failure = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu5_stream_state(canzero_frame* frame, canzero_message_mlu5_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mlu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->command = (mlu_command)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mlu5_stream_power_estimation(canzero_frame* frame, canzero_message_mlu5_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mlu5_stream_sdc_status(canzero_frame* frame, canzero_message_mlu5_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu6_stream_errors(canzero_frame* frame, canzero_message_mlu6_stream_errors* msg) {
  uint8_t* data = frame->data;
  msg->error_45V_over_voltage = (error_flag)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->error_45V_under_voltage = (error_flag)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->error_control_failure = (error_flag)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_magnet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 6) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mcu_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->warn_mosfet_over_temperature = (error_flag)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_mlu6_stream_state(canzero_frame* frame, canzero_message_mlu6_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->state = (mlu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->command = (mlu_command)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 3)));
}
static void canzero_deserialize_canzero_message_mlu6_stream_power_estimation(canzero_frame* frame, canzero_message_mlu6_stream_power_estimation* msg) {
  uint8_t* data = frame->data;
  msg->dc_current = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0009155413138017853 + -10;
  msg->power_estimation = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0;
}
static void canzero_deserialize_canzero_message_mlu6_stream_sdc_status(canzero_frame* frame, canzero_message_mlu6_stream_sdc_status* msg) {
  uint8_t* data = frame->data;
  msg->sdc_status = (sdc_status)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
}
void canzero_can0_poll();
void canzero_can1_poll();
uint32_t canzero_update_continue(uint32_t delta_time);
void canzero_init();
void canzero_set_mlu_command(mlu_command value);
void canzero_set_mgu_command(mgu_command value);
void canzero_set_motor_command(motor_command value);
void canzero_set_pdu24_lp_channel1_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel2_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel3_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel4_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel5_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel6_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel7_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel8_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel9_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel10_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel11_control(pdu_channel_control value);
void canzero_set_pdu24_lp_channel12_control(pdu_channel_control value);
void canzero_set_pdu24_hp_channel1_control(pdu_channel_control value);
void canzero_set_pdu24_hp_channel2_control(pdu_channel_control value);
void canzero_set_pdu24_hp_channel3_control(pdu_channel_control value);
void canzero_set_pdu24_hp_channel4_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel1_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel2_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel3_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel4_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel5_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel6_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel7_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel8_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel9_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel10_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel11_control(pdu_channel_control value);
void canzero_set_pdu12_lp_channel12_control(pdu_channel_control value);
void canzero_set_pdu12_hp_channel1_control(pdu_channel_control value);
void canzero_set_pdu12_hp_channel2_control(pdu_channel_control value);
void canzero_set_pdu12_hp_channel3_control(pdu_channel_control value);
void canzero_set_pdu12_hp_channel4_control(pdu_channel_control value);
void canzero_set_global_state(global_state value);
void canzero_set_command(global_command value);
static inline void canzero_set_sync_mlu_pid_values(bool_t value){
  extern bool_t __oe_sync_mlu_pid_values;
  __oe_sync_mlu_pid_values = value;
}
void canzero_set_mlu_pid_values(mlu_pid_values value);
static inline void canzero_set_sync_mgu_pid_values(bool_t value){
  extern bool_t __oe_sync_mgu_pid_values;
  __oe_sync_mgu_pid_values = value;
}
static inline void canzero_set_mgu_pid_values(mgu_pid_values value){
  extern mgu_pid_values __oe_mgu_pid_values;
  __oe_mgu_pid_values = value;
}
static inline void canzero_set_cpu_temperature(float value){
  extern float __oe_cpu_temperature;
  __oe_cpu_temperature = value;
}
void canzero_set_sdc_status(sdc_status value);
static inline void canzero_set_power_estimation(float value){
  extern float __oe_power_estimation;
  __oe_power_estimation = value;
}
static inline void canzero_set_power_estimation24(float value){
  extern float __oe_power_estimation24;
  __oe_power_estimation24 = value;
}
static inline void canzero_set_power_estimation48(float value){
  extern float __oe_power_estimation48;
  __oe_power_estimation48 = value;
}
static inline void canzero_set_pdu12_state(pdu_state value){
  extern pdu_state __oe_pdu12_state;
  __oe_pdu12_state = value;
}
static inline void canzero_set_pdu24_state(pdu_state value){
  extern pdu_state __oe_pdu24_state;
  __oe_pdu24_state = value;
}
static inline void canzero_set_input_board_state(input_board_state value){
  extern input_board_state __oe_input_board_state;
  __oe_input_board_state = value;
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
static inline void canzero_set_error_input_board_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_input_board_mcu_over_temperature;
  __oe_error_input_board_mcu_over_temperature = value;
}
static inline void canzero_set_error_invalid_position_estimation(error_flag value){
  extern error_flag __oe_error_invalid_position_estimation;
  __oe_error_invalid_position_estimation = value;
}
static inline void canzero_set_error_invalid_position(error_flag value){
  extern error_flag __oe_error_invalid_position;
  __oe_error_invalid_position = value;
}
static inline void canzero_set_error_invalid_velocity_profile(error_flag value){
  extern error_flag __oe_error_invalid_velocity_profile;
  __oe_error_invalid_velocity_profile = value;
}
static inline void canzero_set_error_invalid_acceleration_profile(error_flag value){
  extern error_flag __oe_error_invalid_acceleration_profile;
  __oe_error_invalid_acceleration_profile = value;
}
static inline void canzero_set_error_battery_over_voltage(error_flag value){
  extern error_flag __oe_error_battery_over_voltage;
  __oe_error_battery_over_voltage = value;
}
static inline void canzero_set_error_cooling_cycle_over_pressure(error_flag value){
  extern error_flag __oe_error_cooling_cycle_over_pressure;
  __oe_error_cooling_cycle_over_pressure = value;
}
static inline void canzero_set_error_cooling_cycle_low_pressure(error_flag value){
  extern error_flag __oe_error_cooling_cycle_low_pressure;
  __oe_error_cooling_cycle_low_pressure = value;
}
static inline void canzero_set_error_ebox_over_temperature(error_flag value){
  extern error_flag __oe_error_ebox_over_temperature;
  __oe_error_ebox_over_temperature = value;
}
static inline void canzero_set_error_cooling_cycle_over_temperature(error_flag value){
  extern error_flag __oe_error_cooling_cycle_over_temperature;
  __oe_error_cooling_cycle_over_temperature = value;
}
static inline void canzero_set_warn_invalid_position_estimation(error_flag value){
  extern error_flag __oe_warn_invalid_position_estimation;
  __oe_warn_invalid_position_estimation = value;
}
static inline void canzero_set_warn_invalid_position(error_flag value){
  extern error_flag __oe_warn_invalid_position;
  __oe_warn_invalid_position = value;
}
static inline void canzero_set_warn_invalid_velocity_profile(error_flag value){
  extern error_flag __oe_warn_invalid_velocity_profile;
  __oe_warn_invalid_velocity_profile = value;
}
static inline void canzero_set_warn_invalid_acceleration_profile(error_flag value){
  extern error_flag __oe_warn_invalid_acceleration_profile;
  __oe_warn_invalid_acceleration_profile = value;
}
static inline void canzero_set_warn_battery_over_temperature(error_flag value){
  extern error_flag __oe_warn_battery_over_temperature;
  __oe_warn_battery_over_temperature = value;
}
static inline void canzero_set_warn_cooling_cycle_over_pressure(error_flag value){
  extern error_flag __oe_warn_cooling_cycle_over_pressure;
  __oe_warn_cooling_cycle_over_pressure = value;
}
static inline void canzero_set_warn_cooling_cycle_low_pressure(error_flag value){
  extern error_flag __oe_warn_cooling_cycle_low_pressure;
  __oe_warn_cooling_cycle_low_pressure = value;
}
static inline void canzero_set_warn_input_board_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_input_board_mcu_over_temperature;
  __oe_warn_input_board_mcu_over_temperature = value;
}
static inline void canzero_set_warn_ebox_over_temperature(error_flag value){
  extern error_flag __oe_warn_ebox_over_temperature;
  __oe_warn_ebox_over_temperature = value;
}
static inline void canzero_set_warn_cooling_cycle_over_temperature(error_flag value){
  extern error_flag __oe_warn_cooling_cycle_over_temperature;
  __oe_warn_cooling_cycle_over_temperature = value;
}
static inline void canzero_set_input_board_sdc_status(sdc_status value){
  extern sdc_status __oe_input_board_sdc_status;
  __oe_input_board_sdc_status = value;
}
static inline void canzero_set_pdu12_lp_channel1_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel1_status;
  __oe_pdu12_lp_channel1_status = value;
}
static inline void canzero_set_pdu12_lp_channel2_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel2_status;
  __oe_pdu12_lp_channel2_status = value;
}
static inline void canzero_set_pdu12_lp_channel3_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel3_status;
  __oe_pdu12_lp_channel3_status = value;
}
static inline void canzero_set_pdu12_lp_channel4_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel4_status;
  __oe_pdu12_lp_channel4_status = value;
}
static inline void canzero_set_pdu12_lp_channel5_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel5_status;
  __oe_pdu12_lp_channel5_status = value;
}
static inline void canzero_set_pdu12_lp_channel6_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel6_status;
  __oe_pdu12_lp_channel6_status = value;
}
static inline void canzero_set_pdu12_lp_channel7_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel7_status;
  __oe_pdu12_lp_channel7_status = value;
}
static inline void canzero_set_pdu12_lp_channel8_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel8_status;
  __oe_pdu12_lp_channel8_status = value;
}
static inline void canzero_set_pdu12_lp_chanel9_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_chanel9_status;
  __oe_pdu12_lp_chanel9_status = value;
}
static inline void canzero_set_pdu12_lp_channel10_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel10_status;
  __oe_pdu12_lp_channel10_status = value;
}
static inline void canzero_set_pdu12_lp_channel11_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_lp_channel11_status;
  __oe_pdu12_lp_channel11_status = value;
}
static inline void canzero_set_pdu12_lp_channel12_status(uint8_t value){
  extern uint8_t __oe_pdu12_lp_channel12_status;
  __oe_pdu12_lp_channel12_status = value;
}
static inline void canzero_set_pdu12_hp_channel1_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_hp_channel1_status;
  __oe_pdu12_hp_channel1_status = value;
}
static inline void canzero_set_pdu12_hp_channel2_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_hp_channel2_status;
  __oe_pdu12_hp_channel2_status = value;
}
static inline void canzero_set_pdu12_hp_channel3_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_hp_channel3_status;
  __oe_pdu12_hp_channel3_status = value;
}
static inline void canzero_set_pdu12_hp_channel4_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu12_hp_channel4_status;
  __oe_pdu12_hp_channel4_status = value;
}
static inline void canzero_set_pdu12_sdc_status(sdc_status value){
  extern sdc_status __oe_pdu12_sdc_status;
  __oe_pdu12_sdc_status = value;
}
static inline void canzero_set_error_pdu12_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_pdu12_mcu_over_temperature;
  __oe_error_pdu12_mcu_over_temperature = value;
}
static inline void canzero_set_pdu12_power_estimation(float value){
  extern float __oe_pdu12_power_estimation;
  __oe_pdu12_power_estimation = value;
}
static inline void canzero_set_pdu24_lp_channel1_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel1_status;
  __oe_pdu24_lp_channel1_status = value;
}
static inline void canzero_set_pdu24_lp_channel2_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel2_status;
  __oe_pdu24_lp_channel2_status = value;
}
static inline void canzero_set_pdu24_lp_channel3_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel3_status;
  __oe_pdu24_lp_channel3_status = value;
}
static inline void canzero_set_pdu24_lp_channel4_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel4_status;
  __oe_pdu24_lp_channel4_status = value;
}
static inline void canzero_set_pdu24_lp_channel5_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel5_status;
  __oe_pdu24_lp_channel5_status = value;
}
static inline void canzero_set_pdu24_lp_channel6_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel6_status;
  __oe_pdu24_lp_channel6_status = value;
}
static inline void canzero_set_pdu24_lp_channel7_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel7_status;
  __oe_pdu24_lp_channel7_status = value;
}
static inline void canzero_set_pdu24_lp_channel8_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel8_status;
  __oe_pdu24_lp_channel8_status = value;
}
static inline void canzero_set_pdu24_lp_chanel9_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_chanel9_status;
  __oe_pdu24_lp_chanel9_status = value;
}
static inline void canzero_set_pdu24_lp_channel10_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel10_status;
  __oe_pdu24_lp_channel10_status = value;
}
static inline void canzero_set_pdu24_lp_channel11_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_lp_channel11_status;
  __oe_pdu24_lp_channel11_status = value;
}
static inline void canzero_set_pdu24_lp_channel12_status(uint8_t value){
  extern uint8_t __oe_pdu24_lp_channel12_status;
  __oe_pdu24_lp_channel12_status = value;
}
static inline void canzero_set_pdu24_hp_channel1_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_hp_channel1_status;
  __oe_pdu24_hp_channel1_status = value;
}
static inline void canzero_set_pdu24_hp_channel2_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_hp_channel2_status;
  __oe_pdu24_hp_channel2_status = value;
}
static inline void canzero_set_pdu24_hp_channel3_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_hp_channel3_status;
  __oe_pdu24_hp_channel3_status = value;
}
static inline void canzero_set_pdu24_hp_channel4_status(pdu_channel_status value){
  extern pdu_channel_status __oe_pdu24_hp_channel4_status;
  __oe_pdu24_hp_channel4_status = value;
}
static inline void canzero_set_pdu24_sdc_status(sdc_status value){
  extern sdc_status __oe_pdu24_sdc_status;
  __oe_pdu24_sdc_status = value;
}
static inline void canzero_set_error_pdu24_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_pdu24_mcu_over_temperature;
  __oe_error_pdu24_mcu_over_temperature = value;
}
static inline void canzero_set_pdu24_power_estimation(float value){
  extern float __oe_pdu24_power_estimation;
  __oe_pdu24_power_estimation = value;
}
static inline void canzero_set_error_motor_driver_45V_over_voltage(error_flag value){
  extern error_flag __oe_error_motor_driver_45V_over_voltage;
  __oe_error_motor_driver_45V_over_voltage = value;
}
static inline void canzero_set_error_motor_driver_45V_under_voltage(error_flag value){
  extern error_flag __oe_error_motor_driver_45V_under_voltage;
  __oe_error_motor_driver_45V_under_voltage = value;
}
static inline void canzero_set_error_dslim_over_temperature(error_flag value){
  extern error_flag __oe_error_dslim_over_temperature;
  __oe_error_dslim_over_temperature = value;
}
static inline void canzero_set_error_motor_driver_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_error_motor_driver_mosfet_over_temperature;
  __oe_error_motor_driver_mosfet_over_temperature = value;
}
static inline void canzero_set_error_motor_control_failure(error_flag value){
  extern error_flag __oe_error_motor_control_failure;
  __oe_error_motor_control_failure = value;
}
static inline void canzero_set_error_motor_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_motor_mcu_over_temperature;
  __oe_error_motor_mcu_over_temperature = value;
}
static inline void canzero_set_warn_motor_dslim_over_temperature(error_flag value){
  extern error_flag __oe_warn_motor_dslim_over_temperature;
  __oe_warn_motor_dslim_over_temperature = value;
}
static inline void canzero_set_warn_motor_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_warn_motor_mosfet_over_temperature;
  __oe_warn_motor_mosfet_over_temperature = value;
}
static inline void canzero_set_warn_motor_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_motor_mcu_over_temperature;
  __oe_warn_motor_mcu_over_temperature = value;
}
static inline void canzero_set_motor_power_estimation(float value){
  extern float __oe_motor_power_estimation;
  __oe_motor_power_estimation = value;
}
static inline void canzero_set_motor_state(motor_state value){
  extern motor_state __oe_motor_state;
  __oe_motor_state = value;
}
static inline void canzero_set_motor_sdc_status(sdc_status value){
  extern sdc_status __oe_motor_sdc_status;
  __oe_motor_sdc_status = value;
}
static inline void canzero_set_error_mgu1_45V_over_voltage(error_flag value){
  extern error_flag __oe_error_mgu1_45V_over_voltage;
  __oe_error_mgu1_45V_over_voltage = value;
}
static inline void canzero_set_error_mgu1_45V_under_voltage(error_flag value){
  extern error_flag __oe_error_mgu1_45V_under_voltage;
  __oe_error_mgu1_45V_under_voltage = value;
}
static inline void canzero_set_error_mgu1_control_error(error_flag value){
  extern error_flag __oe_error_mgu1_control_error;
  __oe_error_mgu1_control_error = value;
}
static inline void canzero_set_error_mgu1_magnet_over_temperature_starboard(error_flag value){
  extern error_flag __oe_error_mgu1_magnet_over_temperature_starboard;
  __oe_error_mgu1_magnet_over_temperature_starboard = value;
}
static inline void canzero_set_error_mgu1_magnet_over_temperature_port(error_flag value){
  extern error_flag __oe_error_mgu1_magnet_over_temperature_port;
  __oe_error_mgu1_magnet_over_temperature_port = value;
}
static inline void canzero_set_error_mgu1_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_error_mgu1_mosfet_over_temperature;
  __oe_error_mgu1_mosfet_over_temperature = value;
}
static inline void canzero_set_error_mgu1_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_mgu1_mcu_over_temperature;
  __oe_error_mgu1_mcu_over_temperature = value;
}
static inline void canzero_set_warn_mgu1_magnet_over_temperature_starboard(error_flag value){
  extern error_flag __oe_warn_mgu1_magnet_over_temperature_starboard;
  __oe_warn_mgu1_magnet_over_temperature_starboard = value;
}
static inline void canzero_set_warn_mgu1_magnet_over_temperature_port(error_flag value){
  extern error_flag __oe_warn_mgu1_magnet_over_temperature_port;
  __oe_warn_mgu1_magnet_over_temperature_port = value;
}
static inline void canzero_set_warn_mgu1_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mgu1_mosfet_over_temperature;
  __oe_warn_mgu1_mosfet_over_temperature = value;
}
static inline void canzero_set_warn_mgu1_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_mgu1_mcu_over_temperature;
  __oe_warn_mgu1_mcu_over_temperature = value;
}
static inline void canzero_set_mgu1_state(mgu_state value){
  extern mgu_state __oe_mgu1_state;
  __oe_mgu1_state = value;
}
static inline void canzero_set_mgu1_power_estimation(float value){
  extern float __oe_mgu1_power_estimation;
  __oe_mgu1_power_estimation = value;
}
static inline void canzero_set_mgu1_sdc_status(sdc_status value){
  extern sdc_status __oe_mgu1_sdc_status;
  __oe_mgu1_sdc_status = value;
}
static inline void canzero_set_error_mgu2_45V_over_voltage(error_flag value){
  extern error_flag __oe_error_mgu2_45V_over_voltage;
  __oe_error_mgu2_45V_over_voltage = value;
}
static inline void canzero_set_error_mgu2_45V_under_voltage(error_flag value){
  extern error_flag __oe_error_mgu2_45V_under_voltage;
  __oe_error_mgu2_45V_under_voltage = value;
}
static inline void canzero_set_error_mgu2_control_error(error_flag value){
  extern error_flag __oe_error_mgu2_control_error;
  __oe_error_mgu2_control_error = value;
}
static inline void canzero_set_error_mgu2_magnet_over_temperature_starboard(error_flag value){
  extern error_flag __oe_error_mgu2_magnet_over_temperature_starboard;
  __oe_error_mgu2_magnet_over_temperature_starboard = value;
}
static inline void canzero_set_error_mgu2_magnet_over_temperature_port(error_flag value){
  extern error_flag __oe_error_mgu2_magnet_over_temperature_port;
  __oe_error_mgu2_magnet_over_temperature_port = value;
}
static inline void canzero_set_error_mgu2_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_error_mgu2_mosfet_over_temperature;
  __oe_error_mgu2_mosfet_over_temperature = value;
}
static inline void canzero_set_error_mgu2_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_mgu2_mcu_over_temperature;
  __oe_error_mgu2_mcu_over_temperature = value;
}
static inline void canzero_set_warn_mgu2_magnet_over_temperature_starboard(error_flag value){
  extern error_flag __oe_warn_mgu2_magnet_over_temperature_starboard;
  __oe_warn_mgu2_magnet_over_temperature_starboard = value;
}
static inline void canzero_set_warn_mgu2_magnet_over_temperature_port(error_flag value){
  extern error_flag __oe_warn_mgu2_magnet_over_temperature_port;
  __oe_warn_mgu2_magnet_over_temperature_port = value;
}
static inline void canzero_set_warn_mgu2_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mgu2_mosfet_over_temperature;
  __oe_warn_mgu2_mosfet_over_temperature = value;
}
static inline void canzero_set_warn_mgu2_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_mgu2_mcu_over_temperature;
  __oe_warn_mgu2_mcu_over_temperature = value;
}
static inline void canzero_set_mgu2_state(mgu_state value){
  extern mgu_state __oe_mgu2_state;
  __oe_mgu2_state = value;
}
static inline void canzero_set_mgu2_power_estimation(float value){
  extern float __oe_mgu2_power_estimation;
  __oe_mgu2_power_estimation = value;
}
static inline void canzero_set_mgu2_sdc_status(sdc_status value){
  extern sdc_status __oe_mgu2_sdc_status;
  __oe_mgu2_sdc_status = value;
}
static inline void canzero_set_error_mlu1_45V_over_voltage(error_flag value){
  extern error_flag __oe_error_mlu1_45V_over_voltage;
  __oe_error_mlu1_45V_over_voltage = value;
}
static inline void canzero_set_error_mlu1_45V_under_voltage(error_flag value){
  extern error_flag __oe_error_mlu1_45V_under_voltage;
  __oe_error_mlu1_45V_under_voltage = value;
}
static inline void canzero_set_error_mlu1_magnet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu1_magnet_over_temperature;
  __oe_error_mlu1_magnet_over_temperature = value;
}
static inline void canzero_set_error_mlu1_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu1_mcu_over_temperature;
  __oe_error_mlu1_mcu_over_temperature = value;
}
static inline void canzero_set_error_mlu1_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu1_mosfet_over_temperature;
  __oe_error_mlu1_mosfet_over_temperature = value;
}
static inline void canzero_set_error_mlu1_control_failure(error_flag value){
  extern error_flag __oe_error_mlu1_control_failure;
  __oe_error_mlu1_control_failure = value;
}
static inline void canzero_set_warn_mlu1_magnet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu1_magnet_over_temperature;
  __oe_warn_mlu1_magnet_over_temperature = value;
}
static inline void canzero_set_warn_mlu1_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu1_mcu_over_temperature;
  __oe_warn_mlu1_mcu_over_temperature = value;
}
static inline void canzero_set_warn_mlu1_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu1_mosfet_over_temperature;
  __oe_warn_mlu1_mosfet_over_temperature = value;
}
static inline void canzero_set_mlu1_state(mlu_state value){
  extern mlu_state __oe_mlu1_state;
  __oe_mlu1_state = value;
}
static inline void canzero_set_mlu1_power_estimation(float value){
  extern float __oe_mlu1_power_estimation;
  __oe_mlu1_power_estimation = value;
}
static inline void canzero_set_mlu1_sdc_status(sdc_status value){
  extern sdc_status __oe_mlu1_sdc_status;
  __oe_mlu1_sdc_status = value;
}
static inline void canzero_set_error_mlu2_45V_over_voltage(error_flag value){
  extern error_flag __oe_error_mlu2_45V_over_voltage;
  __oe_error_mlu2_45V_over_voltage = value;
}
static inline void canzero_set_error_mlu2_45V_under_voltage(error_flag value){
  extern error_flag __oe_error_mlu2_45V_under_voltage;
  __oe_error_mlu2_45V_under_voltage = value;
}
static inline void canzero_set_error_mlu2_magnet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu2_magnet_over_temperature;
  __oe_error_mlu2_magnet_over_temperature = value;
}
static inline void canzero_set_error_mlu2_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu2_mcu_over_temperature;
  __oe_error_mlu2_mcu_over_temperature = value;
}
static inline void canzero_set_error_mlu2_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu2_mosfet_over_temperature;
  __oe_error_mlu2_mosfet_over_temperature = value;
}
static inline void canzero_set_error_mlu2_control_failure(error_flag value){
  extern error_flag __oe_error_mlu2_control_failure;
  __oe_error_mlu2_control_failure = value;
}
static inline void canzero_set_warn_mlu2_magnet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu2_magnet_over_temperature;
  __oe_warn_mlu2_magnet_over_temperature = value;
}
static inline void canzero_set_warn_mlu2_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu2_mcu_over_temperature;
  __oe_warn_mlu2_mcu_over_temperature = value;
}
static inline void canzero_set_warn_mlu2_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu2_mosfet_over_temperature;
  __oe_warn_mlu2_mosfet_over_temperature = value;
}
static inline void canzero_set_mlu2_state(mlu_state value){
  extern mlu_state __oe_mlu2_state;
  __oe_mlu2_state = value;
}
static inline void canzero_set_mlu2_power_estimation(float value){
  extern float __oe_mlu2_power_estimation;
  __oe_mlu2_power_estimation = value;
}
static inline void canzero_set_mlu2_sdc_status(sdc_status value){
  extern sdc_status __oe_mlu2_sdc_status;
  __oe_mlu2_sdc_status = value;
}
static inline void canzero_set_error_mlu3_45V_over_voltage(error_flag value){
  extern error_flag __oe_error_mlu3_45V_over_voltage;
  __oe_error_mlu3_45V_over_voltage = value;
}
static inline void canzero_set_error_mlu3_45V_under_voltage(error_flag value){
  extern error_flag __oe_error_mlu3_45V_under_voltage;
  __oe_error_mlu3_45V_under_voltage = value;
}
static inline void canzero_set_error_mlu3_magnet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu3_magnet_over_temperature;
  __oe_error_mlu3_magnet_over_temperature = value;
}
static inline void canzero_set_error_mlu3_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu3_mcu_over_temperature;
  __oe_error_mlu3_mcu_over_temperature = value;
}
static inline void canzero_set_error_mlu3_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu3_mosfet_over_temperature;
  __oe_error_mlu3_mosfet_over_temperature = value;
}
static inline void canzero_set_error_mlu3_control_failure(error_flag value){
  extern error_flag __oe_error_mlu3_control_failure;
  __oe_error_mlu3_control_failure = value;
}
static inline void canzero_set_warn_mlu3_magnet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu3_magnet_over_temperature;
  __oe_warn_mlu3_magnet_over_temperature = value;
}
static inline void canzero_set_warn_mlu3_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu3_mcu_over_temperature;
  __oe_warn_mlu3_mcu_over_temperature = value;
}
static inline void canzero_set_warn_mlu3_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu3_mosfet_over_temperature;
  __oe_warn_mlu3_mosfet_over_temperature = value;
}
static inline void canzero_set_mlu3_state(mlu_state value){
  extern mlu_state __oe_mlu3_state;
  __oe_mlu3_state = value;
}
static inline void canzero_set_mlu3_power_estimation(float value){
  extern float __oe_mlu3_power_estimation;
  __oe_mlu3_power_estimation = value;
}
static inline void canzero_set_mlu3_sdc_status(sdc_status value){
  extern sdc_status __oe_mlu3_sdc_status;
  __oe_mlu3_sdc_status = value;
}
static inline void canzero_set_error_mlu4_45V_over_voltage(error_flag value){
  extern error_flag __oe_error_mlu4_45V_over_voltage;
  __oe_error_mlu4_45V_over_voltage = value;
}
static inline void canzero_set_error_mlu4_45V_under_voltage(error_flag value){
  extern error_flag __oe_error_mlu4_45V_under_voltage;
  __oe_error_mlu4_45V_under_voltage = value;
}
static inline void canzero_set_error_mlu4_magnet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu4_magnet_over_temperature;
  __oe_error_mlu4_magnet_over_temperature = value;
}
static inline void canzero_set_error_mlu4_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu4_mcu_over_temperature;
  __oe_error_mlu4_mcu_over_temperature = value;
}
static inline void canzero_set_error_mlu4_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu4_mosfet_over_temperature;
  __oe_error_mlu4_mosfet_over_temperature = value;
}
static inline void canzero_set_error_mlu4_control_failure(error_flag value){
  extern error_flag __oe_error_mlu4_control_failure;
  __oe_error_mlu4_control_failure = value;
}
static inline void canzero_set_warn_mlu4_magnet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu4_magnet_over_temperature;
  __oe_warn_mlu4_magnet_over_temperature = value;
}
static inline void canzero_set_warn_mlu4_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu4_mcu_over_temperature;
  __oe_warn_mlu4_mcu_over_temperature = value;
}
static inline void canzero_set_warn_mlu4_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu4_mosfet_over_temperature;
  __oe_warn_mlu4_mosfet_over_temperature = value;
}
static inline void canzero_set_mlu4_state(mlu_state value){
  extern mlu_state __oe_mlu4_state;
  __oe_mlu4_state = value;
}
static inline void canzero_set_mlu4_power_estimation(float value){
  extern float __oe_mlu4_power_estimation;
  __oe_mlu4_power_estimation = value;
}
static inline void canzero_set_mlu4_sdc_status(sdc_status value){
  extern sdc_status __oe_mlu4_sdc_status;
  __oe_mlu4_sdc_status = value;
}
static inline void canzero_set_error_mlu5_45V_over_voltage(error_flag value){
  extern error_flag __oe_error_mlu5_45V_over_voltage;
  __oe_error_mlu5_45V_over_voltage = value;
}
static inline void canzero_set_error_mlu5_45V_under_voltage(error_flag value){
  extern error_flag __oe_error_mlu5_45V_under_voltage;
  __oe_error_mlu5_45V_under_voltage = value;
}
static inline void canzero_set_error_mlu5_magnet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu5_magnet_over_temperature;
  __oe_error_mlu5_magnet_over_temperature = value;
}
static inline void canzero_set_error_mlu5_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu5_mcu_over_temperature;
  __oe_error_mlu5_mcu_over_temperature = value;
}
static inline void canzero_set_error_mlu5_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu5_mosfet_over_temperature;
  __oe_error_mlu5_mosfet_over_temperature = value;
}
static inline void canzero_set_error_mlu5_control_failure(error_flag value){
  extern error_flag __oe_error_mlu5_control_failure;
  __oe_error_mlu5_control_failure = value;
}
static inline void canzero_set_warn_mlu5_magnet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu5_magnet_over_temperature;
  __oe_warn_mlu5_magnet_over_temperature = value;
}
static inline void canzero_set_warn_mlu5_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu5_mcu_over_temperature;
  __oe_warn_mlu5_mcu_over_temperature = value;
}
static inline void canzero_set_warn_mlu5_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu5_mosfet_over_temperature;
  __oe_warn_mlu5_mosfet_over_temperature = value;
}
static inline void canzero_set_mlu5_state(mlu_state value){
  extern mlu_state __oe_mlu5_state;
  __oe_mlu5_state = value;
}
static inline void canzero_set_mlu5_power_estimation(float value){
  extern float __oe_mlu5_power_estimation;
  __oe_mlu5_power_estimation = value;
}
static inline void canzero_set_mlu5_sdc_status(sdc_status value){
  extern sdc_status __oe_mlu5_sdc_status;
  __oe_mlu5_sdc_status = value;
}
static inline void canzero_set_error_mlu6_45V_over_voltage(error_flag value){
  extern error_flag __oe_error_mlu6_45V_over_voltage;
  __oe_error_mlu6_45V_over_voltage = value;
}
static inline void canzero_set_error_mlu6_45V_under_voltage(error_flag value){
  extern error_flag __oe_error_mlu6_45V_under_voltage;
  __oe_error_mlu6_45V_under_voltage = value;
}
static inline void canzero_set_error_mlu6_magnet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu6_magnet_over_temperature;
  __oe_error_mlu6_magnet_over_temperature = value;
}
static inline void canzero_set_error_mlu6_mcu_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu6_mcu_over_temperature;
  __oe_error_mlu6_mcu_over_temperature = value;
}
static inline void canzero_set_error_mlu6_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_error_mlu6_mosfet_over_temperature;
  __oe_error_mlu6_mosfet_over_temperature = value;
}
static inline void canzero_set_error_mlu6_control_failure(error_flag value){
  extern error_flag __oe_error_mlu6_control_failure;
  __oe_error_mlu6_control_failure = value;
}
static inline void canzero_set_warn_mlu6_magnet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu6_magnet_over_temperature;
  __oe_warn_mlu6_magnet_over_temperature = value;
}
static inline void canzero_set_warn_mlu6_mcu_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu6_mcu_over_temperature;
  __oe_warn_mlu6_mcu_over_temperature = value;
}
static inline void canzero_set_warn_mlu6_mosfet_over_temperature(error_flag value){
  extern error_flag __oe_warn_mlu6_mosfet_over_temperature;
  __oe_warn_mlu6_mosfet_over_temperature = value;
}
static inline void canzero_set_mlu6_state(mlu_state value){
  extern mlu_state __oe_mlu6_state;
  __oe_mlu6_state = value;
}
static inline void canzero_set_mlu6_power_estimation(float value){
  extern float __oe_mlu6_power_estimation;
  __oe_mlu6_power_estimation = value;
}
static inline void canzero_set_mlu6_sdc_status(sdc_status value){
  extern sdc_status __oe_mlu6_sdc_status;
  __oe_mlu6_sdc_status = value;
}
#endif