#ifndef CANZERO_H
#define CANZERO_H
#include <cinttypes>
#include <cstddef>
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
  node_id_count = 11,
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
  pdu_state_INIT = 0,
  pdu_state_RUNNING = 1,
} pdu_state;
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
static inline guidance_state canzero_get_guidance_board_back_state() {
  extern guidance_state __oe_guidance_board_back_state;
  return __oe_guidance_board_back_state;
}
static inline sdc_status canzero_get_guidance_board_back_sdc_status() {
  extern sdc_status __oe_guidance_board_back_sdc_status;
  return __oe_guidance_board_back_sdc_status;
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
static inline levitation_state canzero_get_levitation_board2_state() {
  extern levitation_state __oe_levitation_board2_state;
  return __oe_levitation_board2_state;
}
static inline sdc_status canzero_get_levitation_board2_sdc_status() {
  extern sdc_status __oe_levitation_board2_sdc_status;
  return __oe_levitation_board2_sdc_status;
}
static inline levitation_state canzero_get_levitation_board3_state() {
  extern levitation_state __oe_levitation_board3_state;
  return __oe_levitation_board3_state;
}
static inline sdc_status canzero_get_levitation_board3_sdc_status() {
  extern sdc_status __oe_levitation_board3_sdc_status;
  return __oe_levitation_board3_sdc_status;
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
static inline pdu_state canzero_get_power_board12_state() {
  extern pdu_state __oe_power_board12_state;
  return __oe_power_board12_state;
}
static inline sdc_status canzero_get_power_board12_sdc_status() {
  extern sdc_status __oe_power_board12_sdc_status;
  return __oe_power_board12_sdc_status;
}
static inline pdu_state canzero_get_power_board24_state() {
  extern pdu_state __oe_power_board24_state;
  return __oe_power_board24_state;
}
static inline sdc_status canzero_get_power_board24_sdc_status() {
  extern sdc_status __oe_power_board24_sdc_status;
  return __oe_power_board24_sdc_status;
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
typedef struct {
  get_resp_header m_header;
  uint32_t m_data;
} canzero_message_get_resp;
static const uint32_t canzero_message_get_resp_id = 0xBD;
typedef struct {
  set_resp_header m_header;
} canzero_message_set_resp;
static const uint32_t canzero_message_set_resp_id = 0xDD;
typedef struct {
  global_state m_state;
  global_command m_command;
} canzero_message_mother_board_stream_state;
static const uint32_t canzero_message_mother_board_stream_state_id = 0x90;
typedef struct {
  float m_target_acceleration;
  motor_command m_motor_driver_command;
} canzero_message_mother_board_stream_motor_command;
static const uint32_t canzero_message_mother_board_stream_motor_command_id = 0x41;
typedef struct {
  input_board_command m_input_board_command;
  bool_t m_input_board_assert_45V_online;
} canzero_message_mother_board_stream_input_board_command;
static const uint32_t canzero_message_mother_board_stream_input_board_command_id = 0x43;
typedef struct {
  guidance_command m_guidance_command;
} canzero_message_mother_board_stream_guidance_command;
static const uint32_t canzero_message_mother_board_stream_guidance_command_id = 0x70;
typedef struct {
  levitation_command m_levitation_command;
} canzero_message_mother_board_stream_levitation_command;
static const uint32_t canzero_message_mother_board_stream_levitation_command_id = 0x42;
typedef struct {
  uint8_t m_node_id;
  uint8_t m_unregister;
  uint8_t m_ticks_next;
} canzero_message_heartbeat_can0;
static const uint32_t canzero_message_heartbeat_can0_id = 0xE5;
typedef struct {
  uint8_t m_node_id;
  uint8_t m_unregister;
  uint8_t m_ticks_next;
} canzero_message_heartbeat_can1;
static const uint32_t canzero_message_heartbeat_can1_id = 0xE4;
typedef struct {
  get_req_header m_header;
} canzero_message_get_req;
static const uint32_t canzero_message_get_req_id = 0xBE;
typedef struct {
  set_req_header m_header;
  uint32_t m_data;
} canzero_message_set_req;
static const uint32_t canzero_message_set_req_id = 0xDE;
typedef struct {
  motor_state m_state;
  sdc_status m_sdc_status;
  motor_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_motor_driver_stream_state;
static const uint32_t canzero_message_motor_driver_stream_state_id = 0x67;
typedef struct {
  guidance_state m_state;
  sdc_status m_sdc_status;
  guidance_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_guidance_board_front_stream_state;
static const uint32_t canzero_message_guidance_board_front_stream_state_id = 0x69;
typedef struct {
  guidance_state m_state;
  sdc_status m_sdc_status;
  guidance_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_guidance_board_back_stream_state;
static const uint32_t canzero_message_guidance_board_back_stream_state_id = 0x49;
typedef struct {
  levitation_state m_state;
  sdc_status m_sdc_status;
  levitation_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_levitation_board1_stream_state;
static const uint32_t canzero_message_levitation_board1_stream_state_id = 0x68;
typedef struct {
  levitation_state m_state;
  sdc_status m_sdc_status;
  levitation_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_levitation_board2_stream_state;
static const uint32_t canzero_message_levitation_board2_stream_state_id = 0x88;
typedef struct {
  levitation_state m_state;
  sdc_status m_sdc_status;
  levitation_command m_command;
  bool_t m_control_active;
  sdc_status m_precharge_status;
  sdc_status m_feedthrough_status;
} canzero_message_levitation_board3_stream_state;
static const uint32_t canzero_message_levitation_board3_stream_state_id = 0x47;
typedef struct {
  input_board_state m_state;
  sdc_status m_sdc_status;
} canzero_message_input_board_stream_state;
static const uint32_t canzero_message_input_board_stream_state_id = 0x48;
typedef struct {
  float m_position;
  float m_velocity;
  float m_acceleration;
} canzero_message_input_board_stream_position_estimation;
static const uint32_t canzero_message_input_board_stream_position_estimation_id = 0x89;
typedef struct {
  pdu_state m_state;
  sdc_status m_sdc_status;
} canzero_message_power_board12_stream_state;
static const uint32_t canzero_message_power_board12_stream_state_id = 0x87;
typedef struct {
  pdu_state m_state;
  sdc_status m_sdc_status;
} canzero_message_power_board24_stream_state;
static const uint32_t canzero_message_power_board24_stream_state_id = 0x46;
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

static inline void canzero_set_motor_driver_state(motor_state value){
  extern motor_state __oe_motor_driver_state;
  __oe_motor_driver_state = value;
}

void canzero_set_motor_driver_command(motor_command value);

static inline void canzero_set_motor_driver_sdc_status(sdc_status value){
  extern sdc_status __oe_motor_driver_sdc_status;
  __oe_motor_driver_sdc_status = value;
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

static inline void canzero_set_guidance_board_back_state(guidance_state value){
  extern guidance_state __oe_guidance_board_back_state;
  __oe_guidance_board_back_state = value;
}

static inline void canzero_set_guidance_board_back_sdc_status(sdc_status value){
  extern sdc_status __oe_guidance_board_back_sdc_status;
  __oe_guidance_board_back_sdc_status = value;
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

static inline void canzero_set_levitation_board2_state(levitation_state value){
  extern levitation_state __oe_levitation_board2_state;
  __oe_levitation_board2_state = value;
}

static inline void canzero_set_levitation_board2_sdc_status(sdc_status value){
  extern sdc_status __oe_levitation_board2_sdc_status;
  __oe_levitation_board2_sdc_status = value;
}

static inline void canzero_set_levitation_board3_state(levitation_state value){
  extern levitation_state __oe_levitation_board3_state;
  __oe_levitation_board3_state = value;
}

static inline void canzero_set_levitation_board3_sdc_status(sdc_status value){
  extern sdc_status __oe_levitation_board3_sdc_status;
  __oe_levitation_board3_sdc_status = value;
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

static inline void canzero_set_power_board12_state(pdu_state value){
  extern pdu_state __oe_power_board12_state;
  __oe_power_board12_state = value;
}

static inline void canzero_set_power_board12_sdc_status(sdc_status value){
  extern sdc_status __oe_power_board12_sdc_status;
  __oe_power_board12_sdc_status = value;
}

static inline void canzero_set_power_board24_state(pdu_state value){
  extern pdu_state __oe_power_board24_state;
  __oe_power_board24_state = value;
}

static inline void canzero_set_power_board24_sdc_status(sdc_status value){
  extern sdc_status __oe_power_board24_sdc_status;
  __oe_power_board24_sdc_status = value;
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

void canzero_send_config_hash();

void canzero_send_build_time();

void canzero_send_state();

void canzero_send_command();

void canzero_send_track_length();

void canzero_send_brake_margin();

void canzero_send_emergency_brake_margin();

void canzero_send_target_acceleration();

void canzero_send_acceleration_target_velocity();

void canzero_send_velocity_pid();

void canzero_send_position();

void canzero_send_velocity();

void canzero_send_acceleration();

void canzero_send_motor_driver_state();

void canzero_send_motor_driver_command();

void canzero_send_motor_driver_sdc_status();

void canzero_send_guidance_command();

void canzero_send_guidance_board_front_state();

void canzero_send_guidance_board_front_sdc_status();

void canzero_send_guidance_board_back_state();

void canzero_send_guidance_board_back_sdc_status();

void canzero_send_levitation_command();

void canzero_send_levitation_board1_state();

void canzero_send_levitation_board1_sdc_status();

void canzero_send_levitation_board2_state();

void canzero_send_levitation_board2_sdc_status();

void canzero_send_levitation_board3_state();

void canzero_send_levitation_board3_sdc_status();

void canzero_send_input_board_state();

void canzero_send_input_board_command();

void canzero_send_input_board_assert_45V_online();

void canzero_send_input_board_sdc_status();

void canzero_send_power_board12_state();

void canzero_send_power_board12_sdc_status();

void canzero_send_power_board24_state();

void canzero_send_power_board24_sdc_status();

void canzero_send_gamepad_max_acceleration();

void canzero_send_gamepad_lt2();

void canzero_send_gamepad_rt2();

#endif