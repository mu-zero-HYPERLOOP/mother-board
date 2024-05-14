#ifndef CANZERO_H
#define CANZERO_H
#include <cinttypes>
#include <cstddef>
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
  motor_state_PRECHARGE = 2,
  motor_state_READY = 3,
  motor_state_START = 4,
  motor_state_CONTROL = 5,
  motor_state_STOP = 6,
  motor_state_MANUAL = 7,
  motor_state_ERROR = 8,
} motor_state;
typedef enum {
  sdc_status_OPEN = 0,
  sdc_status_CLOSED = 1,
} sdc_status;
typedef enum {
  guidance_state_INIT = 0,
  guidance_state_IDLE = 1,
  guidance_state_PRECHARGE = 2,
  guidance_state_READY = 3,
  guidance_state_START = 4,
  guidance_state_CONTROL = 5,
  guidance_state_STOP = 6,
} guidance_state;
typedef enum {
  levitation_state_INIT = 0,
  levitation_state_IDLE = 1,
  levitation_state_PRECHARGE = 2,
  levitation_state_READY = 3,
  levitation_state_START = 4,
  levitation_state_CONTROL = 5,
  levitation_state_STOP = 6,
} levitation_state;
typedef enum {
  input_board_state_INIT = 0,
  input_board_state_CALIBRATION = 1,
  input_board_state_RUNNING = 2,
} input_board_state;
typedef enum {
  pdu_state_INIT = 0,
  pdu_state_RUNNING = 1,
} pdu_state;
typedef enum {
  node_id_mother_board = 0,
  node_id_motor_driver = 1,
  node_id_guidance_board_front = 2,
  node_id_guidance_board_back = 3,
  node_id_levitation_board_front = 4,
  node_id_levitation_board_middle = 5,
  node_id_levitation_board_back = 6,
  node_id_input_board = 7,
  node_id_power_board12 = 8,
  node_id_power_board24 = 9,
  node_id_test_node = 10,
  node_id_count = 11
} node_id;
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
  motor_command_NONE = 0,
  motor_command_PRECHARGE = 1,
  motor_command_ACCELERATE = 2,
  motor_command_DECELERATE = 3,
  motor_command_STOP = 4,
  motor_command_DISCONNECT = 5,
  motor_command_MANUAL_CONTROL = 6,
} motor_command;
typedef enum {
  guidance_command_NONE = 0,
  guidance_command_PRECHARGE = 1,
  guidance_command_START = 2,
  guidance_command_STOP = 3,
  guidance_command_DISCONNECT = 4,
} guidance_command;
typedef enum {
  levitation_command_NONE = 0,
  levitation_command_PRECHARGE = 1,
  levitation_command_START = 2,
  levitation_command_STOP = 3,
  levitation_command_ABORT = 4,
  levitation_command_DISCONNECT = 5,
} levitation_command;
typedef struct {
  uint16_t m_year;
  uint8_t m_month;
  uint8_t m_day;
  uint8_t m_hour;
  uint8_t m_min;
  uint8_t m_sec;
} date_time;
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
static inline float canzero_get_target_velocity() {
  extern float __oe_target_velocity;
  return __oe_target_velocity;
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
static inline levitation_state canzero_get_levitation_board_front_state() {
  extern levitation_state __oe_levitation_board_front_state;
  return __oe_levitation_board_front_state;
}
static inline sdc_status canzero_get_levitation_board_front_sdc_status() {
  extern sdc_status __oe_levitation_board_front_sdc_status;
  return __oe_levitation_board_front_sdc_status;
}
static inline levitation_state canzero_get_levitation_board_middle_state() {
  extern levitation_state __oe_levitation_board_middle_state;
  return __oe_levitation_board_middle_state;
}
static inline sdc_status canzero_get_levitation_board_middle_sdc_status() {
  extern sdc_status __oe_levitation_board_middle_sdc_status;
  return __oe_levitation_board_middle_sdc_status;
}
static inline levitation_state canzero_get_levitation_board_back_state() {
  extern levitation_state __oe_levitation_board_back_state;
  return __oe_levitation_board_back_state;
}
static inline sdc_status canzero_get_levitation_board_back_sdc_status() {
  extern sdc_status __oe_levitation_board_back_sdc_status;
  return __oe_levitation_board_back_sdc_status;
}
static inline input_board_state canzero_get_input_board_state() {
  extern input_board_state __oe_input_board_state;
  return __oe_input_board_state;
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
typedef struct {
  get_resp_header m_header;
  uint32_t m_data;
} canzero_message_get_resp;
static const uint32_t canzero_message_get_resp_id = 0x9E;
typedef struct {
  set_resp_header m_header;
} canzero_message_set_resp;
static const uint32_t canzero_message_set_resp_id = 0xBE;
typedef struct {
  global_state m_state;
} canzero_message_mother_board_stream_state;
static const uint32_t canzero_message_mother_board_stream_state_id = 0x73;
typedef struct {
  motor_command m_motor_driver_command;
} canzero_message_mother_board_stream_motor_command;
static const uint32_t canzero_message_mother_board_stream_motor_command_id = 0x4C;
typedef struct {
  guidance_command m_guidance_command;
} canzero_message_mother_board_stream_guidance_command;
static const uint32_t canzero_message_mother_board_stream_guidance_command_id = 0x74;
typedef struct {
  levitation_command m_levitation_command;
} canzero_message_mother_board_stream_levitation_command;
static const uint32_t canzero_message_mother_board_stream_levitation_command_id = 0x53;
typedef struct {
  node_id m_node_id;
} canzero_message_heartbeat;
static const uint32_t canzero_message_heartbeat_id = 0xDF;
typedef struct {
  get_req_header m_header;
} canzero_message_get_req;
static const uint32_t canzero_message_get_req_id = 0x9F;
typedef struct {
  set_req_header m_header;
  uint32_t m_data;
} canzero_message_set_req;
static const uint32_t canzero_message_set_req_id = 0xBF;
typedef struct {
  motor_state m_state;
  sdc_status m_sdc_status;
} canzero_message_motor_driver_stream_state;
static const uint32_t canzero_message_motor_driver_stream_state_id = 0x6E;
typedef struct {
  guidance_state m_state;
  sdc_status m_sdc_status;
} canzero_message_guidance_board_front_stream_state;
static const uint32_t canzero_message_guidance_board_front_stream_state_id = 0x71;
typedef struct {
  guidance_state m_state;
  sdc_status m_sdc_status;
} canzero_message_guidance_board_back_stream_state;
static const uint32_t canzero_message_guidance_board_back_stream_state_id = 0x51;
typedef struct {
  levitation_state m_state;
  sdc_status m_sdc_status;
} canzero_message_levitation_board_front_stream_state;
static const uint32_t canzero_message_levitation_board_front_stream_state_id = 0x6F;
typedef struct {
  levitation_state m_state;
  sdc_status m_sdc_status;
} canzero_message_levitation_board_middle_stream_state;
static const uint32_t canzero_message_levitation_board_middle_stream_state_id = 0x4E;
typedef struct {
  levitation_state m_state;
  sdc_status m_sdc_status;
} canzero_message_levitation_board_back_stream_state;
static const uint32_t canzero_message_levitation_board_back_stream_state_id = 0x4F;
typedef struct {
  input_board_state m_state;
  sdc_status m_sdc_status;
} canzero_message_input_board_stream_state;
static const uint32_t canzero_message_input_board_stream_state_id = 0x70;
typedef struct {
  float m_position;
  float m_velocity;
  float m_acceleration;
  uint16_t m_linear_encoder_count;
} canzero_message_input_board_stream_position_estimation;
static const uint32_t canzero_message_input_board_stream_position_estimation_id = 0x50;
typedef struct {
  pdu_state m_state;
  sdc_status m_sdc_status;
} canzero_message_power_board12_stream_state;
static const uint32_t canzero_message_power_board12_stream_state_id = 0x4D;
typedef struct {
  pdu_state m_state;
  sdc_status m_sdc_status;
} canzero_message_power_board24_stream_state;
static const uint32_t canzero_message_power_board24_stream_state_id = 0x6D;
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
static inline void canzero_set_command(global_command value){
  extern global_command __oe_command;
  __oe_command = value;
}
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
static inline void canzero_set_target_acceleration(float value){
  extern float __oe_target_acceleration;
  __oe_target_acceleration = value;
}
static inline void canzero_set_target_velocity(float value){
  extern float __oe_target_velocity;
  __oe_target_velocity = value;
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
static inline void canzero_set_levitation_board_front_state(levitation_state value){
  extern levitation_state __oe_levitation_board_front_state;
  __oe_levitation_board_front_state = value;
}
static inline void canzero_set_levitation_board_front_sdc_status(sdc_status value){
  extern sdc_status __oe_levitation_board_front_sdc_status;
  __oe_levitation_board_front_sdc_status = value;
}
static inline void canzero_set_levitation_board_middle_state(levitation_state value){
  extern levitation_state __oe_levitation_board_middle_state;
  __oe_levitation_board_middle_state = value;
}
static inline void canzero_set_levitation_board_middle_sdc_status(sdc_status value){
  extern sdc_status __oe_levitation_board_middle_sdc_status;
  __oe_levitation_board_middle_sdc_status = value;
}
static inline void canzero_set_levitation_board_back_state(levitation_state value){
  extern levitation_state __oe_levitation_board_back_state;
  __oe_levitation_board_back_state = value;
}
static inline void canzero_set_levitation_board_back_sdc_status(sdc_status value){
  extern sdc_status __oe_levitation_board_back_sdc_status;
  __oe_levitation_board_back_sdc_status = value;
}
static inline void canzero_set_input_board_state(input_board_state value){
  extern input_board_state __oe_input_board_state;
  __oe_input_board_state = value;
}
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
#endif