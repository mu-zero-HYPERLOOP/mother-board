#include "canzero.h"
#include <avr/pgmspace.h>
uint32_t min_u32(uint32_t v, uint32_t max) {
    if (v > max) {
        return max;
    }
    return v;
}
uint64_t min_u64(uint64_t v, uint64_t max) {
    if (v > max) {
        return max;
    }
    return v;
}
uint64_t DMAMEM __oe_config_hash;
date_time DMAMEM __oe_build_time;
global_state DMAMEM __oe_state;
global_command DMAMEM __oe_command;
float DMAMEM __oe_track_length;
float DMAMEM __oe_brake_margin;
float DMAMEM __oe_emergency_brake_margin;
float DMAMEM __oe_target_acceleration;
float DMAMEM __oe_acceleration_target_velocity;
pid_parameters DMAMEM __oe_velocity_pid;
float DMAMEM __oe_position;
float DMAMEM __oe_velocity;
float DMAMEM __oe_acceleration;
motor_state DMAMEM __oe_motor_driver_state;
motor_command DMAMEM __oe_motor_driver_command;
sdc_status DMAMEM __oe_motor_driver_sdc_status;
guidance_command DMAMEM __oe_guidance_command;
guidance_state DMAMEM __oe_guidance_board_front_state;
sdc_status DMAMEM __oe_guidance_board_front_sdc_status;
guidance_state DMAMEM __oe_guidance_board_back_state;
sdc_status DMAMEM __oe_guidance_board_back_sdc_status;
levitation_command DMAMEM __oe_levitation_command;
levitation_state DMAMEM __oe_levitation_board1_state;
sdc_status DMAMEM __oe_levitation_board1_sdc_status;
levitation_state DMAMEM __oe_levitation_board2_state;
sdc_status DMAMEM __oe_levitation_board2_sdc_status;
levitation_state DMAMEM __oe_levitation_board3_state;
sdc_status DMAMEM __oe_levitation_board3_sdc_status;
input_board_state DMAMEM __oe_input_board_state;
input_board_command DMAMEM __oe_input_board_command;
bool_t DMAMEM __oe_input_board_assert_45V_online;
sdc_status DMAMEM __oe_input_board_sdc_status;
pdu_12v_state DMAMEM __oe_power_board12_state;
pdu_12v_command DMAMEM __oe_power_board12_command;
sdc_status DMAMEM __oe_power_board12_sdc_status;
pdu_24v_state DMAMEM __oe_power_board24_state;
pdu_24v_command DMAMEM __oe_power_board24_command;
sdc_status DMAMEM __oe_power_board24_sdc_status;
float DMAMEM __oe_gamepad_max_acceleration;
float DMAMEM __oe_gamepad_lt2;
float DMAMEM __oe_gamepad_rt2;
error_flag DMAMEM __oe_error_heartbeat_miss;
static void canzero_serialize_canzero_message_get_resp(canzero_message_get_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x17D;
  frame->dlc = 8;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_header.m_sof & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_header.m_eof & (0xFF >> (8 - 1))) << 1;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_header.m_toggle & (0xFF >> (8 - 1))) << 2;
  ((uint32_t*)data)[0] |= (uint16_t)(msg->m_header.m_od_index & (0xFFFF >> (16 - 13))) << 3;
  ((uint32_t*)data)[0] |= msg->m_header.m_client_id << 16;
  ((uint32_t*)data)[0] |= msg->m_header.m_server_id << 24;
  ((uint32_t*)data)[1] = msg->m_data;
}
static void canzero_serialize_canzero_message_set_resp(canzero_message_set_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x1BD;
  frame->dlc = 4;
  ((uint32_t*)data)[0] = (uint16_t)(msg->m_header.m_od_index & (0xFFFF >> (16 - 13)));
  ((uint32_t*)data)[0] |= msg->m_header.m_client_id << 13;
  ((uint32_t*)data)[0] |= msg->m_header.m_server_id << 21;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_header.m_erno & (0xFF >> (8 - 1))) << 29;
}
static void canzero_serialize_canzero_message_mother_board_stream_state(canzero_message_mother_board_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x130;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_state & (0xFF >> (8 - 5)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_command & (0xFF >> (8 - 4))) << 5;
}
static void canzero_serialize_canzero_message_mother_board_stream_motor_command(canzero_message_mother_board_stream_motor_command* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0xA0;
  frame->dlc = 5;
  uint32_t target_acceleration_0 = ((msg->m_target_acceleration - -50) / 0.000000023283064370807974) + 0.5f;
  if (target_acceleration_0 > 0xFFFFFFFF) {
    target_acceleration_0 = 0xFFFFFFFF;
  }
  ((uint32_t*)data)[0] = target_acceleration_0;
  ((uint32_t*)data)[1] = (uint8_t)(msg->m_motor_driver_command & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_mother_board_stream_input_board_command(canzero_message_mother_board_stream_input_board_command* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0xA2;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_input_board_command & (0xFF >> (8 - 1)));
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_input_board_assert_45V_online & (0xFF >> (8 - 1))) << 1;
}
static void canzero_serialize_canzero_message_mother_board_stream_guidance_command(canzero_message_mother_board_stream_guidance_command* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0xA3;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_guidance_command & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_mother_board_stream_levitation_command(canzero_message_mother_board_stream_levitation_command* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0xA1;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_levitation_command & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_mother_board_stream_pdu_12v_command(canzero_message_mother_board_stream_pdu_12v_command* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x9F;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_power_board12_command & (0xFF >> (8 - 2)));
}
static void canzero_serialize_canzero_message_mother_board_stream_pdu_24v_command(canzero_message_mother_board_stream_pdu_24v_command* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x9E;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_power_board24_command & (0xFF >> (8 - 2)));
}
static void canzero_serialize_canzero_message_mother_board_stream_errors(canzero_message_mother_board_stream_errors* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0xF0;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_error_heartbeat_miss & (0xFF >> (8 - 1)));
}
static void canzero_serialize_canzero_message_heartbeat_can0(canzero_message_heartbeat_can0* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x1E5;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = msg->m_node_id;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_unregister & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_ticks_next & (0xFF >> (8 - 7))) << 9;
}
static void canzero_serialize_canzero_message_heartbeat_can1(canzero_message_heartbeat_can1* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  for(uint8_t i = 0; i < 8; ++i){
    data[i] = 0;
  }
  frame->id = 0x1E4;
  frame->dlc = 2;
  ((uint32_t*)data)[0] = msg->m_node_id;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_unregister & (0xFF >> (8 - 1))) << 8;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_ticks_next & (0xFF >> (8 - 7))) << 9;
}
static void canzero_deserialize_canzero_message_get_req(canzero_frame* frame, canzero_message_get_req* msg) {
  uint8_t* data = frame->data;
  msg->m_header.m_od_index = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 13)));
  msg->m_header.m_client_id = ((((uint32_t*)data)[0] >> 13) & (0xFFFFFFFF >> (32 - 8)));
  msg->m_header.m_server_id = ((((uint32_t*)data)[0] >> 21) & (0xFFFFFFFF >> (32 - 8)));
}
static void canzero_deserialize_canzero_message_set_req(canzero_frame* frame, canzero_message_set_req* msg) {
  uint8_t* data = frame->data;
  msg->m_header.m_sof = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->m_header.m_eof = ((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_header.m_toggle = ((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_header.m_od_index = ((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 13)));
  msg->m_header.m_client_id = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8)));
  msg->m_header.m_server_id = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8)));
  msg->m_data = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 32)));
}
static void canzero_deserialize_canzero_message_motor_driver_stream_state(canzero_frame* frame, canzero_message_motor_driver_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (motor_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_command = (motor_command)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 3)));
  msg->m_control_active = (bool_t)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_precharge_status = (sdc_status)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_feedthrough_status = (sdc_status)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_guidance_board_front_stream_state(canzero_frame* frame, canzero_message_guidance_board_front_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (guidance_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_command = (guidance_command)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 3)));
  msg->m_control_active = (bool_t)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_precharge_status = (sdc_status)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_feedthrough_status = (sdc_status)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_guidance_board_back_stream_state(canzero_frame* frame, canzero_message_guidance_board_back_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (guidance_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_command = (guidance_command)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 3)));
  msg->m_control_active = (bool_t)((((uint32_t*)data)[0] >> 7) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_precharge_status = (sdc_status)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_feedthrough_status = (sdc_status)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_levitation_board1_stream_state(canzero_frame* frame, canzero_message_levitation_board1_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (levitation_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_command = (levitation_command)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 3)));
  msg->m_control_active = (bool_t)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_precharge_status = (sdc_status)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_feedthrough_status = (sdc_status)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_levitation_board2_stream_state(canzero_frame* frame, canzero_message_levitation_board2_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (levitation_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_command = (levitation_command)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 3)));
  msg->m_control_active = (bool_t)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_precharge_status = (sdc_status)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_feedthrough_status = (sdc_status)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_levitation_board3_stream_state(canzero_frame* frame, canzero_message_levitation_board3_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (levitation_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_command = (levitation_command)((((uint32_t*)data)[0] >> 5) & (0xFFFFFFFF >> (32 - 3)));
  msg->m_control_active = (bool_t)((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_precharge_status = (sdc_status)((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_feedthrough_status = (sdc_status)((((uint32_t*)data)[0] >> 10) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_input_board_stream_state(canzero_frame* frame, canzero_message_input_board_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (input_board_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 2)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_input_board_stream_position_estimation(canzero_frame* frame, canzero_message_input_board_stream_position_estimation* msg) {
  uint8_t* data = frame->data;
  msg->m_position = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 16))) * 0.0007629510948348211 + -0;
  msg->m_velocity = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + -10;
  msg->m_acceleration = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 16))) * 0.0015259021896696422 + -50;
}
static void canzero_deserialize_canzero_message_power_board12_stream_state(canzero_frame* frame, canzero_message_power_board12_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (pdu_12v_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 2)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_power_board24_stream_state(canzero_frame* frame, canzero_message_power_board24_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (pdu_24v_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 2)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 2) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_gamepad_stream_input(canzero_frame* frame, canzero_message_gamepad_stream_input* msg) {
  uint8_t* data = frame->data;
  msg->m_lt2 = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 8))) * 0.00392156862745098 + 0;
  msg->m_rt2 = ((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 8))) * 0.00392156862745098 + 0;
  msg->m_lsb_x = ((((uint32_t*)data)[0] >> 16) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1;
  msg->m_lsb_y = ((((uint32_t*)data)[0] >> 24) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1;
  msg->m_rsb_x = (((uint32_t*)data)[1] & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1;
  msg->m_rsb_y = ((((uint32_t*)data)[1] >> 8) & (0xFFFFFFFF >> (32 - 8))) * 0.00784313725490196 + -1;
  msg->m_lt1_down = (bool_t)((((uint32_t*)data)[1] >> 16) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_rt1_down = (bool_t)((((uint32_t*)data)[1] >> 17) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_x_down = (bool_t)((((uint32_t*)data)[1] >> 18) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_y_down = (bool_t)((((uint32_t*)data)[1] >> 19) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_b_down = (bool_t)((((uint32_t*)data)[1] >> 20) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_a_down = (bool_t)((((uint32_t*)data)[1] >> 21) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_lsb_down = (bool_t)((((uint32_t*)data)[1] >> 22) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_rsb_down = (bool_t)((((uint32_t*)data)[1] >> 23) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_heartbeat_can0(canzero_frame* frame, canzero_message_heartbeat_can0* msg) {
  uint8_t* data = frame->data;
  msg->m_node_id = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 8)));
  msg->m_unregister = ((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_ticks_next = ((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 7)));
}
static void canzero_deserialize_canzero_message_heartbeat_can1(canzero_frame* frame, canzero_message_heartbeat_can1* msg) {
  uint8_t* data = frame->data;
  msg->m_node_id = (((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 8)));
  msg->m_unregister = ((((uint32_t*)data)[0] >> 8) & (0xFFFFFFFF >> (32 - 1)));
  msg->m_ticks_next = ((((uint32_t*)data)[0] >> 9) & (0xFFFFFFFF >> (32 - 7)));
}
__attribute__((weak)) void canzero_can0_wdg_timeout(uint8_t node_id) {}
__attribute__((weak)) void canzero_can0_wdg_recovered(uint8_t node_id) {}
__attribute__((weak)) void canzero_can1_wdg_timeout(uint8_t node_id) {}
__attribute__((weak)) void canzero_can1_wdg_recovered(uint8_t node_id) {}

typedef enum {
  HEARTBEAT_JOB_TAG = 0,
  HEARTBEAT_WDG_JOB_TAG = 1,
  GET_RESP_FRAGMENTATION_JOB_TAG = 2,
  STREAM_INTERVAL_JOB_TAG = 3,
} job_tag;

typedef struct {
  uint32_t *buffer;
  uint8_t offset;
  uint8_t size;
  uint8_t od_index;
  uint8_t client_id;
} get_resp_fragmentation_job;

typedef struct {
  uint32_t last_schedule; 
  uint32_t stream_id;
} stream_interval_job;

typedef struct {
  unsigned int* can0_static_wdg_armed;
  int* can0_static_tick_countdowns;
  unsigned int* can0_dynamic_wdg_armed;
  int* can0_dynamic_tick_countdowns;

  unsigned int* can1_static_wdg_armed;
  int* can1_static_tick_countdowns;
  unsigned int* can1_dynamic_wdg_armed;
  int* can1_dynamic_tick_countdowns;
} heartbeat_wdg_job_t;

typedef struct {
  uint32_t climax;
  uint32_t position;
  job_tag tag;
  union {
    get_resp_fragmentation_job get_fragmentation_job;
    stream_interval_job stream_job;
    heartbeat_wdg_job_t wdg_job;
  } job;
} job_t;

union job_pool_allocator_entry {
  job_t job;
  union job_pool_allocator_entry *next;
};

typedef struct {
  union job_pool_allocator_entry job[64];
  union job_pool_allocator_entry *freelist;
} job_pool_allocator;

static job_pool_allocator DMAMEM job_allocator;
static void job_pool_allocator_init() {
  for (uint8_t i = 1; i < 64; i++) {
    job_allocator.job[i - 1].next = job_allocator.job + i;
  }
  job_allocator.job[64 - 1].next = NULL;
  job_allocator.freelist = job_allocator.job;
}

static job_t *job_pool_allocator_alloc() {
  if (job_allocator.freelist != NULL) {
    job_t *job = &job_allocator.freelist->job;
    job_allocator.freelist = job_allocator.freelist->next;
    return job;
  } else {
    return NULL;
  }
}

static void job_pool_allocator_free(job_t *job) {
  union job_pool_allocator_entry *entry = (union job_pool_allocator_entry *)job;
  entry->next = job_allocator.freelist;
  job_allocator.freelist = entry;
}

#define SCHEDULER_HEAP_SIZE 74
typedef struct {
  job_t *heap[SCHEDULER_HEAP_SIZE]; // job**
  uint32_t size;
} job_scheduler_t;

static job_scheduler_t DMAMEM scheduler;
static void scheduler_promote_job(job_t *job) {
  int index = job->position;
  if (index == 0) {
    return;
  }
  int parent = (job->position - 1) / 2;
  while (scheduler.heap[parent]->climax > scheduler.heap[index]->climax) {
    job_t *tmp = scheduler.heap[parent];
    scheduler.heap[parent] = scheduler.heap[index];
    scheduler.heap[index] = tmp;
    scheduler.heap[parent]->position = parent;
    scheduler.heap[index]->position = index;
    index = parent;
    parent = (index - 1) / 2;
  }
  if (index == 0) {
    canzero_request_update(job->climax);
  }
}

static void scheduler_schedule(job_t *job) {
  if (scheduler.size >= SCHEDULER_HEAP_SIZE) {
    return;
  }
  job->position = scheduler.size;
  scheduler.heap[scheduler.size] = job;
  scheduler.size += 1;
  scheduler_promote_job(job);
}

static int scheduler_continue(job_t **job, uint32_t time) {
  *job = scheduler.heap[0];
  return scheduler.heap[0]->climax <= time;
}

static void scheduler_reschedule(uint32_t climax) {
  job_t *job = scheduler.heap[0];
  job->climax = climax;
  int index = 0;
  int hsize = scheduler.size / 2;
  while (index < hsize) {
    int left = index * 2 + 1;
    int right = left + 1;
    int min;
    if (right < scheduler.size &&
        scheduler.heap[left]->climax >= scheduler.heap[right]->climax) {
      min = right;
    } else {
    min = left;
    }
    if (climax <= scheduler.heap[min]->climax) {
      break;
    }
    scheduler.heap[index] = scheduler.heap[min];
    scheduler.heap[index]->position = index;
    index = min;
  }
  scheduler.heap[index] = job;
  scheduler.heap[index]->position = index;
}
static void scheduler_unschedule() {
  scheduler.heap[0] = scheduler.heap[scheduler.size - 1];
  scheduler.heap[0]->position = 0;
  scheduler.size -= 1;
  scheduler_reschedule(scheduler.heap[0]->climax);
}
static const uint32_t get_resp_fragmentation_interval = 10;
static void schedule_get_resp_fragmentation_job(uint32_t *fragmentation_buffer, uint8_t size, uint8_t od_index, uint8_t client_id) {
  job_t *fragmentation_job = job_pool_allocator_alloc();
  fragmentation_job->climax = canzero_get_time() + get_resp_fragmentation_interval;
  fragmentation_job->tag = GET_RESP_FRAGMENTATION_JOB_TAG;
  fragmentation_job->job.get_fragmentation_job.buffer = fragmentation_buffer;
  fragmentation_job->job.get_fragmentation_job.offset = 1;
  fragmentation_job->job.get_fragmentation_job.size = size;
  fragmentation_job->job.get_fragmentation_job.od_index = od_index;
  fragmentation_job->job.get_fragmentation_job.client_id = client_id;
  scheduler_schedule(fragmentation_job);
}

static job_t heartbeat_job;
static const uint32_t heartbeat_interval = 100;
static void schedule_heartbeat_job() {
  heartbeat_job.climax = canzero_get_time();
  heartbeat_job.tag = HEARTBEAT_JOB_TAG;
  scheduler_schedule(&heartbeat_job);
}

static job_t heartbeat_wdg_job;
static const uint32_t heartbeat_wdg_tick_duration = 50;
unsigned int wdg_job_can0_static_wdg_armed[node_id_count];
int wdg_job_can0_static_tick_countdowns[node_id_count];
unsigned int wdg_job_can0_dynamic_wdg_armed[MAX_DYN_HEARTBEATS];
int wdg_job_can0_dynamic_tick_countdowns[MAX_DYN_HEARTBEATS];
unsigned int wdg_job_can1_static_wdg_armed[node_id_count];
int wdg_job_can1_static_tick_countdowns[node_id_count];
unsigned int wdg_job_can1_dynamic_wdg_armed[MAX_DYN_HEARTBEATS];
int wdg_job_can1_dynamic_tick_countdowns[MAX_DYN_HEARTBEATS];

static void schedule_heartbeat_wdg_job() {
  heartbeat_wdg_job.climax = canzero_get_time() + 100;
  heartbeat_wdg_job.tag = HEARTBEAT_WDG_JOB_TAG;
  heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed = wdg_job_can0_static_wdg_armed;
  heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns = wdg_job_can0_static_tick_countdowns;
  heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed = wdg_job_can0_dynamic_wdg_armed;
  heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns = wdg_job_can0_dynamic_tick_countdowns;
  heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed = wdg_job_can1_static_wdg_armed;
  heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns = wdg_job_can1_static_tick_countdowns;
  heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed = wdg_job_can1_dynamic_wdg_armed;
  heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns = wdg_job_can1_dynamic_tick_countdowns;
  for (unsigned int i = 0; i < node_id_count; ++i) {
    heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[i] = 10;
    heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[i] = 0;
    heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[i] = 10;
    heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[i] = 0;
  }
  for (unsigned int i = 0; i < MAX_DYN_HEARTBEATS; ++i) {
    heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[i] = 4;
    heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[i] = 0;
    heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[i] = 4;
    heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[i] = 0;
  }
  scheduler_schedule(&heartbeat_wdg_job);
}

static job_t state_interval_job;
static const uint32_t state_interval = 1;
static void schedule_state_interval_job(){
  uint32_t time = canzero_get_time();
  state_interval_job.climax = time + state_interval;
  state_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  state_interval_job.job.stream_job.stream_id = 0;
  state_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&state_interval_job);
}
static job_t motor_command_interval_job;
static const uint32_t motor_command_interval = 1;
static void schedule_motor_command_interval_job(){
  uint32_t time = canzero_get_time();
  motor_command_interval_job.climax = time + motor_command_interval;
  motor_command_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  motor_command_interval_job.job.stream_job.stream_id = 1;
  motor_command_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&motor_command_interval_job);
}
static job_t input_board_command_interval_job;
static const uint32_t input_board_command_interval = 1;
static void schedule_input_board_command_interval_job(){
  uint32_t time = canzero_get_time();
  input_board_command_interval_job.climax = time + input_board_command_interval;
  input_board_command_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  input_board_command_interval_job.job.stream_job.stream_id = 2;
  input_board_command_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&input_board_command_interval_job);
}
static job_t guidance_command_interval_job;
static const uint32_t guidance_command_interval = 1;
static void schedule_guidance_command_interval_job(){
  uint32_t time = canzero_get_time();
  guidance_command_interval_job.climax = time + guidance_command_interval;
  guidance_command_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  guidance_command_interval_job.job.stream_job.stream_id = 3;
  guidance_command_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&guidance_command_interval_job);
}
static job_t levitation_command_interval_job;
static const uint32_t levitation_command_interval = 1;
static void schedule_levitation_command_interval_job(){
  uint32_t time = canzero_get_time();
  levitation_command_interval_job.climax = time + levitation_command_interval;
  levitation_command_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  levitation_command_interval_job.job.stream_job.stream_id = 4;
  levitation_command_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&levitation_command_interval_job);
}
static job_t pdu_12v_command_interval_job;
static const uint32_t pdu_12v_command_interval = 1;
static void schedule_pdu_12v_command_interval_job(){
  uint32_t time = canzero_get_time();
  pdu_12v_command_interval_job.climax = time + pdu_12v_command_interval;
  pdu_12v_command_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  pdu_12v_command_interval_job.job.stream_job.stream_id = 5;
  pdu_12v_command_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&pdu_12v_command_interval_job);
}
static job_t pdu_24v_command_interval_job;
static const uint32_t pdu_24v_command_interval = 1;
static void schedule_pdu_24v_command_interval_job(){
  uint32_t time = canzero_get_time();
  pdu_24v_command_interval_job.climax = time + pdu_24v_command_interval;
  pdu_24v_command_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  pdu_24v_command_interval_job.job.stream_job.stream_id = 6;
  pdu_24v_command_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&pdu_24v_command_interval_job);
}
static job_t errors_interval_job;
static const uint32_t errors_interval = 1;
static void schedule_errors_interval_job(){
  uint32_t time = canzero_get_time();
  errors_interval_job.climax = time + errors_interval;
  errors_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  errors_interval_job.job.stream_job.stream_id = 7;
  errors_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&errors_interval_job);
}

static void schedule_jobs(uint32_t time) {
  for (uint8_t i = 0; i < 100; ++i) {
    canzero_enter_critical();
    job_t *job;
    if (!scheduler_continue(&job, time)) {
      canzero_exit_critical();
      return;
    }
    switch (job->tag) {
      case STREAM_INTERVAL_JOB_TAG: {
        switch (job->job.stream_job.stream_id) {
      case 0: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mother_board_stream_state stream_message;
        stream_message.m_state = __oe_state;
        stream_message.m_command = __oe_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_state(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 1: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mother_board_stream_motor_command stream_message;
        stream_message.m_target_acceleration = __oe_target_acceleration;
        stream_message.m_motor_driver_command = __oe_motor_driver_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_motor_command(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 2: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mother_board_stream_input_board_command stream_message;
        stream_message.m_input_board_command = __oe_input_board_command;
        stream_message.m_input_board_assert_45V_online = __oe_input_board_assert_45V_online;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_input_board_command(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 3: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mother_board_stream_guidance_command stream_message;
        stream_message.m_guidance_command = __oe_guidance_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_guidance_command(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 4: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mother_board_stream_levitation_command stream_message;
        stream_message.m_levitation_command = __oe_levitation_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_levitation_command(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 5: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mother_board_stream_pdu_12v_command stream_message;
        stream_message.m_power_board12_command = __oe_power_board12_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_pdu_12v_command(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 6: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mother_board_stream_pdu_24v_command stream_message;
        stream_message.m_power_board24_command = __oe_power_board24_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_pdu_24v_command(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 7: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 1000);
        canzero_exit_critical();
        canzero_message_mother_board_stream_errors stream_message;
        stream_message.m_error_heartbeat_miss = __oe_error_heartbeat_miss;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_errors(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
        default:
          canzero_exit_critical();
          break;
        }
        break;
      }
      case HEARTBEAT_JOB_TAG: {
        scheduler_reschedule(time + heartbeat_interval);
        canzero_exit_critical();
        canzero_frame heartbeat_frame;
        canzero_message_heartbeat_can0 heartbeat_can0;
        heartbeat_can0.m_node_id = node_id_mother_board;
        heartbeat_can0.m_unregister = 0;
        heartbeat_can0.m_ticks_next = 20;
        canzero_serialize_canzero_message_heartbeat_can0(&heartbeat_can0, &heartbeat_frame);
        canzero_can0_send(&heartbeat_frame);
        canzero_message_heartbeat_can1 heartbeat_can1;
        heartbeat_can1.m_node_id = node_id_mother_board;
        heartbeat_can1.m_unregister = 0;
        heartbeat_can1.m_ticks_next = 20;
        canzero_serialize_canzero_message_heartbeat_can1(&heartbeat_can1, &heartbeat_frame);
        canzero_can1_send(&heartbeat_frame);
        break;
      }
      case HEARTBEAT_WDG_JOB_TAG: {
        scheduler_reschedule(time + heartbeat_wdg_tick_duration);
        canzero_exit_critical();
        for (unsigned int i = 0; i < node_id_count; ++i) {
          heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[i] 
            -= heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[i];
          heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[i] 
            -= heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[i];
        }
        for (unsigned int i = 0; i < MAX_DYN_HEARTBEATS; ++i) {
          heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[i] 
            -= heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[i];
          heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[i] 
            -= heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[i];
        }
        for (unsigned int i = 0; i < node_id_count; ++i) {
          if (heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[i] <= 0) {
            canzero_can0_wdg_timeout(i);
          }
          if (heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[i] <= 0) {
            canzero_can1_wdg_timeout(i);
          }
        }
        for (unsigned int i = 0; i < MAX_DYN_HEARTBEATS; ++i) {
          if (heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[i] <= 0) {
            canzero_can0_wdg_timeout(node_id_count + i);
          }
          if (heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[i] <= 0) {
            canzero_can1_wdg_timeout(node_id_count + i);
          }
        }
        break;
      }
      case GET_RESP_FRAGMENTATION_JOB_TAG: {
        get_resp_fragmentation_job *fragmentation_job = &job->job.get_fragmentation_job;
        canzero_message_get_resp fragmentation_response;
        fragmentation_response.m_header.m_sof = 0;
        fragmentation_response.m_header.m_toggle = fragmentation_job->offset % 2;
        fragmentation_response.m_header.m_od_index = fragmentation_job->od_index;
        fragmentation_response.m_header.m_client_id = fragmentation_job->client_id;
        fragmentation_response.m_header.m_server_id = 0x1;
        fragmentation_response.m_data = fragmentation_job->buffer[fragmentation_job->offset];
        fragmentation_job->offset += 1;
        if (fragmentation_job->offset == fragmentation_job->size) {
          fragmentation_response.m_header.m_eof = 1;
          scheduler_unschedule();
        } else {
          fragmentation_response.m_header.m_eof = 0;
          scheduler_reschedule(time + get_resp_fragmentation_interval);
        }
        canzero_exit_critical();
        canzero_frame fragmentation_frame;
        canzero_serialize_canzero_message_get_resp(&fragmentation_response, &fragmentation_frame);
        canzero_can0_send(&fragmentation_frame);
        break;
      }
      default: {
        canzero_exit_critical();
        break;
      }
    }
  }
}

static uint32_t scheduler_next_job_timeout() {
  return scheduler.heap[0]->climax;
}

static uint32_t DMAMEM __oe_config_hash_rx_fragmentation_buffer[2];
static uint32_t DMAMEM __oe_build_time_rx_fragmentation_buffer[2];
static uint32_t DMAMEM __oe_velocity_pid_rx_fragmentation_buffer[6];
static PROGMEM void canzero_handle_get_req(canzero_frame* frame) {
  canzero_message_get_req msg;
  canzero_deserialize_canzero_message_get_req(frame, &msg);
  if (msg.m_header.m_server_id != node_id_mother_board) {
    return;
  }
  canzero_message_get_resp resp{};
  switch (msg.m_header.m_od_index) {
  case 0: {
    {
      uint64_t masked = (__oe_config_hash & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
      __oe_config_hash_rx_fragmentation_buffer[0] = ((uint32_t*)&masked)[0];
      __oe_config_hash_rx_fragmentation_buffer[1] = ((uint32_t*)&masked)[1];
    }
    resp.m_data = __oe_config_hash_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_config_hash_rx_fragmentation_buffer, 2, 0, msg.m_header.m_client_id);
    break;
  }
  case 1: {
    __oe_build_time_rx_fragmentation_buffer[0] = (__oe_build_time.m_year & (0xFFFFFFFF >> (32 - 16)));
    __oe_build_time_rx_fragmentation_buffer[0] |= ((__oe_build_time.m_month & (0xFFFFFFFF >> (32 - 8))) << 16);
    __oe_build_time_rx_fragmentation_buffer[0] |= ((__oe_build_time.m_day & (0xFFFFFFFF >> (32 - 8))) << 24);
    __oe_build_time_rx_fragmentation_buffer[1] = (__oe_build_time.m_hour & (0xFFFFFFFF >> (32 - 8)));
    __oe_build_time_rx_fragmentation_buffer[1] |= ((__oe_build_time.m_min & (0xFFFFFFFF >> (32 - 8))) << 8);
    __oe_build_time_rx_fragmentation_buffer[1] |= ((__oe_build_time.m_sec & (0xFFFFFFFF >> (32 - 8))) << 16);

    resp.m_data = __oe_build_time_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_build_time_rx_fragmentation_buffer, 2, 1, msg.m_header.m_client_id);
    break;
  }
  case 2: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_state) & (0xFF >> (8 - 5)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 3: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_command) & (0xFF >> (8 - 4)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 4: {
    resp.m_data |= min_u32((__oe_track_length - (0)) / 0.000000011641532185403987, 0xFFFFFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 5: {
    resp.m_data |= min_u32((__oe_brake_margin - (0)) / 0.000000011641532185403987, 0xFFFFFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 6: {
    resp.m_data |= min_u32((__oe_emergency_brake_margin - (0)) / 0.000000011641532185403987, 0xFFFFFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 7: {
    resp.m_data |= min_u32((__oe_target_acceleration - (-50)) / 0.000000023283064370807974, 0xFFFFFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 8: {
    resp.m_data |= min_u32((__oe_acceleration_target_velocity - (0)) / 0.0000000023283064370807974, 0xFFFFFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 9: {
    {
      uint64_t masked = (min_u64((__oe_velocity_pid.m_Kp - ((double)0)) / (double)0.00000000000000005421010862427522, 0xFFFFFFFFFFFFFFFFull) & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
      __oe_velocity_pid_rx_fragmentation_buffer[0] = ((uint32_t*)&masked)[0];
      __oe_velocity_pid_rx_fragmentation_buffer[1] = ((uint32_t*)&masked)[1];
    }    {
      uint64_t masked = (min_u64((__oe_velocity_pid.m_Ki - ((double)0)) / (double)0.00000000000000005421010862427522, 0xFFFFFFFFFFFFFFFFull) & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
      __oe_velocity_pid_rx_fragmentation_buffer[2] = ((uint32_t*)&masked)[0];
      __oe_velocity_pid_rx_fragmentation_buffer[3] = ((uint32_t*)&masked)[1];
    }    {
      uint64_t masked = (min_u64((__oe_velocity_pid.m_Kd - ((double)0)) / (double)0.00000000000000005421010862427522, 0xFFFFFFFFFFFFFFFFull) & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
      __oe_velocity_pid_rx_fragmentation_buffer[4] = ((uint32_t*)&masked)[0];
      __oe_velocity_pid_rx_fragmentation_buffer[5] = ((uint32_t*)&masked)[1];
    }
    resp.m_data = __oe_velocity_pid_rx_fragmentation_buffer[0];
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 0;
    resp.m_header.m_toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_velocity_pid_rx_fragmentation_buffer, 6, 9, msg.m_header.m_client_id);
    break;
  }
  case 10: {
    resp.m_data |= min_u32((__oe_position - (-0)) / 0.0007629510948348211, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 11: {
    resp.m_data |= min_u32((__oe_velocity - (-10)) / 0.00030518043793392844, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 12: {
    resp.m_data |= min_u32((__oe_acceleration - (-50)) / 0.0015259021896696422, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 13: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_motor_driver_state) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 14: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_motor_driver_command) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 15: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_motor_driver_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 16: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_command) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 17: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_front_state) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 18: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_front_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 19: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_back_state) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 20: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_back_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 21: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_command) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 22: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board1_state) & (0xFF >> (8 - 4)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 23: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board1_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 24: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board2_state) & (0xFF >> (8 - 4)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 25: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board2_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 26: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board3_state) & (0xFF >> (8 - 4)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 27: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board3_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 28: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_state) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 29: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_command) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 30: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_assert_45V_online) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 31: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 32: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board12_state) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 33: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board12_command) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 34: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board12_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 35: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board24_state) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 36: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board24_command) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 37: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board24_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 38: {
    resp.m_data |= min_u32((__oe_gamepad_max_acceleration - (0)) / 0.0000000023283064370807974, 0xFFFFFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 39: {
    resp.m_data |= min_u32((__oe_gamepad_lt2 - (0)) / 0.00392156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 40: {
    resp.m_data |= min_u32((__oe_gamepad_rt2 - (0)) / 0.00392156862745098, 0xFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 41: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_error_heartbeat_miss) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  }
  resp.m_header.m_od_index = msg.m_header.m_od_index;
  resp.m_header.m_client_id = msg.m_header.m_client_id;
  resp.m_header.m_server_id = msg.m_header.m_server_id;
  canzero_frame resp_frame;
  canzero_serialize_canzero_message_get_resp(&resp, &resp_frame);
  canzero_can0_send(&resp_frame);
}
static uint32_t DMAMEM config_hash_tmp_tx_fragmentation_buffer[2];
static uint32_t DMAMEM config_hash_tmp_tx_fragmentation_offset = 0;
static uint32_t DMAMEM build_time_tmp_tx_fragmentation_buffer[2];
static uint32_t DMAMEM build_time_tmp_tx_fragmentation_offset = 0;
static uint32_t DMAMEM velocity_pid_tmp_tx_fragmentation_buffer[6];
static uint32_t DMAMEM velocity_pid_tmp_tx_fragmentation_offset = 0;
static PROGMEM void canzero_handle_set_req(canzero_frame* frame) {
  canzero_message_set_req msg;
  canzero_deserialize_canzero_message_set_req(frame, &msg);
  if (msg.m_header.m_server_id != 1) {
    return;
  }
  canzero_message_set_resp resp{};
  switch (msg.m_header.m_od_index) {
  case 0 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      config_hash_tmp_tx_fragmentation_offset = 0;
    }else {
      config_hash_tmp_tx_fragmentation_offset += 1;
      if (config_hash_tmp_tx_fragmentation_offset >= 2) {
        return;
      }
    }
    config_hash_tmp_tx_fragmentation_buffer[config_hash_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    uint64_t config_hash_tmp;
    config_hash_tmp = (uint64_t)config_hash_tmp_tx_fragmentation_buffer[0] | (((uint64_t)(config_hash_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 32)))) << 32);
    canzero_set_config_hash(config_hash_tmp);
    break;
  }
  case 1 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      build_time_tmp_tx_fragmentation_offset = 0;
    }else {
      build_time_tmp_tx_fragmentation_offset += 1;
      if (build_time_tmp_tx_fragmentation_offset >= 2) {
        return;
      }
    }
    build_time_tmp_tx_fragmentation_buffer[build_time_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    date_time build_time_tmp;
    build_time_tmp.m_year = (build_time_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 16)));
    build_time_tmp.m_month = (build_time_tmp_tx_fragmentation_buffer[0] >> 16) & (0xFFFFFFFF >> (32 - 8));
    build_time_tmp.m_day = (build_time_tmp_tx_fragmentation_buffer[0] >> 24) & (0xFFFFFFFF >> (32 - 8));
    build_time_tmp.m_hour = (build_time_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 8)));
    build_time_tmp.m_min = (build_time_tmp_tx_fragmentation_buffer[1] >> 8) & (0xFFFFFFFF >> (32 - 8));
    build_time_tmp.m_sec = (build_time_tmp_tx_fragmentation_buffer[1] >> 16) & (0xFFFFFFFF >> (32 - 8));
    canzero_set_build_time(build_time_tmp);
    break;
  }
  case 2 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    global_state state_tmp;
    state_tmp = ((global_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 5))));
    canzero_set_state(state_tmp);
    break;
  }
  case 3 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    global_command command_tmp;
    command_tmp = ((global_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 4))));
    canzero_set_command(command_tmp);
    break;
  }
  case 4 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float track_length_tmp;
    track_length_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 32))) * 0.000000011641532185403987 + 0);
    canzero_set_track_length(track_length_tmp);
    break;
  }
  case 5 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float brake_margin_tmp;
    brake_margin_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 32))) * 0.000000011641532185403987 + 0);
    canzero_set_brake_margin(brake_margin_tmp);
    break;
  }
  case 6 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float emergency_brake_margin_tmp;
    emergency_brake_margin_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 32))) * 0.000000011641532185403987 + 0);
    canzero_set_emergency_brake_margin(emergency_brake_margin_tmp);
    break;
  }
  case 7 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float target_acceleration_tmp;
    target_acceleration_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 32))) * 0.000000023283064370807974 + -50);
    canzero_set_target_acceleration(target_acceleration_tmp);
    break;
  }
  case 8 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float acceleration_target_velocity_tmp;
    acceleration_target_velocity_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 32))) * 0.0000000023283064370807974 + 0);
    canzero_set_acceleration_target_velocity(acceleration_target_velocity_tmp);
    break;
  }
  case 9 : {
    if (msg.m_header.m_sof == 1) {
      if (msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 0) {
        return; //TODO proper error response frame!
      }
      velocity_pid_tmp_tx_fragmentation_offset = 0;
    }else {
      velocity_pid_tmp_tx_fragmentation_offset += 1;
      if (velocity_pid_tmp_tx_fragmentation_offset >= 6) {
        return;
      }
    }
    velocity_pid_tmp_tx_fragmentation_buffer[velocity_pid_tmp_tx_fragmentation_offset] = msg.m_data;
    if (msg.m_header.m_eof == 0) {
      return;
    }
    pid_parameters velocity_pid_tmp;
    velocity_pid_tmp.m_Kp = ((uint64_t)velocity_pid_tmp_tx_fragmentation_buffer[0] | (((uint64_t)(velocity_pid_tmp_tx_fragmentation_buffer[1] & (0xFFFFFFFF >> (32 - 32)))) << 32)) * 0.00000000000000005421010862427522 + 0;
    velocity_pid_tmp.m_Ki = ((uint64_t)velocity_pid_tmp_tx_fragmentation_buffer[2] | (((uint64_t)(velocity_pid_tmp_tx_fragmentation_buffer[3] & (0xFFFFFFFF >> (32 - 32)))) << 32)) * 0.00000000000000005421010862427522 + 0;
    velocity_pid_tmp.m_Kd = ((uint64_t)velocity_pid_tmp_tx_fragmentation_buffer[4] | (((uint64_t)(velocity_pid_tmp_tx_fragmentation_buffer[5] & (0xFFFFFFFF >> (32 - 32)))) << 32)) * 0.00000000000000005421010862427522 + 0;
    canzero_set_velocity_pid(velocity_pid_tmp);
    break;
  }
  case 10 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float position_tmp;
    position_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.0007629510948348211 + -0);
    canzero_set_position(position_tmp);
    break;
  }
  case 11 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float velocity_tmp;
    velocity_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + -10);
    canzero_set_velocity(velocity_tmp);
    break;
  }
  case 12 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float acceleration_tmp;
    acceleration_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.0015259021896696422 + -50);
    canzero_set_acceleration(acceleration_tmp);
    break;
  }
  case 13 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    motor_state motor_driver_state_tmp;
    motor_driver_state_tmp = ((motor_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_motor_driver_state(motor_driver_state_tmp);
    break;
  }
  case 14 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    motor_command motor_driver_command_tmp;
    motor_driver_command_tmp = ((motor_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_motor_driver_command(motor_driver_command_tmp);
    break;
  }
  case 15 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status motor_driver_sdc_status_tmp;
    motor_driver_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_motor_driver_sdc_status(motor_driver_sdc_status_tmp);
    break;
  }
  case 16 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    guidance_command guidance_command_tmp;
    guidance_command_tmp = ((guidance_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_guidance_command(guidance_command_tmp);
    break;
  }
  case 17 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    guidance_state guidance_board_front_state_tmp;
    guidance_board_front_state_tmp = ((guidance_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_guidance_board_front_state(guidance_board_front_state_tmp);
    break;
  }
  case 18 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status guidance_board_front_sdc_status_tmp;
    guidance_board_front_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_guidance_board_front_sdc_status(guidance_board_front_sdc_status_tmp);
    break;
  }
  case 19 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    guidance_state guidance_board_back_state_tmp;
    guidance_board_back_state_tmp = ((guidance_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_guidance_board_back_state(guidance_board_back_state_tmp);
    break;
  }
  case 20 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status guidance_board_back_sdc_status_tmp;
    guidance_board_back_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_guidance_board_back_sdc_status(guidance_board_back_sdc_status_tmp);
    break;
  }
  case 21 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    levitation_command levitation_command_tmp;
    levitation_command_tmp = ((levitation_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_levitation_command(levitation_command_tmp);
    break;
  }
  case 22 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    levitation_state levitation_board1_state_tmp;
    levitation_board1_state_tmp = ((levitation_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 4))));
    canzero_set_levitation_board1_state(levitation_board1_state_tmp);
    break;
  }
  case 23 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status levitation_board1_sdc_status_tmp;
    levitation_board1_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_levitation_board1_sdc_status(levitation_board1_sdc_status_tmp);
    break;
  }
  case 24 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    levitation_state levitation_board2_state_tmp;
    levitation_board2_state_tmp = ((levitation_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 4))));
    canzero_set_levitation_board2_state(levitation_board2_state_tmp);
    break;
  }
  case 25 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status levitation_board2_sdc_status_tmp;
    levitation_board2_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_levitation_board2_sdc_status(levitation_board2_sdc_status_tmp);
    break;
  }
  case 26 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    levitation_state levitation_board3_state_tmp;
    levitation_board3_state_tmp = ((levitation_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 4))));
    canzero_set_levitation_board3_state(levitation_board3_state_tmp);
    break;
  }
  case 27 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status levitation_board3_sdc_status_tmp;
    levitation_board3_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_levitation_board3_sdc_status(levitation_board3_sdc_status_tmp);
    break;
  }
  case 28 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    input_board_state input_board_state_tmp;
    input_board_state_tmp = ((input_board_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_input_board_state(input_board_state_tmp);
    break;
  }
  case 29 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    input_board_command input_board_command_tmp;
    input_board_command_tmp = ((input_board_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_input_board_command(input_board_command_tmp);
    break;
  }
  case 30 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    bool_t input_board_assert_45V_online_tmp;
    input_board_assert_45V_online_tmp = ((bool_t)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_input_board_assert_45V_online(input_board_assert_45V_online_tmp);
    break;
  }
  case 31 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status input_board_sdc_status_tmp;
    input_board_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_input_board_sdc_status(input_board_sdc_status_tmp);
    break;
  }
  case 32 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    pdu_12v_state power_board12_state_tmp;
    power_board12_state_tmp = ((pdu_12v_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_power_board12_state(power_board12_state_tmp);
    break;
  }
  case 33 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    pdu_12v_command power_board12_command_tmp;
    power_board12_command_tmp = ((pdu_12v_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_power_board12_command(power_board12_command_tmp);
    break;
  }
  case 34 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status power_board12_sdc_status_tmp;
    power_board12_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_power_board12_sdc_status(power_board12_sdc_status_tmp);
    break;
  }
  case 35 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    pdu_24v_state power_board24_state_tmp;
    power_board24_state_tmp = ((pdu_24v_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_power_board24_state(power_board24_state_tmp);
    break;
  }
  case 36 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    pdu_24v_command power_board24_command_tmp;
    power_board24_command_tmp = ((pdu_24v_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_power_board24_command(power_board24_command_tmp);
    break;
  }
  case 37 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status power_board24_sdc_status_tmp;
    power_board24_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_power_board24_sdc_status(power_board24_sdc_status_tmp);
    break;
  }
  case 38 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float gamepad_max_acceleration_tmp;
    gamepad_max_acceleration_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 32))) * 0.0000000023283064370807974 + 0);
    canzero_set_gamepad_max_acceleration(gamepad_max_acceleration_tmp);
    break;
  }
  case 39 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float gamepad_lt2_tmp;
    gamepad_lt2_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.00392156862745098 + 0);
    canzero_set_gamepad_lt2(gamepad_lt2_tmp);
    break;
  }
  case 40 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float gamepad_rt2_tmp;
    gamepad_rt2_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 8))) * 0.00392156862745098 + 0);
    canzero_set_gamepad_rt2(gamepad_rt2_tmp);
    break;
  }
  case 41 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    error_flag error_heartbeat_miss_tmp;
    error_heartbeat_miss_tmp = ((error_flag)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_heartbeat_miss(error_heartbeat_miss_tmp);
    break;
  }
  default:
    return;
  }
  resp.m_header.m_od_index = msg.m_header.m_od_index;
  resp.m_header.m_client_id = msg.m_header.m_client_id;
  resp.m_header.m_server_id = msg.m_header.m_server_id;
  resp.m_header.m_erno = set_resp_erno_Success;
  canzero_frame resp_frame;
  canzero_serialize_canzero_message_set_resp(&resp, &resp_frame);
  canzero_can1_send(&resp_frame);

}
static void canzero_handle_motor_driver_stream_state(canzero_frame* frame) {
  canzero_message_motor_driver_stream_state msg;
  canzero_deserialize_canzero_message_motor_driver_stream_state(frame, &msg);
  canzero_set_motor_driver_state(msg.m_state);
  canzero_set_motor_driver_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_guidance_board_front_stream_state(canzero_frame* frame) {
  canzero_message_guidance_board_front_stream_state msg;
  canzero_deserialize_canzero_message_guidance_board_front_stream_state(frame, &msg);
  canzero_set_guidance_board_front_state(msg.m_state);
  canzero_set_guidance_board_front_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_guidance_board_back_stream_state(canzero_frame* frame) {
  canzero_message_guidance_board_back_stream_state msg;
  canzero_deserialize_canzero_message_guidance_board_back_stream_state(frame, &msg);
  canzero_set_guidance_board_back_state(msg.m_state);
  canzero_set_guidance_board_back_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_levitation_board1_stream_state(canzero_frame* frame) {
  canzero_message_levitation_board1_stream_state msg;
  canzero_deserialize_canzero_message_levitation_board1_stream_state(frame, &msg);
  canzero_set_levitation_board1_state(msg.m_state);
  canzero_set_levitation_board1_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_levitation_board2_stream_state(canzero_frame* frame) {
  canzero_message_levitation_board2_stream_state msg;
  canzero_deserialize_canzero_message_levitation_board2_stream_state(frame, &msg);
  canzero_set_levitation_board2_state(msg.m_state);
  canzero_set_levitation_board2_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_levitation_board3_stream_state(canzero_frame* frame) {
  canzero_message_levitation_board3_stream_state msg;
  canzero_deserialize_canzero_message_levitation_board3_stream_state(frame, &msg);
  canzero_set_levitation_board3_state(msg.m_state);
  canzero_set_levitation_board3_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_input_board_stream_state(canzero_frame* frame) {
  canzero_message_input_board_stream_state msg;
  canzero_deserialize_canzero_message_input_board_stream_state(frame, &msg);
  canzero_set_input_board_state(msg.m_state);
  canzero_set_input_board_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_input_board_stream_position_estimation(canzero_frame* frame) {
  canzero_message_input_board_stream_position_estimation msg;
  canzero_deserialize_canzero_message_input_board_stream_position_estimation(frame, &msg);
  canzero_set_position(msg.m_position);
  canzero_set_velocity(msg.m_velocity);
  canzero_set_acceleration(msg.m_acceleration);
}
static void canzero_handle_power_board12_stream_state(canzero_frame* frame) {
  canzero_message_power_board12_stream_state msg;
  canzero_deserialize_canzero_message_power_board12_stream_state(frame, &msg);
  canzero_set_power_board12_state(msg.m_state);
  canzero_set_power_board12_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_power_board24_stream_state(canzero_frame* frame) {
  canzero_message_power_board24_stream_state msg;
  canzero_deserialize_canzero_message_power_board24_stream_state(frame, &msg);
  canzero_set_power_board24_state(msg.m_state);
  canzero_set_power_board24_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_gamepad_stream_input(canzero_frame* frame) {
  canzero_message_gamepad_stream_input msg;
  canzero_deserialize_canzero_message_gamepad_stream_input(frame, &msg);
  canzero_set_gamepad_lt2(msg.m_lt2);
  canzero_set_gamepad_rt2(msg.m_rt2);
}
 void canzero_handle_heartbeat_can0(canzero_frame* frame) {
  canzero_message_heartbeat_can0 msg;
  canzero_deserialize_canzero_message_heartbeat_can0(frame, &msg);

  if (msg.m_node_id < node_id_count) {   // static heartbeat
    if (msg.m_unregister != 0) {  // unregister only unregisters this bus
      heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[msg.m_node_id] = 0;
    } else { // register registers for all buses
      heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[msg.m_node_id] = 1;
      heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[msg.m_node_id] = 1;
    }
    if (heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[msg.m_node_id] <= 0 &&
        msg.m_ticks_next > 0) {
      canzero_can0_wdg_recovered(msg.m_node_id);
    }
    heartbeat_wdg_job.job.wdg_job.can0_static_tick_countdowns[msg.m_node_id] = msg.m_ticks_next;
  } else {  // dynamic heartbeat
    if (msg.m_unregister != 0) { // unregister only unregisters this bus
      heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 0;
    } else { // register registers all buses
      heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 1;
      heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 1;
    }
    if (heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[msg.m_node_id - node_id_count] <= 0 
        && msg.m_ticks_next > 0) {
      canzero_can0_wdg_recovered(msg.m_node_id);
    }
    heartbeat_wdg_job.job.wdg_job.can0_dynamic_tick_countdowns[msg.m_node_id - node_id_count]
      = msg.m_ticks_next;
  }
}
 void canzero_handle_heartbeat_can1(canzero_frame* frame) {
  canzero_message_heartbeat_can1 msg;
  canzero_deserialize_canzero_message_heartbeat_can1(frame, &msg);

  if (msg.m_node_id < node_id_count) {   // static heartbeat
    if (msg.m_unregister != 0) {  // unregister only unregisters this bus
      heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[msg.m_node_id] = 0;
    } else { // register registers for all buses
      heartbeat_wdg_job.job.wdg_job.can0_static_wdg_armed[msg.m_node_id] = 1;
      heartbeat_wdg_job.job.wdg_job.can1_static_wdg_armed[msg.m_node_id] = 1;
    }
    if (heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[msg.m_node_id] <= 0 &&
        msg.m_ticks_next > 0) {
      canzero_can1_wdg_recovered(msg.m_node_id);
    }
    heartbeat_wdg_job.job.wdg_job.can1_static_tick_countdowns[msg.m_node_id] = msg.m_ticks_next;
  } else {  // dynamic heartbeat
    if (msg.m_unregister != 0) { // unregister only unregisters this bus
      heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 0;
    } else { // register registers all buses
      heartbeat_wdg_job.job.wdg_job.can0_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 1;
      heartbeat_wdg_job.job.wdg_job.can1_dynamic_wdg_armed[msg.m_node_id - node_id_count] = 1;
    }
    if (heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[msg.m_node_id - node_id_count] <= 0 
        && msg.m_ticks_next > 0) {
      canzero_can1_wdg_recovered(msg.m_node_id);
    }
    heartbeat_wdg_job.job.wdg_job.can1_dynamic_tick_countdowns[msg.m_node_id - node_id_count]
      = msg.m_ticks_next;
  }
}
void canzero_can0_poll() {
  canzero_frame frame;
  while (canzero_can0_recv(&frame)) {
    switch (frame.id) {
      case 0x17E:
        canzero_handle_get_req(&frame);
        break;
      case 0xE7:
        canzero_handle_motor_driver_stream_state(&frame);
        break;
      case 0xA9:
        canzero_handle_guidance_board_back_stream_state(&frame);
        break;
      case 0x128:
        canzero_handle_levitation_board2_stream_state(&frame);
        break;
      case 0x127:
        canzero_handle_power_board12_stream_state(&frame);
        break;
      case 0x1E5:
        canzero_handle_heartbeat_can0(&frame);
        break;
    }
  }
}
void canzero_can1_poll() {
  canzero_frame frame;
  while (canzero_can1_recv(&frame)) {
    switch (frame.id) {
      case 0x1BE:
        canzero_handle_set_req(&frame);
        break;
      case 0xE9:
        canzero_handle_guidance_board_front_stream_state(&frame);
        break;
      case 0xE8:
        canzero_handle_levitation_board1_stream_state(&frame);
        break;
      case 0xA7:
        canzero_handle_levitation_board3_stream_state(&frame);
        break;
      case 0xA8:
        canzero_handle_input_board_stream_state(&frame);
        break;
      case 0x129:
        canzero_handle_input_board_stream_position_estimation(&frame);
        break;
      case 0xA6:
        canzero_handle_power_board24_stream_state(&frame);
        break;
      case 0xBF:
        canzero_handle_gamepad_stream_input(&frame);
        break;
      case 0x1E4:
        canzero_handle_heartbeat_can1(&frame);
        break;
    }
  }
}
uint32_t canzero_update_continue(uint32_t time){
  schedule_jobs(time);
  return scheduler_next_job_timeout();
}
#define COMPUTE_BUILD_YEAR \
    ( (__DATE__[ 7] - '0') * 1000 + \
        (__DATE__[ 8] - '0') *  100 + \
        (__DATE__[ 9] - '0') *   10 + \
        (__DATE__[10] - '0') \
    )
#define COMPUTE_BUILD_DAY \
    ( \
        ((__DATE__[4] >= '0') ? (__DATE__[4] - '0') * 10 : 0) + \
        (__DATE__[5] - '0') \
    )
#define BUILD_MONTH_IS_JAN (__DATE__[0] == 'J' && __DATE__[1] == 'a' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_FEB (__DATE__[0] == 'F')
#define BUILD_MONTH_IS_MAR (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'r')
#define BUILD_MONTH_IS_APR (__DATE__[0] == 'A' && __DATE__[1] == 'p')
#define BUILD_MONTH_IS_MAY (__DATE__[0] == 'M' && __DATE__[1] == 'a' && __DATE__[2] == 'y')
#define BUILD_MONTH_IS_JUN (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'n')
#define BUILD_MONTH_IS_JUL (__DATE__[0] == 'J' && __DATE__[1] == 'u' && __DATE__[2] == 'l')
#define BUILD_MONTH_IS_AUG (__DATE__[0] == 'A' && __DATE__[1] == 'u')
#define BUILD_MONTH_IS_SEP (__DATE__[0] == 'S')
#define BUILD_MONTH_IS_OCT (__DATE__[0] == 'O')
#define BUILD_MONTH_IS_NOV (__DATE__[0] == 'N')
#define BUILD_MONTH_IS_DEC (__DATE__[0] == 'D')
#define COMPUTE_BUILD_MONTH \
    ( \
        (BUILD_MONTH_IS_JAN) ?  1 : \
        (BUILD_MONTH_IS_FEB) ?  2 : \
        (BUILD_MONTH_IS_MAR) ?  3 : \
        (BUILD_MONTH_IS_APR) ?  4 : \
        (BUILD_MONTH_IS_MAY) ?  5 : \
        (BUILD_MONTH_IS_JUN) ?  6 : \
        (BUILD_MONTH_IS_JUL) ?  7 : \
        (BUILD_MONTH_IS_AUG) ?  8 : \
        (BUILD_MONTH_IS_SEP) ?  9 : \
        (BUILD_MONTH_IS_OCT) ? 10 : \
        (BUILD_MONTH_IS_NOV) ? 11 : \
        (BUILD_MONTH_IS_DEC) ? 12 : \
        /* error default */  99 \
    )
#define COMPUTE_BUILD_HOUR ((__TIME__[0] - '0') * 10 + __TIME__[1] - '0')
#define COMPUTE_BUILD_MIN  ((__TIME__[3] - '0') * 10 + __TIME__[4] - '0')
#define COMPUTE_BUILD_SEC  ((__TIME__[6] - '0') * 10 + __TIME__[7] - '0')
#define BUILD_DATE_IS_BAD (__DATE__[0] == '?')
#define BUILD_YEAR  ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_YEAR)
#define BUILD_MONTH ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_MONTH)
#define BUILD_DAY   ((BUILD_DATE_IS_BAD) ? 99 : COMPUTE_BUILD_DAY)
#define BUILD_TIME_IS_BAD (__TIME__[0] == '?')
#define BUILD_HOUR  ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_HOUR)
#define BUILD_MIN   ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_MIN)
#define BUILD_SEC   ((BUILD_TIME_IS_BAD) ? 99 :  COMPUTE_BUILD_SEC)
void canzero_init() {
  __oe_config_hash = 13839418029710939869ull;
  __oe_build_time = {
    .m_year = BUILD_YEAR,
    .m_month = BUILD_MONTH,
    .m_day = BUILD_DAY,
    .m_hour = BUILD_HOUR,
    .m_min = BUILD_MIN,
    .m_sec = BUILD_SEC
  };
  canzero_can0_setup(1000000, NULL, 0);
  canzero_can1_setup(1000000, NULL, 0);

  job_pool_allocator_init();
  scheduler.size = 0;
  schedule_heartbeat_job();
  schedule_heartbeat_wdg_job();
  schedule_state_interval_job();
  schedule_motor_command_interval_job();
  schedule_input_board_command_interval_job();
  schedule_guidance_command_interval_job();
  schedule_levitation_command_interval_job();
  schedule_pdu_12v_command_interval_job();
  schedule_pdu_24v_command_interval_job();
  schedule_errors_interval_job();

}
void canzero_set_state(global_state value) {
  extern global_state __oe_state;
  if (__oe_state != value) {
    __oe_state = value;
    if (state_interval_job.climax > state_interval_job.job.stream_job.last_schedule + 1) {
      state_interval_job.climax = state_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_command(global_command value) {
  extern global_command __oe_command;
  if (__oe_command != value) {
    __oe_command = value;
    if (state_interval_job.climax > state_interval_job.job.stream_job.last_schedule + 1) {
      state_interval_job.climax = state_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_target_acceleration(float value) {
  extern float __oe_target_acceleration;
  if (__oe_target_acceleration != value) {
    __oe_target_acceleration = value;
    if (motor_command_interval_job.climax > motor_command_interval_job.job.stream_job.last_schedule + 1) {
      motor_command_interval_job.climax = motor_command_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&motor_command_interval_job);
    }
  }
}
void canzero_set_motor_driver_command(motor_command value) {
  extern motor_command __oe_motor_driver_command;
  if (__oe_motor_driver_command != value) {
    __oe_motor_driver_command = value;
    if (motor_command_interval_job.climax > motor_command_interval_job.job.stream_job.last_schedule + 1) {
      motor_command_interval_job.climax = motor_command_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&motor_command_interval_job);
    }
  }
}
void canzero_set_guidance_command(guidance_command value) {
  extern guidance_command __oe_guidance_command;
  if (__oe_guidance_command != value) {
    __oe_guidance_command = value;
    if (guidance_command_interval_job.climax > guidance_command_interval_job.job.stream_job.last_schedule + 1) {
      guidance_command_interval_job.climax = guidance_command_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&guidance_command_interval_job);
    }
  }
}
void canzero_set_levitation_command(levitation_command value) {
  extern levitation_command __oe_levitation_command;
  if (__oe_levitation_command != value) {
    __oe_levitation_command = value;
    if (levitation_command_interval_job.climax > levitation_command_interval_job.job.stream_job.last_schedule + 1) {
      levitation_command_interval_job.climax = levitation_command_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&levitation_command_interval_job);
    }
  }
}
void canzero_set_input_board_command(input_board_command value) {
  extern input_board_command __oe_input_board_command;
  if (__oe_input_board_command != value) {
    __oe_input_board_command = value;
    if (input_board_command_interval_job.climax > input_board_command_interval_job.job.stream_job.last_schedule + 1) {
      input_board_command_interval_job.climax = input_board_command_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&input_board_command_interval_job);
    }
  }
}
void canzero_set_input_board_assert_45V_online(bool_t value) {
  extern bool_t __oe_input_board_assert_45V_online;
  if (__oe_input_board_assert_45V_online != value) {
    __oe_input_board_assert_45V_online = value;
    if (input_board_command_interval_job.climax > input_board_command_interval_job.job.stream_job.last_schedule + 1) {
      input_board_command_interval_job.climax = input_board_command_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&input_board_command_interval_job);
    }
  }
}
void canzero_set_power_board12_command(pdu_12v_command value) {
  extern pdu_12v_command __oe_power_board12_command;
  if (__oe_power_board12_command != value) {
    __oe_power_board12_command = value;
    if (pdu_12v_command_interval_job.climax > pdu_12v_command_interval_job.job.stream_job.last_schedule + 1) {
      pdu_12v_command_interval_job.climax = pdu_12v_command_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&pdu_12v_command_interval_job);
    }
  }
}
void canzero_set_power_board24_command(pdu_24v_command value) {
  extern pdu_24v_command __oe_power_board24_command;
  if (__oe_power_board24_command != value) {
    __oe_power_board24_command = value;
    if (pdu_24v_command_interval_job.climax > pdu_24v_command_interval_job.job.stream_job.last_schedule + 1) {
      pdu_24v_command_interval_job.climax = pdu_24v_command_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&pdu_24v_command_interval_job);
    }
  }
}
void canzero_set_error_heartbeat_miss(error_flag value) {
  extern error_flag __oe_error_heartbeat_miss;
  if (__oe_error_heartbeat_miss != value) {
    __oe_error_heartbeat_miss = value;
    if (errors_interval_job.climax > errors_interval_job.job.stream_job.last_schedule + 1) {
      errors_interval_job.climax = errors_interval_job.job.stream_job.last_schedule + 1;
      scheduler_promote_job(&errors_interval_job);
    }
  }
}
static uint32_t DMAMEM __oe_config_hash_send_fragmentation_buffer[2];
static uint32_t DMAMEM __oe_build_time_send_fragmentation_buffer[2];
static uint32_t DMAMEM __oe_velocity_pid_send_fragmentation_buffer[6];
void canzero_send_config_hash() {
  canzero_message_get_resp msg;
  {
    uint64_t masked = (__oe_config_hash & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
    __oe_config_hash_send_fragmentation_buffer[0] = ((uint32_t*)&masked)[0];
    __oe_config_hash_send_fragmentation_buffer[1] = ((uint32_t*)&masked)[1];
  }
  msg.m_data = __oe_config_hash_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 0;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_config_hash_send_fragmentation_buffer, 2, 0, 255);

}
void canzero_send_build_time() {
  canzero_message_get_resp msg;
  __oe_build_time_send_fragmentation_buffer[0] = (__oe_build_time.m_year & (0xFFFFFFFF >> (32 - 16)));
  __oe_build_time_send_fragmentation_buffer[0] |= ((__oe_build_time.m_month & (0xFFFFFFFF >> (32 - 8))) << 16);
  __oe_build_time_send_fragmentation_buffer[0] |= ((__oe_build_time.m_day & (0xFFFFFFFF >> (32 - 8))) << 24);
  __oe_build_time_send_fragmentation_buffer[1] = (__oe_build_time.m_hour & (0xFFFFFFFF >> (32 - 8)));
  __oe_build_time_send_fragmentation_buffer[1] |= ((__oe_build_time.m_min & (0xFFFFFFFF >> (32 - 8))) << 8);
  __oe_build_time_send_fragmentation_buffer[1] |= ((__oe_build_time.m_sec & (0xFFFFFFFF >> (32 - 8))) << 16);

  msg.m_data = __oe_build_time_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 1;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_build_time_send_fragmentation_buffer, 2, 1, 255);

}
void canzero_send_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_state) & (0xFF >> (8 - 5)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 2;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_command() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_command) & (0xFF >> (8 - 4)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 3;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_track_length() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_track_length - (0)) / 0.000000011641532185403987, 0xFFFFFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 4;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_brake_margin() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_brake_margin - (0)) / 0.000000011641532185403987, 0xFFFFFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 5;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_emergency_brake_margin() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_emergency_brake_margin - (0)) / 0.000000011641532185403987, 0xFFFFFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 6;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_target_acceleration() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_target_acceleration - (-50)) / 0.000000023283064370807974, 0xFFFFFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 7;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_acceleration_target_velocity() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_acceleration_target_velocity - (0)) / 0.0000000023283064370807974, 0xFFFFFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 8;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_velocity_pid() {
  canzero_message_get_resp msg;
  {
    uint64_t masked = (min_u64((__oe_velocity_pid.m_Kp - ((double)0)) / (double)0.00000000000000005421010862427522, 0xFFFFFFFFFFFFFFFFull) & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
    __oe_velocity_pid_send_fragmentation_buffer[0] = ((uint32_t*)&masked)[0];
    __oe_velocity_pid_send_fragmentation_buffer[1] = ((uint32_t*)&masked)[1];
  }  {
    uint64_t masked = (min_u64((__oe_velocity_pid.m_Ki - ((double)0)) / (double)0.00000000000000005421010862427522, 0xFFFFFFFFFFFFFFFFull) & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
    __oe_velocity_pid_send_fragmentation_buffer[2] = ((uint32_t*)&masked)[0];
    __oe_velocity_pid_send_fragmentation_buffer[3] = ((uint32_t*)&masked)[1];
  }  {
    uint64_t masked = (min_u64((__oe_velocity_pid.m_Kd - ((double)0)) / (double)0.00000000000000005421010862427522, 0xFFFFFFFFFFFFFFFFull) & (0xFFFFFFFFFFFFFFFF >> (64 - 64)));
    __oe_velocity_pid_send_fragmentation_buffer[4] = ((uint32_t*)&masked)[0];
    __oe_velocity_pid_send_fragmentation_buffer[5] = ((uint32_t*)&masked)[1];
  }
  msg.m_data = __oe_velocity_pid_send_fragmentation_buffer[0];
  msg.m_header.m_eof = 0;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 9;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
  schedule_get_resp_fragmentation_job(__oe_velocity_pid_send_fragmentation_buffer, 6, 9, 255);

}
void canzero_send_position() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_position - (-0)) / 0.0007629510948348211, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 10;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_velocity() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_velocity - (-10)) / 0.00030518043793392844, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 11;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_acceleration() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_acceleration - (-50)) / 0.0015259021896696422, 0xFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 12;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_motor_driver_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_motor_driver_state) & (0xFF >> (8 - 3)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 13;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_motor_driver_command() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_motor_driver_command) & (0xFF >> (8 - 3)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 14;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_motor_driver_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_motor_driver_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 15;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_guidance_command() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_command) & (0xFF >> (8 - 3)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 16;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_guidance_board_front_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_front_state) & (0xFF >> (8 - 3)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 17;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_guidance_board_front_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_front_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 18;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_guidance_board_back_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_back_state) & (0xFF >> (8 - 3)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 19;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_guidance_board_back_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_back_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 20;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_levitation_command() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_command) & (0xFF >> (8 - 3)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 21;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_levitation_board1_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board1_state) & (0xFF >> (8 - 4)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 22;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_levitation_board1_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board1_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 23;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_levitation_board2_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board2_state) & (0xFF >> (8 - 4)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 24;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_levitation_board2_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board2_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 25;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_levitation_board3_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board3_state) & (0xFF >> (8 - 4)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 26;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_levitation_board3_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board3_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 27;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_input_board_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_state) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 28;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_input_board_command() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_command) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 29;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_input_board_assert_45V_online() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_assert_45V_online) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 30;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_input_board_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 31;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_power_board12_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_power_board12_state) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 32;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_power_board12_command() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_power_board12_command) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 33;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_power_board12_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_power_board12_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 34;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_power_board24_state() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_power_board24_state) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 35;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_power_board24_command() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_power_board24_command) & (0xFF >> (8 - 2)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 36;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_power_board24_sdc_status() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_power_board24_sdc_status) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 37;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_max_acceleration() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_gamepad_max_acceleration - (0)) / 0.0000000023283064370807974, 0xFFFFFFFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 38;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_lt2() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_gamepad_lt2 - (0)) / 0.00392156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 39;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_gamepad_rt2() {
  canzero_message_get_resp msg;
  msg.m_data |= min_u32((__oe_gamepad_rt2 - (0)) / 0.00392156862745098, 0xFF) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 40;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
void canzero_send_error_heartbeat_miss() {
  canzero_message_get_resp msg;
  msg.m_data |= ((uint32_t)(((uint8_t)__oe_error_heartbeat_miss) & (0xFF >> (8 - 1)))) << 0;
  msg.m_header.m_eof = 1;
  msg.m_header.m_sof = 1;
  msg.m_header.m_toggle = 0;
  msg.m_header.m_od_index = 41;
  msg.m_header.m_client_id = 255;
  msg.m_header.m_server_id = node_id_mother_board;
  canzero_frame sender_frame;
  canzero_serialize_canzero_message_get_resp(&msg, &sender_frame);
  canzero_can0_send(&sender_frame);
}
