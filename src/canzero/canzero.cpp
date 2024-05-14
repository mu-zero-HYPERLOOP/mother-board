#include "canzero.h"
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
uint64_t __oe_config_hash;
date_time __oe_build_time;
global_state __oe_state;
global_command __oe_command;
float __oe_track_length;
float __oe_brake_margin;
float __oe_emergency_brake_margin;
float __oe_target_acceleration;
float __oe_target_velocity;
float __oe_position;
float __oe_velocity;
float __oe_acceleration;
motor_state __oe_motor_driver_state;
motor_command __oe_motor_driver_command;
sdc_status __oe_motor_driver_sdc_status;
guidance_command __oe_guidance_command;
guidance_state __oe_guidance_board_front_state;
sdc_status __oe_guidance_board_front_sdc_status;
guidance_state __oe_guidance_board_back_state;
sdc_status __oe_guidance_board_back_sdc_status;
levitation_command __oe_levitation_command;
levitation_state __oe_levitation_board_front_state;
sdc_status __oe_levitation_board_front_sdc_status;
levitation_state __oe_levitation_board_middle_state;
sdc_status __oe_levitation_board_middle_sdc_status;
levitation_state __oe_levitation_board_back_state;
sdc_status __oe_levitation_board_back_sdc_status;
input_board_state __oe_input_board_state;
sdc_status __oe_input_board_sdc_status;
pdu_state __oe_power_board12_state;
sdc_status __oe_power_board12_sdc_status;
pdu_state __oe_power_board24_state;
sdc_status __oe_power_board24_sdc_status;
static void canzero_serialize_canzero_message_get_resp(canzero_message_get_resp* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x9E;
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
  frame->id = 0xBE;
  frame->dlc = 4;
  ((uint32_t*)data)[0] = (uint16_t)(msg->m_header.m_od_index & (0xFFFF >> (16 - 13)));
  ((uint32_t*)data)[0] |= msg->m_header.m_client_id << 13;
  ((uint32_t*)data)[0] |= msg->m_header.m_server_id << 21;
  ((uint32_t*)data)[0] |= (uint8_t)(msg->m_header.m_erno & (0xFF >> (8 - 1))) << 29;
}
static void canzero_serialize_canzero_message_mother_board_stream_state(canzero_message_mother_board_stream_state* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x73;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_state & (0xFF >> (8 - 4)));
}
static void canzero_serialize_canzero_message_mother_board_stream_motor_command(canzero_message_mother_board_stream_motor_command* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x4C;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_motor_driver_command & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_mother_board_stream_guidance_command(canzero_message_mother_board_stream_guidance_command* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x74;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_guidance_command & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_mother_board_stream_levitation_command(canzero_message_mother_board_stream_levitation_command* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0x53;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_levitation_command & (0xFF >> (8 - 3)));
}
static void canzero_serialize_canzero_message_heartbeat(canzero_message_heartbeat* msg, canzero_frame* frame) {
  uint8_t* data = frame->data;
  frame->id = 0xDF;
  frame->dlc = 1;
  ((uint32_t*)data)[0] = (uint8_t)(msg->m_node_id & (0xFF >> (8 - 4)));
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
  msg->m_state = (motor_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 4) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_guidance_board_front_stream_state(canzero_frame* frame, canzero_message_guidance_board_front_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (guidance_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_guidance_board_back_stream_state(canzero_frame* frame, canzero_message_guidance_board_back_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (guidance_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_levitation_board_front_stream_state(canzero_frame* frame, canzero_message_levitation_board_front_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (levitation_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_levitation_board_middle_stream_state(canzero_frame* frame, canzero_message_levitation_board_middle_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (levitation_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_levitation_board_back_stream_state(canzero_frame* frame, canzero_message_levitation_board_back_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (levitation_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 3)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 3) & (0xFFFFFFFF >> (32 - 1)));
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
  msg->m_linear_encoder_count = ((((uint32_t*)data)[1] >> 16) & (0xFFFFFFFF >> (32 - 16)));
}
static void canzero_deserialize_canzero_message_power_board12_stream_state(canzero_frame* frame, canzero_message_power_board12_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (pdu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_power_board24_stream_state(canzero_frame* frame, canzero_message_power_board24_stream_state* msg) {
  uint8_t* data = frame->data;
  msg->m_state = (pdu_state)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 1)));
  msg->m_sdc_status = (sdc_status)((((uint32_t*)data)[0] >> 1) & (0xFFFFFFFF >> (32 - 1)));
}
static void canzero_deserialize_canzero_message_heartbeat(canzero_frame* frame, canzero_message_heartbeat* msg) {
  uint8_t* data = frame->data;
  msg->m_node_id = (node_id)(((uint32_t*)data)[0] & (0xFFFFFFFF >> (32 - 4)));
}

__attribute__((weak)) void canzero_wdg_timeout(uint8_t node_id) {}

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

#define MAX_DYN_HEARTBEATS 10
typedef struct {
  unsigned int static_wdg_armed[node_id_count];
  unsigned int static_tick_counters[node_id_count];
  unsigned int dynamic_wdg_armed[MAX_DYN_HEARTBEATS];
  unsigned int dynamic_tick_counters[MAX_DYN_HEARTBEATS];
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

static job_pool_allocator job_allocator;
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
#define SCHEDULE_HEAP_SIZE 256
typedef struct {
  job_t *heap[SCHEDULE_HEAP_SIZE]; // job**
  uint32_t size;
} job_scheduler_t;
static job_scheduler_t scheduler;
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
  if (scheduler.size >= SCHEDULE_HEAP_SIZE) {
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
static void schedule_heartbeat_wdg_job() {
  heartbeat_wdg_job.climax = canzero_get_time() + 100;
  heartbeat_wdg_job.tag = HEARTBEAT_WDG_JOB_TAG;
  for (unsigned int i = 0; i < node_id_count; ++i) {
    heartbeat_wdg_job.job.wdg_job.static_tick_counters[i] = 0;
    heartbeat_wdg_job.job.wdg_job.static_wdg_armed[i] = 0;
  }
  for (unsigned int i = 0; i < MAX_DYN_HEARTBEATS; ++i) {
    heartbeat_wdg_job.job.wdg_job.dynamic_tick_counters[i] = 0;
    heartbeat_wdg_job.job.wdg_job.dynamic_wdg_armed[i] = 0;
  }
  scheduler_schedule(&heartbeat_wdg_job);
}

static job_t state_interval_job;
static const uint32_t state_interval = 0;
static void schedule_state_interval_job(){
  uint32_t time = canzero_get_time();
  state_interval_job.climax = time + state_interval;
  state_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  state_interval_job.job.stream_job.stream_id = 0;
  state_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&state_interval_job);
}
static job_t motor_command_interval_job;
static const uint32_t motor_command_interval = 0;
static void schedule_motor_command_interval_job(){
  uint32_t time = canzero_get_time();
  motor_command_interval_job.climax = time + motor_command_interval;
  motor_command_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  motor_command_interval_job.job.stream_job.stream_id = 1;
  motor_command_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&motor_command_interval_job);
}
static job_t guidance_command_interval_job;
static const uint32_t guidance_command_interval = 0;
static void schedule_guidance_command_interval_job(){
  uint32_t time = canzero_get_time();
  guidance_command_interval_job.climax = time + guidance_command_interval;
  guidance_command_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  guidance_command_interval_job.job.stream_job.stream_id = 2;
  guidance_command_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&guidance_command_interval_job);
}
static job_t levitation_command_interval_job;
static const uint32_t levitation_command_interval = 0;
static void schedule_levitation_command_interval_job(){
  uint32_t time = canzero_get_time();
  levitation_command_interval_job.climax = time + levitation_command_interval;
  levitation_command_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  levitation_command_interval_job.job.stream_job.stream_id = 3;
  levitation_command_interval_job.job.stream_job.last_schedule = time;
  scheduler_schedule(&levitation_command_interval_job);
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
        stream_message.m_motor_driver_command = __oe_motor_driver_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_motor_command(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 2: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mother_board_stream_guidance_command stream_message;
        stream_message.m_guidance_command = __oe_guidance_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_guidance_command(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 3: {
        job->job.stream_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_mother_board_stream_levitation_command stream_message;
        stream_message.m_levitation_command = __oe_levitation_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_mother_board_stream_levitation_command(&stream_message, &stream_frame);
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
        canzero_message_heartbeat heartbeat;
        heartbeat.m_node_id = node_id_mother_board;
        canzero_frame heartbeat_frame;
        canzero_serialize_canzero_message_heartbeat(&heartbeat, &heartbeat_frame);
        canzero_can1_send(&heartbeat_frame);
        break;
      }
      case HEARTBEAT_WDG_JOB_TAG: {
        scheduler_reschedule(time + heartbeat_wdg_tick_duration);
        canzero_exit_critical();
        for (unsigned int i = 0; i < node_id_count; ++i) {
          heartbeat_wdg_job.job.wdg_job.static_tick_counters[i] 
            += heartbeat_wdg_job.job.wdg_job.static_wdg_armed[i];
        }
        for (unsigned int i = 0; i < MAX_DYN_HEARTBEATS; ++i) {
          heartbeat_wdg_job.job.wdg_job.dynamic_tick_counters[i] 
            += heartbeat_wdg_job.job.wdg_job.dynamic_wdg_armed[i];
        }
        for (unsigned int i = 0; i < node_id_count; ++i) {
          if (heartbeat_wdg_job.job.wdg_job.static_tick_counters[i] >= 4) {
            canzero_wdg_timeout(i);
          }
        }
        for (unsigned int i = 0; i < MAX_DYN_HEARTBEATS; ++i) {
          if (heartbeat_wdg_job.job.wdg_job.dynamic_tick_counters[i] >= 4) {
            canzero_wdg_timeout(node_id_count + i);
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
        fragmentation_response.m_header.m_server_id = 0x0;
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
        canzero_can1_send(&fragmentation_frame);
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

static uint32_t __oe_config_hash_rx_fragmentation_buffer[2];
static uint32_t __oe_build_time_rx_fragmentation_buffer[2];
static void canzero_handle_get_req(canzero_frame* frame) {
  canzero_message_get_req msg;
  canzero_deserialize_canzero_message_get_req(frame, &msg);
  if (msg.m_header.m_server_id != 0) {
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
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_state) & (0xFF >> (8 - 4)))) << 0;
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
    resp.m_data |= min_u32((__oe_target_acceleration - (0)) / 0.0000000011641532185403987, 0xFFFFFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 8: {
    resp.m_data |= min_u32((__oe_target_velocity - (0)) / 0.0000000023283064370807974, 0xFFFFFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 9: {
    resp.m_data |= min_u32((__oe_position - (-0)) / 0.0007629510948348211, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 10: {
    resp.m_data |= min_u32((__oe_velocity - (-10)) / 0.00030518043793392844, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 11: {
    resp.m_data |= min_u32((__oe_acceleration - (-50)) / 0.0015259021896696422, 0xFFFF) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 12: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_motor_driver_state) & (0xFF >> (8 - 4)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 13: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_motor_driver_command) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 14: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_motor_driver_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 15: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_command) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 16: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_front_state) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 17: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_front_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 18: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_back_state) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 19: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_guidance_board_back_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 20: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_command) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 21: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board_front_state) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 22: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board_front_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 23: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board_middle_state) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 24: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board_middle_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 25: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board_back_state) & (0xFF >> (8 - 3)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 26: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_levitation_board_back_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 27: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_state) & (0xFF >> (8 - 2)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 28: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_input_board_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 29: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board12_state) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 30: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board12_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 31: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board24_state) & (0xFF >> (8 - 1)))) << 0;
    resp.m_header.m_sof = 1;
    resp.m_header.m_eof = 1;
    resp.m_header.m_toggle = 0;
    break;
  }
  case 32: {
    resp.m_data |= ((uint32_t)(((uint8_t)__oe_power_board24_sdc_status) & (0xFF >> (8 - 1)))) << 0;
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
  canzero_can1_send(&resp_frame);
}
static uint32_t config_hash_tmp_tx_fragmentation_buffer[2];
static uint32_t config_hash_tmp_tx_fragmentation_offset = 0;
static uint32_t build_time_tmp_tx_fragmentation_buffer[2];
static uint32_t build_time_tmp_tx_fragmentation_offset = 0;
static void canzero_handle_set_req(canzero_frame* frame) {
  canzero_message_set_req msg;
  canzero_deserialize_canzero_message_set_req(frame, &msg);
  if (msg.m_header.m_server_id != 0) {
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
    state_tmp = ((global_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 4))));
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
    target_acceleration_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 32))) * 0.0000000011641532185403987 + 0);
    canzero_set_target_acceleration(target_acceleration_tmp);
    break;
  }
  case 8 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float target_velocity_tmp;
    target_velocity_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 32))) * 0.0000000023283064370807974 + 0);
    canzero_set_target_velocity(target_velocity_tmp);
    break;
  }
  case 9 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float position_tmp;
    position_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.0007629510948348211 + -0);
    canzero_set_position(position_tmp);
    break;
  }
  case 10 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float velocity_tmp;
    velocity_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + -10);
    canzero_set_velocity(velocity_tmp);
    break;
  }
  case 11 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    float acceleration_tmp;
    acceleration_tmp = (float)(((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 16))) * 0.0015259021896696422 + -50);
    canzero_set_acceleration(acceleration_tmp);
    break;
  }
  case 12 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    motor_state motor_driver_state_tmp;
    motor_driver_state_tmp = ((motor_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 4))));
    canzero_set_motor_driver_state(motor_driver_state_tmp);
    break;
  }
  case 13 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    motor_command motor_driver_command_tmp;
    motor_driver_command_tmp = ((motor_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_motor_driver_command(motor_driver_command_tmp);
    break;
  }
  case 14 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status motor_driver_sdc_status_tmp;
    motor_driver_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_motor_driver_sdc_status(motor_driver_sdc_status_tmp);
    break;
  }
  case 15 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    guidance_command guidance_command_tmp;
    guidance_command_tmp = ((guidance_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_guidance_command(guidance_command_tmp);
    break;
  }
  case 16 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    guidance_state guidance_board_front_state_tmp;
    guidance_board_front_state_tmp = ((guidance_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_guidance_board_front_state(guidance_board_front_state_tmp);
    break;
  }
  case 17 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status guidance_board_front_sdc_status_tmp;
    guidance_board_front_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_guidance_board_front_sdc_status(guidance_board_front_sdc_status_tmp);
    break;
  }
  case 18 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    guidance_state guidance_board_back_state_tmp;
    guidance_board_back_state_tmp = ((guidance_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_guidance_board_back_state(guidance_board_back_state_tmp);
    break;
  }
  case 19 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status guidance_board_back_sdc_status_tmp;
    guidance_board_back_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_guidance_board_back_sdc_status(guidance_board_back_sdc_status_tmp);
    break;
  }
  case 20 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    levitation_command levitation_command_tmp;
    levitation_command_tmp = ((levitation_command)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_levitation_command(levitation_command_tmp);
    break;
  }
  case 21 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    levitation_state levitation_board_front_state_tmp;
    levitation_board_front_state_tmp = ((levitation_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_levitation_board_front_state(levitation_board_front_state_tmp);
    break;
  }
  case 22 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status levitation_board_front_sdc_status_tmp;
    levitation_board_front_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_levitation_board_front_sdc_status(levitation_board_front_sdc_status_tmp);
    break;
  }
  case 23 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    levitation_state levitation_board_middle_state_tmp;
    levitation_board_middle_state_tmp = ((levitation_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_levitation_board_middle_state(levitation_board_middle_state_tmp);
    break;
  }
  case 24 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status levitation_board_middle_sdc_status_tmp;
    levitation_board_middle_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_levitation_board_middle_sdc_status(levitation_board_middle_sdc_status_tmp);
    break;
  }
  case 25 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    levitation_state levitation_board_back_state_tmp;
    levitation_board_back_state_tmp = ((levitation_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_levitation_board_back_state(levitation_board_back_state_tmp);
    break;
  }
  case 26 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status levitation_board_back_sdc_status_tmp;
    levitation_board_back_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_levitation_board_back_sdc_status(levitation_board_back_sdc_status_tmp);
    break;
  }
  case 27 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    input_board_state input_board_state_tmp;
    input_board_state_tmp = ((input_board_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_input_board_state(input_board_state_tmp);
    break;
  }
  case 28 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status input_board_sdc_status_tmp;
    input_board_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_input_board_sdc_status(input_board_sdc_status_tmp);
    break;
  }
  case 29 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    pdu_state power_board12_state_tmp;
    power_board12_state_tmp = ((pdu_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_power_board12_state(power_board12_state_tmp);
    break;
  }
  case 30 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status power_board12_sdc_status_tmp;
    power_board12_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_power_board12_sdc_status(power_board12_sdc_status_tmp);
    break;
  }
  case 31 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    pdu_state power_board24_state_tmp;
    power_board24_state_tmp = ((pdu_state)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_power_board24_state(power_board24_state_tmp);
    break;
  }
  case 32 : {
    if (msg.m_header.m_sof != 1 || msg.m_header.m_toggle != 0 || msg.m_header.m_eof != 1) {
      return;
    }
    sdc_status power_board24_sdc_status_tmp;
    power_board24_sdc_status_tmp = ((sdc_status)((msg.m_data >> 0) & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_power_board24_sdc_status(power_board24_sdc_status_tmp);
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
  canzero_can0_send(&resp_frame);

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
static void canzero_handle_levitation_board_front_stream_state(canzero_frame* frame) {
  canzero_message_levitation_board_front_stream_state msg;
  canzero_deserialize_canzero_message_levitation_board_front_stream_state(frame, &msg);
  canzero_set_levitation_board_front_state(msg.m_state);
  canzero_set_levitation_board_front_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_levitation_board_middle_stream_state(canzero_frame* frame) {
  canzero_message_levitation_board_middle_stream_state msg;
  canzero_deserialize_canzero_message_levitation_board_middle_stream_state(frame, &msg);
  canzero_set_levitation_board_middle_state(msg.m_state);
  canzero_set_levitation_board_middle_sdc_status(msg.m_sdc_status);
}
static void canzero_handle_levitation_board_back_stream_state(canzero_frame* frame) {
  canzero_message_levitation_board_back_stream_state msg;
  canzero_deserialize_canzero_message_levitation_board_back_stream_state(frame, &msg);
  canzero_set_levitation_board_back_state(msg.m_state);
  canzero_set_levitation_board_back_sdc_status(msg.m_sdc_status);
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
__attribute__((weak)) void canzero_handle_heartbeat(canzero_frame* frame) {
  canzero_message_heartbeat msg;
  canzero_deserialize_canzero_message_heartbeat(frame, &msg);
}
void canzero_can0_poll() {
  canzero_frame frame;
  while (canzero_can0_recv(&frame)) {
    switch (frame.id) {
      case 0x6E:
        canzero_handle_motor_driver_stream_state(&frame);
        break;
      case 0x51:
        canzero_handle_guidance_board_back_stream_state(&frame);
        break;
      case 0x4E:
        canzero_handle_levitation_board_middle_stream_state(&frame);
        break;
      case 0x4F:
        canzero_handle_levitation_board_back_stream_state(&frame);
        break;
      case 0x70:
        canzero_handle_input_board_stream_state(&frame);
        break;
      case 0x6D:
        canzero_handle_power_board24_stream_state(&frame);
        break;
    }
  }
}
void canzero_can1_poll() {
  canzero_frame frame;
  while (canzero_can1_recv(&frame)) {
    switch (frame.id) {
      case 0x9F:
        canzero_handle_get_req(&frame);
        break;
      case 0xBF:
        canzero_handle_set_req(&frame);
        break;
      case 0x71:
        canzero_handle_guidance_board_front_stream_state(&frame);
        break;
      case 0x6F:
        canzero_handle_levitation_board_front_stream_state(&frame);
        break;
      case 0x50:
        canzero_handle_input_board_stream_position_estimation(&frame);
        break;
      case 0x4D:
        canzero_handle_power_board12_stream_state(&frame);
        break;
      case 0xDF:
        canzero_handle_heartbeat(&frame);
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
  __oe_config_hash = 16778065883237437206ull;
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
  schedule_state_interval_job();
  schedule_motor_command_interval_job();
  schedule_guidance_command_interval_job();
  schedule_levitation_command_interval_job();

}
void canzero_set_state(global_state value) {
  extern global_state __oe_state;
  if (__oe_state != value) {
    __oe_state = value;
    uint32_t time = canzero_get_time();
    if (state_interval_job.climax > state_interval_job.job.stream_job.last_schedule + 0) {
      state_interval_job.climax = state_interval_job.job.stream_job.last_schedule + 0;
      scheduler_promote_job(&state_interval_job);
    }
  }
}
void canzero_set_motor_driver_command(motor_command value) {
  extern motor_command __oe_motor_driver_command;
  if (__oe_motor_driver_command != value) {
    __oe_motor_driver_command = value;
    uint32_t time = canzero_get_time();
    if (motor_command_interval_job.climax > motor_command_interval_job.job.stream_job.last_schedule + 0) {
      motor_command_interval_job.climax = motor_command_interval_job.job.stream_job.last_schedule + 0;
      scheduler_promote_job(&motor_command_interval_job);
    }
  }
}
void canzero_set_guidance_command(guidance_command value) {
  extern guidance_command __oe_guidance_command;
  if (__oe_guidance_command != value) {
    __oe_guidance_command = value;
    uint32_t time = canzero_get_time();
    if (guidance_command_interval_job.climax > guidance_command_interval_job.job.stream_job.last_schedule + 0) {
      guidance_command_interval_job.climax = guidance_command_interval_job.job.stream_job.last_schedule + 0;
      scheduler_promote_job(&guidance_command_interval_job);
    }
  }
}
void canzero_set_levitation_command(levitation_command value) {
  extern levitation_command __oe_levitation_command;
  if (__oe_levitation_command != value) {
    __oe_levitation_command = value;
    uint32_t time = canzero_get_time();
    if (levitation_command_interval_job.climax > levitation_command_interval_job.job.stream_job.last_schedule + 0) {
      levitation_command_interval_job.climax = levitation_command_interval_job.job.stream_job.last_schedule + 0;
      scheduler_promote_job(&levitation_command_interval_job);
    }
  }
}
