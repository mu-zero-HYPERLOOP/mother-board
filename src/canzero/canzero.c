#include "canzero.h"
mlu_command __oe_mlu_command;
mgu_command __oe_mgu_command;
motor_command __oe_motor_command;
pdu_channel_control __oe_pdu24_lp_channel1_control;
pdu_channel_control __oe_pdu24_lp_channel2_control;
pdu_channel_control __oe_pdu24_lp_channel3_control;
pdu_channel_control __oe_pdu24_lp_channel4_control;
pdu_channel_control __oe_pdu24_lp_channel5_control;
pdu_channel_control __oe_pdu24_lp_channel6_control;
pdu_channel_control __oe_pdu24_lp_channel7_control;
pdu_channel_control __oe_pdu24_lp_channel8_control;
pdu_channel_control __oe_pdu24_lp_channel9_control;
pdu_channel_control __oe_pdu24_lp_channel10_control;
pdu_channel_control __oe_pdu24_lp_channel11_control;
pdu_channel_control __oe_pdu24_lp_channel12_control;
pdu_channel_control __oe_pdu24_hp_channel1_control;
pdu_channel_control __oe_pdu24_hp_channel2_control;
pdu_channel_control __oe_pdu24_hp_channel3_control;
pdu_channel_control __oe_pdu24_hp_channel4_control;
pdu_channel_control __oe_pdu12_lp_channel1_control;
pdu_channel_control __oe_pdu12_lp_channel2_control;
pdu_channel_control __oe_pdu12_lp_channel3_control;
pdu_channel_control __oe_pdu12_lp_channel4_control;
pdu_channel_control __oe_pdu12_lp_channel5_control;
pdu_channel_control __oe_pdu12_lp_channel6_control;
pdu_channel_control __oe_pdu12_lp_channel7_control;
pdu_channel_control __oe_pdu12_lp_channel8_control;
pdu_channel_control __oe_pdu12_lp_channel9_control;
pdu_channel_control __oe_pdu12_lp_channel10_control;
pdu_channel_control __oe_pdu12_lp_channel11_control;
pdu_channel_control __oe_pdu12_lp_channel12_control;
pdu_channel_control __oe_pdu12_hp_channel1_control;
pdu_channel_control __oe_pdu12_hp_channel2_control;
pdu_channel_control __oe_pdu12_hp_channel3_control;
pdu_channel_control __oe_pdu12_hp_channel4_control;
global_state __oe_global_state;
global_command __oe_command;
bool_t __oe_sync_mlu_pid_values;
mlu_pid_values __oe_mlu_pid_values;
bool_t __oe_sync_mgu_pid_values;
mgu_pid_values __oe_mgu_pid_values;
float __oe_cpu_temperature;
sdc_status __oe_sdc_status;
float __oe_power_estimation;
float __oe_power_estimation24;
float __oe_power_estimation48;
pdu_state __oe_pdu12_state;
pdu_state __oe_pdu24_state;
input_board_state __oe_input_board_state;
float __oe_position;
float __oe_velocity;
float __oe_acceleration;
error_flag __oe_error_input_board_mcu_over_temperature;
error_flag __oe_error_invalid_position_estimation;
error_flag __oe_error_invalid_position;
error_flag __oe_error_invalid_velocity_profile;
error_flag __oe_error_invalid_acceleration_profile;
error_flag __oe_error_battery_over_voltage;
error_flag __oe_error_cooling_cycle_over_pressure;
error_flag __oe_error_cooling_cycle_low_pressure;
error_flag __oe_error_ebox_over_temperature;
error_flag __oe_error_cooling_cycle_over_temperature;
error_flag __oe_warn_invalid_position_estimation;
error_flag __oe_warn_invalid_position;
error_flag __oe_warn_invalid_velocity_profile;
error_flag __oe_warn_invalid_acceleration_profile;
error_flag __oe_warn_battery_over_temperature;
error_flag __oe_warn_cooling_cycle_over_pressure;
error_flag __oe_warn_cooling_cycle_low_pressure;
error_flag __oe_warn_input_board_mcu_over_temperature;
error_flag __oe_warn_ebox_over_temperature;
error_flag __oe_warn_cooling_cycle_over_temperature;
sdc_status __oe_input_board_sdc_status;
pdu_channel_status __oe_pdu12_lp_channel1_status;
pdu_channel_status __oe_pdu12_lp_channel2_status;
pdu_channel_status __oe_pdu12_lp_channel3_status;
pdu_channel_status __oe_pdu12_lp_channel4_status;
pdu_channel_status __oe_pdu12_lp_channel5_status;
pdu_channel_status __oe_pdu12_lp_channel6_status;
pdu_channel_status __oe_pdu12_lp_channel7_status;
pdu_channel_status __oe_pdu12_lp_channel8_status;
pdu_channel_status __oe_pdu12_lp_chanel9_status;
pdu_channel_status __oe_pdu12_lp_channel10_status;
pdu_channel_status __oe_pdu12_lp_channel11_status;
uint8_t __oe_pdu12_lp_channel12_status;
pdu_channel_status __oe_pdu12_hp_channel1_status;
pdu_channel_status __oe_pdu12_hp_channel2_status;
pdu_channel_status __oe_pdu12_hp_channel3_status;
pdu_channel_status __oe_pdu12_hp_channel4_status;
sdc_status __oe_pdu12_sdc_status;
error_flag __oe_error_pdu12_mcu_over_temperature;
float __oe_pdu12_power_estimation;
pdu_channel_status __oe_pdu24_lp_channel1_status;
pdu_channel_status __oe_pdu24_lp_channel2_status;
pdu_channel_status __oe_pdu24_lp_channel3_status;
pdu_channel_status __oe_pdu24_lp_channel4_status;
pdu_channel_status __oe_pdu24_lp_channel5_status;
pdu_channel_status __oe_pdu24_lp_channel6_status;
pdu_channel_status __oe_pdu24_lp_channel7_status;
pdu_channel_status __oe_pdu24_lp_channel8_status;
pdu_channel_status __oe_pdu24_lp_chanel9_status;
pdu_channel_status __oe_pdu24_lp_channel10_status;
pdu_channel_status __oe_pdu24_lp_channel11_status;
uint8_t __oe_pdu24_lp_channel12_status;
pdu_channel_status __oe_pdu24_hp_channel1_status;
pdu_channel_status __oe_pdu24_hp_channel2_status;
pdu_channel_status __oe_pdu24_hp_channel3_status;
pdu_channel_status __oe_pdu24_hp_channel4_status;
sdc_status __oe_pdu24_sdc_status;
error_flag __oe_error_pdu24_mcu_over_temperature;
float __oe_pdu24_power_estimation;
error_flag __oe_error_motor_driver_45V_over_voltage;
error_flag __oe_error_motor_driver_45V_under_voltage;
error_flag __oe_error_dslim_over_temperature;
error_flag __oe_error_motor_driver_mosfet_over_temperature;
error_flag __oe_error_motor_control_failure;
error_flag __oe_error_motor_mcu_over_temperature;
error_flag __oe_warn_motor_dslim_over_temperature;
error_flag __oe_warn_motor_mosfet_over_temperature;
error_flag __oe_warn_motor_mcu_over_temperature;
float __oe_motor_power_estimation;
motor_state __oe_motor_state;
sdc_status __oe_motor_sdc_status;
error_flag __oe_error_mgu1_45V_over_voltage;
error_flag __oe_error_mgu1_45V_under_voltage;
error_flag __oe_error_mgu1_control_error;
error_flag __oe_error_mgu1_magnet_over_temperature_starboard;
error_flag __oe_error_mgu1_magnet_over_temperature_port;
error_flag __oe_error_mgu1_mosfet_over_temperature;
error_flag __oe_error_mgu1_mcu_over_temperature;
error_flag __oe_warn_mgu1_magnet_over_temperature_starboard;
error_flag __oe_warn_mgu1_magnet_over_temperature_port;
error_flag __oe_warn_mgu1_mosfet_over_temperature;
error_flag __oe_warn_mgu1_mcu_over_temperature;
mgu_state __oe_mgu1_state;
float __oe_mgu1_power_estimation;
sdc_status __oe_mgu1_sdc_status;
error_flag __oe_error_mgu2_45V_over_voltage;
error_flag __oe_error_mgu2_45V_under_voltage;
error_flag __oe_error_mgu2_control_error;
error_flag __oe_error_mgu2_magnet_over_temperature_starboard;
error_flag __oe_error_mgu2_magnet_over_temperature_port;
error_flag __oe_error_mgu2_mosfet_over_temperature;
error_flag __oe_error_mgu2_mcu_over_temperature;
error_flag __oe_warn_mgu2_magnet_over_temperature_starboard;
error_flag __oe_warn_mgu2_magnet_over_temperature_port;
error_flag __oe_warn_mgu2_mosfet_over_temperature;
error_flag __oe_warn_mgu2_mcu_over_temperature;
mgu_state __oe_mgu2_state;
float __oe_mgu2_power_estimation;
sdc_status __oe_mgu2_sdc_status;
error_flag __oe_error_mlu1_45V_over_voltage;
error_flag __oe_error_mlu1_45V_under_voltage;
error_flag __oe_error_mlu1_magnet_over_temperature;
error_flag __oe_error_mlu1_mcu_over_temperature;
error_flag __oe_error_mlu1_mosfet_over_temperature;
error_flag __oe_error_mlu1_control_failure;
error_flag __oe_warn_mlu1_magnet_over_temperature;
error_flag __oe_warn_mlu1_mcu_over_temperature;
error_flag __oe_warn_mlu1_mosfet_over_temperature;
mlu_state __oe_mlu1_state;
float __oe_mlu1_power_estimation;
sdc_status __oe_mlu1_sdc_status;
error_flag __oe_error_mlu2_45V_over_voltage;
error_flag __oe_error_mlu2_45V_under_voltage;
error_flag __oe_error_mlu2_magnet_over_temperature;
error_flag __oe_error_mlu2_mcu_over_temperature;
error_flag __oe_error_mlu2_mosfet_over_temperature;
error_flag __oe_error_mlu2_control_failure;
error_flag __oe_warn_mlu2_magnet_over_temperature;
error_flag __oe_warn_mlu2_mcu_over_temperature;
error_flag __oe_warn_mlu2_mosfet_over_temperature;
mlu_state __oe_mlu2_state;
float __oe_mlu2_power_estimation;
sdc_status __oe_mlu2_sdc_status;
error_flag __oe_error_mlu3_45V_over_voltage;
error_flag __oe_error_mlu3_45V_under_voltage;
error_flag __oe_error_mlu3_magnet_over_temperature;
error_flag __oe_error_mlu3_mcu_over_temperature;
error_flag __oe_error_mlu3_mosfet_over_temperature;
error_flag __oe_error_mlu3_control_failure;
error_flag __oe_warn_mlu3_magnet_over_temperature;
error_flag __oe_warn_mlu3_mcu_over_temperature;
error_flag __oe_warn_mlu3_mosfet_over_temperature;
mlu_state __oe_mlu3_state;
float __oe_mlu3_power_estimation;
sdc_status __oe_mlu3_sdc_status;
error_flag __oe_error_mlu4_45V_over_voltage;
error_flag __oe_error_mlu4_45V_under_voltage;
error_flag __oe_error_mlu4_magnet_over_temperature;
error_flag __oe_error_mlu4_mcu_over_temperature;
error_flag __oe_error_mlu4_mosfet_over_temperature;
error_flag __oe_error_mlu4_control_failure;
error_flag __oe_warn_mlu4_magnet_over_temperature;
error_flag __oe_warn_mlu4_mcu_over_temperature;
error_flag __oe_warn_mlu4_mosfet_over_temperature;
mlu_state __oe_mlu4_state;
float __oe_mlu4_power_estimation;
sdc_status __oe_mlu4_sdc_status;
error_flag __oe_error_mlu5_45V_over_voltage;
error_flag __oe_error_mlu5_45V_under_voltage;
error_flag __oe_error_mlu5_magnet_over_temperature;
error_flag __oe_error_mlu5_mcu_over_temperature;
error_flag __oe_error_mlu5_mosfet_over_temperature;
error_flag __oe_error_mlu5_control_failure;
error_flag __oe_warn_mlu5_magnet_over_temperature;
error_flag __oe_warn_mlu5_mcu_over_temperature;
error_flag __oe_warn_mlu5_mosfet_over_temperature;
mlu_state __oe_mlu5_state;
float __oe_mlu5_power_estimation;
sdc_status __oe_mlu5_sdc_status;
error_flag __oe_error_mlu6_45V_over_voltage;
error_flag __oe_error_mlu6_45V_under_voltage;
error_flag __oe_error_mlu6_magnet_over_temperature;
error_flag __oe_error_mlu6_mcu_over_temperature;
error_flag __oe_error_mlu6_mosfet_over_temperature;
error_flag __oe_error_mlu6_control_failure;
error_flag __oe_warn_mlu6_magnet_over_temperature;
error_flag __oe_warn_mlu6_mcu_over_temperature;
error_flag __oe_warn_mlu6_mosfet_over_temperature;
mlu_state __oe_mlu6_state;
float __oe_mlu6_power_estimation;
sdc_status __oe_mlu6_sdc_status;

typedef enum {
  HEARTBEAT_JOB_TAG = 0,
  GET_RESP_FRAGMENTATION_JOB_TAG = 1,
  STREAM_INTERVAL_JOB_TAG = 2,
} job_tag;
typedef struct {
  uint32_t *buffer;
  uint8_t offset;
  uint8_t size;
  uint8_t od_index;
  uint8_t server_id;
} get_resp_fragmentation_job;
typedef struct {
  uint32_t command_resp_msg_id;
  uint8_t bus_id;
} command_resp_timeout_job;
typedef struct {
  uint32_t last_schedule; 
  uint32_t stream_id;
} stream_interval_job;
typedef struct {
  uint32_t climax;
  uint32_t position;
  job_tag tag;
  union {
    get_resp_fragmentation_job get_fragmentation_job;
    stream_interval_job stream_interval_job;
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
static void schedule_get_resp_fragmentation_job(uint32_t *fragmentation_buffer, uint8_t size, uint8_t od_index, uint8_t server_id) {
  job_t *fragmentation_job = job_pool_allocator_alloc();
  fragmentation_job->climax = canzero_get_time() + get_resp_fragmentation_interval;
  fragmentation_job->tag = GET_RESP_FRAGMENTATION_JOB_TAG;
  fragmentation_job->job.get_fragmentation_job.buffer = fragmentation_buffer;
  fragmentation_job->job.get_fragmentation_job.offset = 1;
  fragmentation_job->job.get_fragmentation_job.size = size;
  fragmentation_job->job.get_fragmentation_job.od_index = od_index;
  fragmentation_job->job.get_fragmentation_job.server_id = server_id;
  scheduler_schedule(fragmentation_job);
}
static job_t heartbeat_job;
static const uint32_t heartbeat_interval = 100;
static void schedule_heartbeat_job() {
  heartbeat_job.climax = canzero_get_time() + heartbeat_interval;
  heartbeat_job.tag = HEARTBEAT_JOB_TAG;
  scheduler_schedule(&heartbeat_job);
}
static job_t mlu_control_interval_job;
static const uint32_t mlu_control_interval = 0;
static void schedule_mlu_control_interval_job(){
  uint32_t time = canzero_get_time();
  mlu_control_interval_job.climax = time + mlu_control_interval;
  mlu_control_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  mlu_control_interval_job.job.stream_interval_job.stream_id = 0;
  mlu_control_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&mlu_control_interval_job);
}
static job_t mgu_control_interval_job;
static const uint32_t mgu_control_interval = 0;
static void schedule_mgu_control_interval_job(){
  uint32_t time = canzero_get_time();
  mgu_control_interval_job.climax = time + mgu_control_interval;
  mgu_control_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  mgu_control_interval_job.job.stream_interval_job.stream_id = 1;
  mgu_control_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&mgu_control_interval_job);
}
static job_t motor_control_interval_job;
static const uint32_t motor_control_interval = 0;
static void schedule_motor_control_interval_job(){
  uint32_t time = canzero_get_time();
  motor_control_interval_job.climax = time + motor_control_interval;
  motor_control_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  motor_control_interval_job.job.stream_interval_job.stream_id = 2;
  motor_control_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&motor_control_interval_job);
}
static job_t pdu24_control_interval_job;
static const uint32_t pdu24_control_interval = 0;
static void schedule_pdu24_control_interval_job(){
  uint32_t time = canzero_get_time();
  pdu24_control_interval_job.climax = time + pdu24_control_interval;
  pdu24_control_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  pdu24_control_interval_job.job.stream_interval_job.stream_id = 3;
  pdu24_control_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&pdu24_control_interval_job);
}
static job_t pdu12_control_interval_job;
static const uint32_t pdu12_control_interval = 0;
static void schedule_pdu12_control_interval_job(){
  uint32_t time = canzero_get_time();
  pdu12_control_interval_job.climax = time + pdu12_control_interval;
  pdu12_control_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  pdu12_control_interval_job.job.stream_interval_job.stream_id = 4;
  pdu12_control_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&pdu12_control_interval_job);
}
static job_t global_state_interval_job;
static const uint32_t global_state_interval = 0;
static void schedule_global_state_interval_job(){
  uint32_t time = canzero_get_time();
  global_state_interval_job.climax = time + global_state_interval;
  global_state_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  global_state_interval_job.job.stream_interval_job.stream_id = 5;
  global_state_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&global_state_interval_job);
}
static job_t mlu_pid_sync_interval_job;
static const uint32_t mlu_pid_sync_interval = 0;
static void schedule_mlu_pid_sync_interval_job(){
  uint32_t time = canzero_get_time();
  mlu_pid_sync_interval_job.climax = time + mlu_pid_sync_interval;
  mlu_pid_sync_interval_job.tag = STREAM_INTERVAL_JOB_TAG;
  mlu_pid_sync_interval_job.job.stream_interval_job.stream_id = 6;
  mlu_pid_sync_interval_job.job.stream_interval_job.last_schedule = time;
  scheduler_schedule(&mlu_pid_sync_interval_job);
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
      switch (job->job.stream_interval_job.stream_id) {
      case 0: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_master_stream_mlu_control stream_message;
        stream_message.mlu_command = __oe_mlu_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_master_stream_mlu_control(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 1: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_master_stream_mgu_control stream_message;
        stream_message.mgu_command = __oe_mgu_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_master_stream_mgu_control(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 2: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_master_stream_motor_control stream_message;
        stream_message.motor_command = __oe_motor_command;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_master_stream_motor_control(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 3: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_master_stream_pdu24_control stream_message;
        stream_message.pdu24_lp_channel1_control = __oe_pdu24_lp_channel1_control;
        stream_message.pdu24_lp_channel2_control = __oe_pdu24_lp_channel2_control;
        stream_message.pdu24_lp_channel3_control = __oe_pdu24_lp_channel3_control;
        stream_message.pdu24_lp_channel4_control = __oe_pdu24_lp_channel4_control;
        stream_message.pdu24_lp_channel5_control = __oe_pdu24_lp_channel5_control;
        stream_message.pdu24_lp_channel6_control = __oe_pdu24_lp_channel6_control;
        stream_message.pdu24_lp_channel7_control = __oe_pdu24_lp_channel7_control;
        stream_message.pdu24_lp_channel8_control = __oe_pdu24_lp_channel8_control;
        stream_message.pdu24_lp_channel9_control = __oe_pdu24_lp_channel9_control;
        stream_message.pdu24_lp_channel10_control = __oe_pdu24_lp_channel10_control;
        stream_message.pdu24_lp_channel11_control = __oe_pdu24_lp_channel11_control;
        stream_message.pdu24_lp_channel12_control = __oe_pdu24_lp_channel12_control;
        stream_message.pdu24_hp_channel1_control = __oe_pdu24_hp_channel1_control;
        stream_message.pdu24_hp_channel2_control = __oe_pdu24_hp_channel2_control;
        stream_message.pdu24_hp_channel3_control = __oe_pdu24_hp_channel3_control;
        stream_message.pdu24_hp_channel4_control = __oe_pdu24_hp_channel4_control;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_master_stream_pdu24_control(&stream_message, &stream_frame);
        canzero_can0_send(&stream_frame);
        break;
      }
      case 4: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_master_stream_pdu12_control stream_message;
        stream_message.pdu12_lp_channel1_control = __oe_pdu12_lp_channel1_control;
        stream_message.pdu12_lp_channel2_control = __oe_pdu12_lp_channel2_control;
        stream_message.pdu12_lp_channel3_control = __oe_pdu12_lp_channel3_control;
        stream_message.pdu12_lp_channel4_control = __oe_pdu12_lp_channel4_control;
        stream_message.pdu12_lp_channel5_control = __oe_pdu12_lp_channel5_control;
        stream_message.pdu12_lp_channel6_control = __oe_pdu12_lp_channel6_control;
        stream_message.pdu12_lp_channel7_control = __oe_pdu12_lp_channel7_control;
        stream_message.pdu12_lp_channel8_control = __oe_pdu12_lp_channel8_control;
        stream_message.pdu12_lp_channel9_control = __oe_pdu12_lp_channel9_control;
        stream_message.pdu12_lp_channel10_control = __oe_pdu12_lp_channel10_control;
        stream_message.pdu12_lp_channel11_control = __oe_pdu12_lp_channel11_control;
        stream_message.pdu12_lp_channel12_control = __oe_pdu12_lp_channel12_control;
        stream_message.pdu12_hp_channel1_control = __oe_pdu12_hp_channel1_control;
        stream_message.pdu12_hp_channel2_control = __oe_pdu12_hp_channel2_control;
        stream_message.pdu12_hp_channel3_control = __oe_pdu12_hp_channel3_control;
        stream_message.pdu12_hp_channel4_control = __oe_pdu12_hp_channel4_control;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_master_stream_pdu12_control(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 5: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 500);
        canzero_exit_critical();
        canzero_message_master_stream_global_state stream_message;
        stream_message.global_state = __oe_global_state;
        stream_message.command = __oe_command;
        stream_message.sdc_status = __oe_sdc_status;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_master_stream_global_state(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
        break;
      }
      case 6: {
        job->job.stream_interval_job.last_schedule = time;
        scheduler_reschedule(time + 10000);
        canzero_exit_critical();
        canzero_message_master_stream_mlu_pid_sync stream_message;
        stream_message.mlu_pid_values = __oe_mlu_pid_values;
        canzero_frame stream_frame;
        canzero_serialize_canzero_message_master_stream_mlu_pid_sync(&stream_message, &stream_frame);
        canzero_can1_send(&stream_frame);
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
      heartbeat.node_id = node_id_master;
      canzero_frame heartbeat_frame;
      canzero_serialize_canzero_message_heartbeat(&heartbeat, &heartbeat_frame);
      canzero_can0_send(&heartbeat_frame);
      break;
    }
    case GET_RESP_FRAGMENTATION_JOB_TAG: {
      get_resp_fragmentation_job *fragmentation_job = &job->job.get_fragmentation_job;
      canzero_message_get_resp fragmentation_response;
      fragmentation_response.header.sof = 0;
      fragmentation_response.header.toggle = fragmentation_job->offset % 2;
      fragmentation_response.header.od_index = fragmentation_job->od_index;
      fragmentation_response.header.client_id = 0x1;
      fragmentation_response.header.server_id = fragmentation_job->server_id;
      fragmentation_response.data = fragmentation_job->buffer[fragmentation_job->offset];
      fragmentation_job->offset += 1;
      if (fragmentation_job->offset == fragmentation_job->size) {
        fragmentation_response.header.eof = 1;
        scheduler_unschedule();
      } else {
        fragmentation_response.header.eof = 0;
        scheduler_reschedule(time + get_resp_fragmentation_interval);
      }
      canzero_exit_critical();
      canzero_frame fragmentation_frame;
      canzero_serialize_canzero_message_get_resp(&fragmentation_response, &fragmentation_frame);
      canzero_can1_send(&fragmentation_frame);
      break;
    }
    default:
      canzero_exit_critical();
      break;
    }
  }
}
static uint32_t scheduler_next_job_timeout(){
  return scheduler.heap[0]->climax;
}
static uint32_t __oe_mlu_pid_values_rx_fragmentation_buffer[2];
static uint32_t __oe_mgu_pid_values_rx_fragmentation_buffer[2];
static void canzero_handle_get_req(canzero_frame* frame) {
  canzero_message_get_req msg;
  canzero_deserialize_canzero_message_get_req(frame, &msg);
  if (msg.header.server_id != 1) {
    return;
  }
  canzero_message_get_resp resp;
  switch (msg.header.od_index) {
  case 0: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu_command) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 1: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mgu_command) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 2: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_motor_command) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 3: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel1_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 4: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel2_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 5: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel3_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 6: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel4_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 7: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel5_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 8: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel6_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 9: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel7_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 10: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel8_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 11: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel9_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 12: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel10_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 13: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel11_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 14: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel12_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 15: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_hp_channel1_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 16: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_hp_channel2_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 17: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_hp_channel3_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 18: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_hp_channel4_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 19: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel1_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 20: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel2_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 21: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel3_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 22: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel4_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 23: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel5_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 24: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel6_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 25: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel7_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 26: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel8_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 27: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel9_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 28: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel10_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 29: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel11_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 30: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel12_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 31: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_hp_channel1_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 32: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_hp_channel2_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 33: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_hp_channel3_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 34: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_hp_channel4_control) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 35: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_global_state) & (0xFF >> (8 - 4)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 36: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_command) & (0xFF >> (8 - 4)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 37: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_sync_mlu_pid_values) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 38: {
    __oe_mlu_pid_values_rx_fragmentation_buffer[0] = (((uint32_t)((__oe_mlu_pid_values.p_value - ((float)-100)) / (float)0.00009536747711538177)) & (0xFFFFFFFF >> (32 - 21)));
    __oe_mlu_pid_values_rx_fragmentation_buffer[0] = (((uint32_t)((__oe_mlu_pid_values.i_value - ((float)-100)) / (float)0.00009536747711538177)) & (0xFFFFFFFF >> (32 - 21)));
    __oe_mlu_pid_values_rx_fragmentation_buffer[0] = (((uint32_t)((__oe_mlu_pid_values.d_value - ((float)-100)) / (float)0.00009536747711538177)) & (0xFFFFFFFF >> (32 - 21)));

    resp.data = __oe_mlu_pid_values_rx_fragmentation_buffer[0];
    resp.header.sof = 1;
    resp.header.eof = 0;
    resp.header.toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_mlu_pid_values_rx_fragmentation_buffer, 2, 38, msg.header.server_id);
    break;
  }
  case 39: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_sync_mgu_pid_values) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 40: {
    __oe_mgu_pid_values_rx_fragmentation_buffer[0] = (((uint32_t)((__oe_mgu_pid_values.p_value - ((float)-100)) / (float)0.00009536747711538177)) & (0xFFFFFFFF >> (32 - 21)));
    __oe_mgu_pid_values_rx_fragmentation_buffer[0] = (((uint32_t)((__oe_mgu_pid_values.i_value - ((float)-100)) / (float)0.00009536747711538177)) & (0xFFFFFFFF >> (32 - 21)));
    __oe_mgu_pid_values_rx_fragmentation_buffer[0] = (((uint32_t)((__oe_mgu_pid_values.d_value - ((float)-100)) / (float)0.00009536747711538177)) & (0xFFFFFFFF >> (32 - 21)));

    resp.data = __oe_mgu_pid_values_rx_fragmentation_buffer[0];
    resp.header.sof = 1;
    resp.header.eof = 0;
    resp.header.toggle = 0;
    schedule_get_resp_fragmentation_job(__oe_mgu_pid_values_rx_fragmentation_buffer, 2, 40, msg.header.server_id);
    break;
  }
  case 41: {
    resp.data |= ((uint32_t)((__oe_cpu_temperature - (-1)) / 0.592156862745098)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 42: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 43: {
    resp.data |= ((uint32_t)((__oe_power_estimation - (0)) / 0.0000011641532185403987)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 44: {
    resp.data |= ((uint32_t)((__oe_power_estimation24 - (0)) / 0.0000011641532185403987)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 45: {
    resp.data |= ((uint32_t)((__oe_power_estimation48 - (0)) / 0.0000011641532185403987)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 46: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_state) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 47: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_state) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 48: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_input_board_state) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 49: {
    resp.data |= ((uint32_t)((__oe_position - (-100)) / 0.0030518043793392844)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 50: {
    resp.data |= ((uint32_t)((__oe_velocity - (-10)) / 0.00030518043793392844)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 51: {
    resp.data |= ((uint32_t)((__oe_acceleration - (-5)) / 0.00008392462043183032)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 52: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_input_board_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 53: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_invalid_position_estimation) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 54: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_invalid_position) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 55: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_invalid_velocity_profile) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 56: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_invalid_acceleration_profile) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 57: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_battery_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 58: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_cooling_cycle_over_pressure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 59: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_cooling_cycle_low_pressure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 60: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_ebox_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 61: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_cooling_cycle_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 62: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_invalid_position_estimation) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 63: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_invalid_position) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 64: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_invalid_velocity_profile) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 65: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_invalid_acceleration_profile) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 66: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_battery_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 67: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_cooling_cycle_over_pressure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 68: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_cooling_cycle_low_pressure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 69: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_input_board_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 70: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_ebox_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 71: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_cooling_cycle_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 72: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_input_board_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 73: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel1_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 74: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel2_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 75: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel3_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 76: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel4_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 77: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel5_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 78: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel6_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 79: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel7_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 80: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel8_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 81: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_chanel9_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 82: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel10_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 83: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_lp_channel11_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 84: {
    resp.data |= ((uint32_t)(__oe_pdu12_lp_channel12_status & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 85: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_hp_channel1_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 86: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_hp_channel2_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 87: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_hp_channel3_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 88: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_hp_channel4_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 89: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu12_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 90: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_pdu12_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 91: {
    resp.data |= ((uint32_t)((__oe_pdu12_power_estimation - (0)) / 0.030518043793392843)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 92: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel1_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 93: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel2_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 94: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel3_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 95: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel4_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 96: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel5_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 97: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel6_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 98: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel7_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 99: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel8_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 100: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_chanel9_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 101: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel10_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 102: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_lp_channel11_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 103: {
    resp.data |= ((uint32_t)(__oe_pdu24_lp_channel12_status & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 104: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_hp_channel1_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 105: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_hp_channel2_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 106: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_hp_channel3_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 107: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_hp_channel4_status) & (0xFF >> (8 - 2)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 108: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_pdu24_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 109: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_pdu24_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 110: {
    resp.data |= ((uint32_t)((__oe_pdu24_power_estimation - (0)) / 0.030518043793392843)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 111: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_motor_driver_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 112: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_motor_driver_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 113: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_dslim_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 114: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_motor_driver_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 115: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_motor_control_failure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 116: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_motor_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 117: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_motor_dslim_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 118: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_motor_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 119: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_motor_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 120: {
    resp.data |= ((uint32_t)((__oe_motor_power_estimation - (0)) / 0.030518043793392843)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 121: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_motor_state) & (0xFF >> (8 - 4)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 122: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_motor_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 123: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu1_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 124: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu1_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 125: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu1_control_error) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 126: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu1_magnet_over_temperature_starboard) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 127: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu1_magnet_over_temperature_port) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 128: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu1_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 129: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu1_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 130: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mgu1_magnet_over_temperature_starboard) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 131: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mgu1_magnet_over_temperature_port) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 132: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mgu1_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 133: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mgu1_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 134: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mgu1_state) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 135: {
    resp.data |= ((uint32_t)((__oe_mgu1_power_estimation - (0)) / 0.015259021896696421)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 136: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mgu1_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 137: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu2_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 138: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu2_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 139: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu2_control_error) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 140: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu2_magnet_over_temperature_starboard) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 141: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu2_magnet_over_temperature_port) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 142: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu2_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 143: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mgu2_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 144: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mgu2_magnet_over_temperature_starboard) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 145: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mgu2_magnet_over_temperature_port) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 146: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mgu2_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 147: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mgu2_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 148: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mgu2_state) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 149: {
    resp.data |= ((uint32_t)((__oe_mgu2_power_estimation - (0)) / 0.015259021896696421)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 150: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mgu2_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 151: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu1_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 152: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu1_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 153: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu1_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 154: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu1_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 155: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu1_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 156: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu1_control_failure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 157: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu1_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 158: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu1_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 159: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu1_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 160: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu1_state) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 161: {
    resp.data |= ((uint32_t)((__oe_mlu1_power_estimation - (0)) / 0.015259021896696421)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 162: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu1_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 163: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu2_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 164: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu2_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 165: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu2_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 166: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu2_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 167: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu2_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 168: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu2_control_failure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 169: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu2_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 170: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu2_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 171: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu2_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 172: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu2_state) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 173: {
    resp.data |= ((uint32_t)((__oe_mlu2_power_estimation - (0)) / 0.015259021896696421)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 174: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu2_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 175: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu3_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 176: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu3_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 177: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu3_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 178: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu3_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 179: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu3_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 180: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu3_control_failure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 181: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu3_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 182: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu3_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 183: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu3_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 184: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu3_state) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 185: {
    resp.data |= ((uint32_t)((__oe_mlu3_power_estimation - (0)) / 0.015259021896696421)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 186: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu3_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 187: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu4_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 188: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu4_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 189: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu4_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 190: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu4_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 191: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu4_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 192: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu4_control_failure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 193: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu4_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 194: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu4_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 195: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu4_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 196: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu4_state) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 197: {
    resp.data |= ((uint32_t)((__oe_mlu4_power_estimation - (0)) / 0.015259021896696421)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 198: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu4_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 199: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu5_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 200: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu5_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 201: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu5_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 202: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu5_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 203: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu5_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 204: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu5_control_failure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 205: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu5_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 206: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu5_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 207: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu5_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 208: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu5_state) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 209: {
    resp.data |= ((uint32_t)((__oe_mlu5_power_estimation - (0)) / 0.015259021896696421)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 210: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu5_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 211: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu6_45V_over_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 212: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu6_45V_under_voltage) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 213: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu6_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 214: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu6_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 215: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu6_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 216: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_error_mlu6_control_failure) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 217: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu6_magnet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 218: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu6_mcu_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 219: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_warn_mlu6_mosfet_over_temperature) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 220: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu6_state) & (0xFF >> (8 - 3)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 221: {
    resp.data |= ((uint32_t)((__oe_mlu6_power_estimation - (0)) / 0.015259021896696421)) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  case 222: {
    resp.data |= ((uint32_t)(((uint8_t)__oe_mlu6_sdc_status) & (0xFF >> (8 - 1)))) << 0;
    resp.header.sof = 1;
    resp.header.eof = 1;
    resp.header.toggle = 0;
    break;
  }
  }
  resp.header.od_index = msg.header.od_index;
  resp.header.client_id = msg.header.client_id;
  resp.header.server_id = msg.header.server_id;
  canzero_frame resp_frame;
  canzero_serialize_canzero_message_get_resp(&resp, &resp_frame);
  canzero_can1_send(&resp_frame);
}
static uint32_t mlu_pid_values_tmp_tx_fragmentation_buffer[2];
static uint32_t mlu_pid_values_tmp_tx_fragmentation_offset = 0;
static uint32_t mgu_pid_values_tmp_tx_fragmentation_buffer[2];
static uint32_t mgu_pid_values_tmp_tx_fragmentation_offset = 0;
static void canzero_handle_set_req(canzero_frame* frame) {
  canzero_message_set_req msg;
  canzero_deserialize_canzero_message_set_req(frame, &msg);
  if (msg.header.server_id != 1) {
    return;
  }
  canzero_message_set_resp resp;
  switch (msg.header.od_index) {
  case 0 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mlu_command mlu_command_tmp;
    mlu_command_tmp = (mlu_command)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mlu_command(mlu_command_tmp);
    break;
  }
  case 1 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mgu_command mgu_command_tmp;
    mgu_command_tmp = (mgu_command)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mgu_command(mgu_command_tmp);
    break;
  }
  case 2 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    motor_command motor_command_tmp;
    motor_command_tmp = (motor_command)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_motor_command(motor_command_tmp);
    break;
  }
  case 3 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel1_control_tmp;
    pdu24_lp_channel1_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel1_control(pdu24_lp_channel1_control_tmp);
    break;
  }
  case 4 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel2_control_tmp;
    pdu24_lp_channel2_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel2_control(pdu24_lp_channel2_control_tmp);
    break;
  }
  case 5 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel3_control_tmp;
    pdu24_lp_channel3_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel3_control(pdu24_lp_channel3_control_tmp);
    break;
  }
  case 6 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel4_control_tmp;
    pdu24_lp_channel4_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel4_control(pdu24_lp_channel4_control_tmp);
    break;
  }
  case 7 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel5_control_tmp;
    pdu24_lp_channel5_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel5_control(pdu24_lp_channel5_control_tmp);
    break;
  }
  case 8 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel6_control_tmp;
    pdu24_lp_channel6_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel6_control(pdu24_lp_channel6_control_tmp);
    break;
  }
  case 9 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel7_control_tmp;
    pdu24_lp_channel7_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel7_control(pdu24_lp_channel7_control_tmp);
    break;
  }
  case 10 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel8_control_tmp;
    pdu24_lp_channel8_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel8_control(pdu24_lp_channel8_control_tmp);
    break;
  }
  case 11 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel9_control_tmp;
    pdu24_lp_channel9_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel9_control(pdu24_lp_channel9_control_tmp);
    break;
  }
  case 12 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel10_control_tmp;
    pdu24_lp_channel10_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel10_control(pdu24_lp_channel10_control_tmp);
    break;
  }
  case 13 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel11_control_tmp;
    pdu24_lp_channel11_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel11_control(pdu24_lp_channel11_control_tmp);
    break;
  }
  case 14 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_lp_channel12_control_tmp;
    pdu24_lp_channel12_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_lp_channel12_control(pdu24_lp_channel12_control_tmp);
    break;
  }
  case 15 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_hp_channel1_control_tmp;
    pdu24_hp_channel1_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_hp_channel1_control(pdu24_hp_channel1_control_tmp);
    break;
  }
  case 16 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_hp_channel2_control_tmp;
    pdu24_hp_channel2_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_hp_channel2_control(pdu24_hp_channel2_control_tmp);
    break;
  }
  case 17 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_hp_channel3_control_tmp;
    pdu24_hp_channel3_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_hp_channel3_control(pdu24_hp_channel3_control_tmp);
    break;
  }
  case 18 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu24_hp_channel4_control_tmp;
    pdu24_hp_channel4_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_hp_channel4_control(pdu24_hp_channel4_control_tmp);
    break;
  }
  case 19 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel1_control_tmp;
    pdu12_lp_channel1_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel1_control(pdu12_lp_channel1_control_tmp);
    break;
  }
  case 20 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel2_control_tmp;
    pdu12_lp_channel2_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel2_control(pdu12_lp_channel2_control_tmp);
    break;
  }
  case 21 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel3_control_tmp;
    pdu12_lp_channel3_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel3_control(pdu12_lp_channel3_control_tmp);
    break;
  }
  case 22 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel4_control_tmp;
    pdu12_lp_channel4_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel4_control(pdu12_lp_channel4_control_tmp);
    break;
  }
  case 23 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel5_control_tmp;
    pdu12_lp_channel5_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel5_control(pdu12_lp_channel5_control_tmp);
    break;
  }
  case 24 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel6_control_tmp;
    pdu12_lp_channel6_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel6_control(pdu12_lp_channel6_control_tmp);
    break;
  }
  case 25 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel7_control_tmp;
    pdu12_lp_channel7_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel7_control(pdu12_lp_channel7_control_tmp);
    break;
  }
  case 26 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel8_control_tmp;
    pdu12_lp_channel8_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel8_control(pdu12_lp_channel8_control_tmp);
    break;
  }
  case 27 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel9_control_tmp;
    pdu12_lp_channel9_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel9_control(pdu12_lp_channel9_control_tmp);
    break;
  }
  case 28 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel10_control_tmp;
    pdu12_lp_channel10_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel10_control(pdu12_lp_channel10_control_tmp);
    break;
  }
  case 29 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel11_control_tmp;
    pdu12_lp_channel11_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel11_control(pdu12_lp_channel11_control_tmp);
    break;
  }
  case 30 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_lp_channel12_control_tmp;
    pdu12_lp_channel12_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_lp_channel12_control(pdu12_lp_channel12_control_tmp);
    break;
  }
  case 31 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_hp_channel1_control_tmp;
    pdu12_hp_channel1_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_hp_channel1_control(pdu12_hp_channel1_control_tmp);
    break;
  }
  case 32 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_hp_channel2_control_tmp;
    pdu12_hp_channel2_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_hp_channel2_control(pdu12_hp_channel2_control_tmp);
    break;
  }
  case 33 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_hp_channel3_control_tmp;
    pdu12_hp_channel3_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_hp_channel3_control(pdu12_hp_channel3_control_tmp);
    break;
  }
  case 34 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_control pdu12_hp_channel4_control_tmp;
    pdu12_hp_channel4_control_tmp = (pdu_channel_control)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_hp_channel4_control(pdu12_hp_channel4_control_tmp);
    break;
  }
  case 35 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    global_state global_state_tmp;
    global_state_tmp = (global_state)((msg.data & (0xFFFFFFFF >> (32 - 4))));
    canzero_set_global_state(global_state_tmp);
    break;
  }
  case 36 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    global_command command_tmp;
    command_tmp = (global_command)((msg.data & (0xFFFFFFFF >> (32 - 4))));
    canzero_set_command(command_tmp);
    break;
  }
  case 37 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    bool_t sync_mlu_pid_values_tmp;
    sync_mlu_pid_values_tmp = (bool_t)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_sync_mlu_pid_values(sync_mlu_pid_values_tmp);
    break;
  }
  case 38 : {
    if (msg.header.sof == 1) {
      if (msg.header.toggle == 0 || msg.header.eof != 0) {
        return;
      }
      mlu_pid_values_tmp_tx_fragmentation_offset = 0;
    }else {
      mlu_pid_values_tmp_tx_fragmentation_offset += 1;
      if (mlu_pid_values_tmp_tx_fragmentation_offset >= 2) {
        return;
      }
    }
    mlu_pid_values_tmp_tx_fragmentation_buffer[mlu_pid_values_tmp_tx_fragmentation_offset] = msg.data;
    if (msg.header.eof == 0) {
      return;
    }
    mlu_pid_values mlu_pid_values_tmp;
    mlu_pid_values_tmp.p_value = ((mlu_pid_values_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 21)))) * 0.00009536747711538177 + -100;
    mlu_pid_values_tmp.i_value = ((mlu_pid_values_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 21)))) * 0.00009536747711538177 + -100;
    mlu_pid_values_tmp.d_value = ((mlu_pid_values_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 21)))) * 0.00009536747711538177 + -100;
    canzero_set_mlu_pid_values(mlu_pid_values_tmp);
    break;
  }
  case 39 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    bool_t sync_mgu_pid_values_tmp;
    sync_mgu_pid_values_tmp = (bool_t)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_sync_mgu_pid_values(sync_mgu_pid_values_tmp);
    break;
  }
  case 40 : {
    if (msg.header.sof == 1) {
      if (msg.header.toggle == 0 || msg.header.eof != 0) {
        return;
      }
      mgu_pid_values_tmp_tx_fragmentation_offset = 0;
    }else {
      mgu_pid_values_tmp_tx_fragmentation_offset += 1;
      if (mgu_pid_values_tmp_tx_fragmentation_offset >= 2) {
        return;
      }
    }
    mgu_pid_values_tmp_tx_fragmentation_buffer[mgu_pid_values_tmp_tx_fragmentation_offset] = msg.data;
    if (msg.header.eof == 0) {
      return;
    }
    mgu_pid_values mgu_pid_values_tmp;
    mgu_pid_values_tmp.p_value = ((mgu_pid_values_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 21)))) * 0.00009536747711538177 + -100;
    mgu_pid_values_tmp.i_value = ((mgu_pid_values_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 21)))) * 0.00009536747711538177 + -100;
    mgu_pid_values_tmp.d_value = ((mgu_pid_values_tmp_tx_fragmentation_buffer[0] & (0xFFFFFFFF >> (32 - 21)))) * 0.00009536747711538177 + -100;
    canzero_set_mgu_pid_values(mgu_pid_values_tmp);
    break;
  }
  case 41 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float cpu_temperature_tmp;
    cpu_temperature_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 8))) * 0.592156862745098 + -1);
    canzero_set_cpu_temperature(cpu_temperature_tmp);
    break;
  }
  case 42 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status sdc_status_tmp;
    sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_sdc_status(sdc_status_tmp);
    break;
  }
  case 43 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float power_estimation_tmp;
    power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 32))) * 0.0000011641532185403987 + 0);
    canzero_set_power_estimation(power_estimation_tmp);
    break;
  }
  case 44 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float power_estimation24_tmp;
    power_estimation24_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 32))) * 0.0000011641532185403987 + 0);
    canzero_set_power_estimation24(power_estimation24_tmp);
    break;
  }
  case 45 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float power_estimation48_tmp;
    power_estimation48_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 32))) * 0.0000011641532185403987 + 0);
    canzero_set_power_estimation48(power_estimation48_tmp);
    break;
  }
  case 46 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_state pdu12_state_tmp;
    pdu12_state_tmp = (pdu_state)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_state(pdu12_state_tmp);
    break;
  }
  case 47 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_state pdu24_state_tmp;
    pdu24_state_tmp = (pdu_state)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_state(pdu24_state_tmp);
    break;
  }
  case 48 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    input_board_state input_board_state_tmp;
    input_board_state_tmp = (input_board_state)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_input_board_state(input_board_state_tmp);
    break;
  }
  case 49 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float position_tmp;
    position_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.0030518043793392844 + -100);
    canzero_set_position(position_tmp);
    break;
  }
  case 50 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float velocity_tmp;
    velocity_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.00030518043793392844 + -10);
    canzero_set_velocity(velocity_tmp);
    break;
  }
  case 51 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float acceleration_tmp;
    acceleration_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.00008392462043183032 + -5);
    canzero_set_acceleration(acceleration_tmp);
    break;
  }
  case 52 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_input_board_mcu_over_temperature_tmp;
    error_input_board_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_input_board_mcu_over_temperature(error_input_board_mcu_over_temperature_tmp);
    break;
  }
  case 53 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_invalid_position_estimation_tmp;
    error_invalid_position_estimation_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_invalid_position_estimation(error_invalid_position_estimation_tmp);
    break;
  }
  case 54 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_invalid_position_tmp;
    error_invalid_position_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_invalid_position(error_invalid_position_tmp);
    break;
  }
  case 55 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_invalid_velocity_profile_tmp;
    error_invalid_velocity_profile_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_invalid_velocity_profile(error_invalid_velocity_profile_tmp);
    break;
  }
  case 56 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_invalid_acceleration_profile_tmp;
    error_invalid_acceleration_profile_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_invalid_acceleration_profile(error_invalid_acceleration_profile_tmp);
    break;
  }
  case 57 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_battery_over_voltage_tmp;
    error_battery_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_battery_over_voltage(error_battery_over_voltage_tmp);
    break;
  }
  case 58 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_cooling_cycle_over_pressure_tmp;
    error_cooling_cycle_over_pressure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_cooling_cycle_over_pressure(error_cooling_cycle_over_pressure_tmp);
    break;
  }
  case 59 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_cooling_cycle_low_pressure_tmp;
    error_cooling_cycle_low_pressure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_cooling_cycle_low_pressure(error_cooling_cycle_low_pressure_tmp);
    break;
  }
  case 60 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_ebox_over_temperature_tmp;
    error_ebox_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_ebox_over_temperature(error_ebox_over_temperature_tmp);
    break;
  }
  case 61 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_cooling_cycle_over_temperature_tmp;
    error_cooling_cycle_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_cooling_cycle_over_temperature(error_cooling_cycle_over_temperature_tmp);
    break;
  }
  case 62 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_invalid_position_estimation_tmp;
    warn_invalid_position_estimation_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_invalid_position_estimation(warn_invalid_position_estimation_tmp);
    break;
  }
  case 63 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_invalid_position_tmp;
    warn_invalid_position_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_invalid_position(warn_invalid_position_tmp);
    break;
  }
  case 64 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_invalid_velocity_profile_tmp;
    warn_invalid_velocity_profile_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_invalid_velocity_profile(warn_invalid_velocity_profile_tmp);
    break;
  }
  case 65 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_invalid_acceleration_profile_tmp;
    warn_invalid_acceleration_profile_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_invalid_acceleration_profile(warn_invalid_acceleration_profile_tmp);
    break;
  }
  case 66 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_battery_over_temperature_tmp;
    warn_battery_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_battery_over_temperature(warn_battery_over_temperature_tmp);
    break;
  }
  case 67 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_cooling_cycle_over_pressure_tmp;
    warn_cooling_cycle_over_pressure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_cooling_cycle_over_pressure(warn_cooling_cycle_over_pressure_tmp);
    break;
  }
  case 68 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_cooling_cycle_low_pressure_tmp;
    warn_cooling_cycle_low_pressure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_cooling_cycle_low_pressure(warn_cooling_cycle_low_pressure_tmp);
    break;
  }
  case 69 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_input_board_mcu_over_temperature_tmp;
    warn_input_board_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_input_board_mcu_over_temperature(warn_input_board_mcu_over_temperature_tmp);
    break;
  }
  case 70 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_ebox_over_temperature_tmp;
    warn_ebox_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_ebox_over_temperature(warn_ebox_over_temperature_tmp);
    break;
  }
  case 71 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_cooling_cycle_over_temperature_tmp;
    warn_cooling_cycle_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_cooling_cycle_over_temperature(warn_cooling_cycle_over_temperature_tmp);
    break;
  }
  case 72 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status input_board_sdc_status_tmp;
    input_board_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_input_board_sdc_status(input_board_sdc_status_tmp);
    break;
  }
  case 73 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel1_status_tmp;
    pdu12_lp_channel1_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel1_status(pdu12_lp_channel1_status_tmp);
    break;
  }
  case 74 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel2_status_tmp;
    pdu12_lp_channel2_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel2_status(pdu12_lp_channel2_status_tmp);
    break;
  }
  case 75 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel3_status_tmp;
    pdu12_lp_channel3_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel3_status(pdu12_lp_channel3_status_tmp);
    break;
  }
  case 76 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel4_status_tmp;
    pdu12_lp_channel4_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel4_status(pdu12_lp_channel4_status_tmp);
    break;
  }
  case 77 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel5_status_tmp;
    pdu12_lp_channel5_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel5_status(pdu12_lp_channel5_status_tmp);
    break;
  }
  case 78 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel6_status_tmp;
    pdu12_lp_channel6_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel6_status(pdu12_lp_channel6_status_tmp);
    break;
  }
  case 79 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel7_status_tmp;
    pdu12_lp_channel7_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel7_status(pdu12_lp_channel7_status_tmp);
    break;
  }
  case 80 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel8_status_tmp;
    pdu12_lp_channel8_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel8_status(pdu12_lp_channel8_status_tmp);
    break;
  }
  case 81 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_chanel9_status_tmp;
    pdu12_lp_chanel9_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_chanel9_status(pdu12_lp_chanel9_status_tmp);
    break;
  }
  case 82 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel10_status_tmp;
    pdu12_lp_channel10_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel10_status(pdu12_lp_channel10_status_tmp);
    break;
  }
  case 83 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_lp_channel11_status_tmp;
    pdu12_lp_channel11_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_lp_channel11_status(pdu12_lp_channel11_status_tmp);
    break;
  }
  case 84 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    uint8_t pdu12_lp_channel12_status_tmp;
    pdu12_lp_channel12_status_tmp = (uint8_t)(msg.data & (0xFFFFFFFF >> (32 - 1)));
    canzero_set_pdu12_lp_channel12_status(pdu12_lp_channel12_status_tmp);
    break;
  }
  case 85 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_hp_channel1_status_tmp;
    pdu12_hp_channel1_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_hp_channel1_status(pdu12_hp_channel1_status_tmp);
    break;
  }
  case 86 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_hp_channel2_status_tmp;
    pdu12_hp_channel2_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_hp_channel2_status(pdu12_hp_channel2_status_tmp);
    break;
  }
  case 87 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_hp_channel3_status_tmp;
    pdu12_hp_channel3_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_hp_channel3_status(pdu12_hp_channel3_status_tmp);
    break;
  }
  case 88 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu12_hp_channel4_status_tmp;
    pdu12_hp_channel4_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu12_hp_channel4_status(pdu12_hp_channel4_status_tmp);
    break;
  }
  case 89 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status pdu12_sdc_status_tmp;
    pdu12_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu12_sdc_status(pdu12_sdc_status_tmp);
    break;
  }
  case 90 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_pdu12_mcu_over_temperature_tmp;
    error_pdu12_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_pdu12_mcu_over_temperature(error_pdu12_mcu_over_temperature_tmp);
    break;
  }
  case 91 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float pdu12_power_estimation_tmp;
    pdu12_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.030518043793392843 + 0);
    canzero_set_pdu12_power_estimation(pdu12_power_estimation_tmp);
    break;
  }
  case 92 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel1_status_tmp;
    pdu24_lp_channel1_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel1_status(pdu24_lp_channel1_status_tmp);
    break;
  }
  case 93 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel2_status_tmp;
    pdu24_lp_channel2_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel2_status(pdu24_lp_channel2_status_tmp);
    break;
  }
  case 94 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel3_status_tmp;
    pdu24_lp_channel3_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel3_status(pdu24_lp_channel3_status_tmp);
    break;
  }
  case 95 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel4_status_tmp;
    pdu24_lp_channel4_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel4_status(pdu24_lp_channel4_status_tmp);
    break;
  }
  case 96 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel5_status_tmp;
    pdu24_lp_channel5_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel5_status(pdu24_lp_channel5_status_tmp);
    break;
  }
  case 97 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel6_status_tmp;
    pdu24_lp_channel6_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel6_status(pdu24_lp_channel6_status_tmp);
    break;
  }
  case 98 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel7_status_tmp;
    pdu24_lp_channel7_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel7_status(pdu24_lp_channel7_status_tmp);
    break;
  }
  case 99 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel8_status_tmp;
    pdu24_lp_channel8_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel8_status(pdu24_lp_channel8_status_tmp);
    break;
  }
  case 100 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_chanel9_status_tmp;
    pdu24_lp_chanel9_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_chanel9_status(pdu24_lp_chanel9_status_tmp);
    break;
  }
  case 101 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel10_status_tmp;
    pdu24_lp_channel10_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel10_status(pdu24_lp_channel10_status_tmp);
    break;
  }
  case 102 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_lp_channel11_status_tmp;
    pdu24_lp_channel11_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_lp_channel11_status(pdu24_lp_channel11_status_tmp);
    break;
  }
  case 103 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    uint8_t pdu24_lp_channel12_status_tmp;
    pdu24_lp_channel12_status_tmp = (uint8_t)(msg.data & (0xFFFFFFFF >> (32 - 1)));
    canzero_set_pdu24_lp_channel12_status(pdu24_lp_channel12_status_tmp);
    break;
  }
  case 104 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_hp_channel1_status_tmp;
    pdu24_hp_channel1_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_hp_channel1_status(pdu24_hp_channel1_status_tmp);
    break;
  }
  case 105 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_hp_channel2_status_tmp;
    pdu24_hp_channel2_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_hp_channel2_status(pdu24_hp_channel2_status_tmp);
    break;
  }
  case 106 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_hp_channel3_status_tmp;
    pdu24_hp_channel3_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_hp_channel3_status(pdu24_hp_channel3_status_tmp);
    break;
  }
  case 107 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    pdu_channel_status pdu24_hp_channel4_status_tmp;
    pdu24_hp_channel4_status_tmp = (pdu_channel_status)((msg.data & (0xFFFFFFFF >> (32 - 2))));
    canzero_set_pdu24_hp_channel4_status(pdu24_hp_channel4_status_tmp);
    break;
  }
  case 108 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status pdu24_sdc_status_tmp;
    pdu24_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_pdu24_sdc_status(pdu24_sdc_status_tmp);
    break;
  }
  case 109 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_pdu24_mcu_over_temperature_tmp;
    error_pdu24_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_pdu24_mcu_over_temperature(error_pdu24_mcu_over_temperature_tmp);
    break;
  }
  case 110 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float pdu24_power_estimation_tmp;
    pdu24_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.030518043793392843 + 0);
    canzero_set_pdu24_power_estimation(pdu24_power_estimation_tmp);
    break;
  }
  case 111 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_motor_driver_45V_over_voltage_tmp;
    error_motor_driver_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_motor_driver_45V_over_voltage(error_motor_driver_45V_over_voltage_tmp);
    break;
  }
  case 112 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_motor_driver_45V_under_voltage_tmp;
    error_motor_driver_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_motor_driver_45V_under_voltage(error_motor_driver_45V_under_voltage_tmp);
    break;
  }
  case 113 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_dslim_over_temperature_tmp;
    error_dslim_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_dslim_over_temperature(error_dslim_over_temperature_tmp);
    break;
  }
  case 114 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_motor_driver_mosfet_over_temperature_tmp;
    error_motor_driver_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_motor_driver_mosfet_over_temperature(error_motor_driver_mosfet_over_temperature_tmp);
    break;
  }
  case 115 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_motor_control_failure_tmp;
    error_motor_control_failure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_motor_control_failure(error_motor_control_failure_tmp);
    break;
  }
  case 116 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_motor_mcu_over_temperature_tmp;
    error_motor_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_motor_mcu_over_temperature(error_motor_mcu_over_temperature_tmp);
    break;
  }
  case 117 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_motor_dslim_over_temperature_tmp;
    warn_motor_dslim_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_motor_dslim_over_temperature(warn_motor_dslim_over_temperature_tmp);
    break;
  }
  case 118 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_motor_mosfet_over_temperature_tmp;
    warn_motor_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_motor_mosfet_over_temperature(warn_motor_mosfet_over_temperature_tmp);
    break;
  }
  case 119 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_motor_mcu_over_temperature_tmp;
    warn_motor_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_motor_mcu_over_temperature(warn_motor_mcu_over_temperature_tmp);
    break;
  }
  case 120 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float motor_power_estimation_tmp;
    motor_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.030518043793392843 + 0);
    canzero_set_motor_power_estimation(motor_power_estimation_tmp);
    break;
  }
  case 121 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    motor_state motor_state_tmp;
    motor_state_tmp = (motor_state)((msg.data & (0xFFFFFFFF >> (32 - 4))));
    canzero_set_motor_state(motor_state_tmp);
    break;
  }
  case 122 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status motor_sdc_status_tmp;
    motor_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_motor_sdc_status(motor_sdc_status_tmp);
    break;
  }
  case 123 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu1_45V_over_voltage_tmp;
    error_mgu1_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu1_45V_over_voltage(error_mgu1_45V_over_voltage_tmp);
    break;
  }
  case 124 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu1_45V_under_voltage_tmp;
    error_mgu1_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu1_45V_under_voltage(error_mgu1_45V_under_voltage_tmp);
    break;
  }
  case 125 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu1_control_error_tmp;
    error_mgu1_control_error_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu1_control_error(error_mgu1_control_error_tmp);
    break;
  }
  case 126 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu1_magnet_over_temperature_starboard_tmp;
    error_mgu1_magnet_over_temperature_starboard_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu1_magnet_over_temperature_starboard(error_mgu1_magnet_over_temperature_starboard_tmp);
    break;
  }
  case 127 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu1_magnet_over_temperature_port_tmp;
    error_mgu1_magnet_over_temperature_port_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu1_magnet_over_temperature_port(error_mgu1_magnet_over_temperature_port_tmp);
    break;
  }
  case 128 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu1_mosfet_over_temperature_tmp;
    error_mgu1_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu1_mosfet_over_temperature(error_mgu1_mosfet_over_temperature_tmp);
    break;
  }
  case 129 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu1_mcu_over_temperature_tmp;
    error_mgu1_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu1_mcu_over_temperature(error_mgu1_mcu_over_temperature_tmp);
    break;
  }
  case 130 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mgu1_magnet_over_temperature_starboard_tmp;
    warn_mgu1_magnet_over_temperature_starboard_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mgu1_magnet_over_temperature_starboard(warn_mgu1_magnet_over_temperature_starboard_tmp);
    break;
  }
  case 131 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mgu1_magnet_over_temperature_port_tmp;
    warn_mgu1_magnet_over_temperature_port_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mgu1_magnet_over_temperature_port(warn_mgu1_magnet_over_temperature_port_tmp);
    break;
  }
  case 132 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mgu1_mosfet_over_temperature_tmp;
    warn_mgu1_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mgu1_mosfet_over_temperature(warn_mgu1_mosfet_over_temperature_tmp);
    break;
  }
  case 133 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mgu1_mcu_over_temperature_tmp;
    warn_mgu1_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mgu1_mcu_over_temperature(warn_mgu1_mcu_over_temperature_tmp);
    break;
  }
  case 134 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mgu_state mgu1_state_tmp;
    mgu1_state_tmp = (mgu_state)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mgu1_state(mgu1_state_tmp);
    break;
  }
  case 135 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mgu1_power_estimation_tmp;
    mgu1_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0);
    canzero_set_mgu1_power_estimation(mgu1_power_estimation_tmp);
    break;
  }
  case 136 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status mgu1_sdc_status_tmp;
    mgu1_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_mgu1_sdc_status(mgu1_sdc_status_tmp);
    break;
  }
  case 137 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu2_45V_over_voltage_tmp;
    error_mgu2_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu2_45V_over_voltage(error_mgu2_45V_over_voltage_tmp);
    break;
  }
  case 138 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu2_45V_under_voltage_tmp;
    error_mgu2_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu2_45V_under_voltage(error_mgu2_45V_under_voltage_tmp);
    break;
  }
  case 139 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu2_control_error_tmp;
    error_mgu2_control_error_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu2_control_error(error_mgu2_control_error_tmp);
    break;
  }
  case 140 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu2_magnet_over_temperature_starboard_tmp;
    error_mgu2_magnet_over_temperature_starboard_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu2_magnet_over_temperature_starboard(error_mgu2_magnet_over_temperature_starboard_tmp);
    break;
  }
  case 141 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu2_magnet_over_temperature_port_tmp;
    error_mgu2_magnet_over_temperature_port_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu2_magnet_over_temperature_port(error_mgu2_magnet_over_temperature_port_tmp);
    break;
  }
  case 142 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu2_mosfet_over_temperature_tmp;
    error_mgu2_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu2_mosfet_over_temperature(error_mgu2_mosfet_over_temperature_tmp);
    break;
  }
  case 143 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mgu2_mcu_over_temperature_tmp;
    error_mgu2_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mgu2_mcu_over_temperature(error_mgu2_mcu_over_temperature_tmp);
    break;
  }
  case 144 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mgu2_magnet_over_temperature_starboard_tmp;
    warn_mgu2_magnet_over_temperature_starboard_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mgu2_magnet_over_temperature_starboard(warn_mgu2_magnet_over_temperature_starboard_tmp);
    break;
  }
  case 145 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mgu2_magnet_over_temperature_port_tmp;
    warn_mgu2_magnet_over_temperature_port_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mgu2_magnet_over_temperature_port(warn_mgu2_magnet_over_temperature_port_tmp);
    break;
  }
  case 146 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mgu2_mosfet_over_temperature_tmp;
    warn_mgu2_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mgu2_mosfet_over_temperature(warn_mgu2_mosfet_over_temperature_tmp);
    break;
  }
  case 147 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mgu2_mcu_over_temperature_tmp;
    warn_mgu2_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mgu2_mcu_over_temperature(warn_mgu2_mcu_over_temperature_tmp);
    break;
  }
  case 148 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mgu_state mgu2_state_tmp;
    mgu2_state_tmp = (mgu_state)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mgu2_state(mgu2_state_tmp);
    break;
  }
  case 149 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mgu2_power_estimation_tmp;
    mgu2_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0);
    canzero_set_mgu2_power_estimation(mgu2_power_estimation_tmp);
    break;
  }
  case 150 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status mgu2_sdc_status_tmp;
    mgu2_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_mgu2_sdc_status(mgu2_sdc_status_tmp);
    break;
  }
  case 151 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu1_45V_over_voltage_tmp;
    error_mlu1_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu1_45V_over_voltage(error_mlu1_45V_over_voltage_tmp);
    break;
  }
  case 152 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu1_45V_under_voltage_tmp;
    error_mlu1_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu1_45V_under_voltage(error_mlu1_45V_under_voltage_tmp);
    break;
  }
  case 153 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu1_magnet_over_temperature_tmp;
    error_mlu1_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu1_magnet_over_temperature(error_mlu1_magnet_over_temperature_tmp);
    break;
  }
  case 154 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu1_mcu_over_temperature_tmp;
    error_mlu1_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu1_mcu_over_temperature(error_mlu1_mcu_over_temperature_tmp);
    break;
  }
  case 155 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu1_mosfet_over_temperature_tmp;
    error_mlu1_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu1_mosfet_over_temperature(error_mlu1_mosfet_over_temperature_tmp);
    break;
  }
  case 156 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu1_control_failure_tmp;
    error_mlu1_control_failure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu1_control_failure(error_mlu1_control_failure_tmp);
    break;
  }
  case 157 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu1_magnet_over_temperature_tmp;
    warn_mlu1_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu1_magnet_over_temperature(warn_mlu1_magnet_over_temperature_tmp);
    break;
  }
  case 158 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu1_mcu_over_temperature_tmp;
    warn_mlu1_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu1_mcu_over_temperature(warn_mlu1_mcu_over_temperature_tmp);
    break;
  }
  case 159 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu1_mosfet_over_temperature_tmp;
    warn_mlu1_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu1_mosfet_over_temperature(warn_mlu1_mosfet_over_temperature_tmp);
    break;
  }
  case 160 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mlu_state mlu1_state_tmp;
    mlu1_state_tmp = (mlu_state)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mlu1_state(mlu1_state_tmp);
    break;
  }
  case 161 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mlu1_power_estimation_tmp;
    mlu1_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0);
    canzero_set_mlu1_power_estimation(mlu1_power_estimation_tmp);
    break;
  }
  case 162 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status mlu1_sdc_status_tmp;
    mlu1_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_mlu1_sdc_status(mlu1_sdc_status_tmp);
    break;
  }
  case 163 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu2_45V_over_voltage_tmp;
    error_mlu2_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu2_45V_over_voltage(error_mlu2_45V_over_voltage_tmp);
    break;
  }
  case 164 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu2_45V_under_voltage_tmp;
    error_mlu2_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu2_45V_under_voltage(error_mlu2_45V_under_voltage_tmp);
    break;
  }
  case 165 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu2_magnet_over_temperature_tmp;
    error_mlu2_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu2_magnet_over_temperature(error_mlu2_magnet_over_temperature_tmp);
    break;
  }
  case 166 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu2_mcu_over_temperature_tmp;
    error_mlu2_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu2_mcu_over_temperature(error_mlu2_mcu_over_temperature_tmp);
    break;
  }
  case 167 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu2_mosfet_over_temperature_tmp;
    error_mlu2_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu2_mosfet_over_temperature(error_mlu2_mosfet_over_temperature_tmp);
    break;
  }
  case 168 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu2_control_failure_tmp;
    error_mlu2_control_failure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu2_control_failure(error_mlu2_control_failure_tmp);
    break;
  }
  case 169 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu2_magnet_over_temperature_tmp;
    warn_mlu2_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu2_magnet_over_temperature(warn_mlu2_magnet_over_temperature_tmp);
    break;
  }
  case 170 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu2_mcu_over_temperature_tmp;
    warn_mlu2_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu2_mcu_over_temperature(warn_mlu2_mcu_over_temperature_tmp);
    break;
  }
  case 171 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu2_mosfet_over_temperature_tmp;
    warn_mlu2_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu2_mosfet_over_temperature(warn_mlu2_mosfet_over_temperature_tmp);
    break;
  }
  case 172 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mlu_state mlu2_state_tmp;
    mlu2_state_tmp = (mlu_state)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mlu2_state(mlu2_state_tmp);
    break;
  }
  case 173 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mlu2_power_estimation_tmp;
    mlu2_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0);
    canzero_set_mlu2_power_estimation(mlu2_power_estimation_tmp);
    break;
  }
  case 174 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status mlu2_sdc_status_tmp;
    mlu2_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_mlu2_sdc_status(mlu2_sdc_status_tmp);
    break;
  }
  case 175 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu3_45V_over_voltage_tmp;
    error_mlu3_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu3_45V_over_voltage(error_mlu3_45V_over_voltage_tmp);
    break;
  }
  case 176 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu3_45V_under_voltage_tmp;
    error_mlu3_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu3_45V_under_voltage(error_mlu3_45V_under_voltage_tmp);
    break;
  }
  case 177 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu3_magnet_over_temperature_tmp;
    error_mlu3_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu3_magnet_over_temperature(error_mlu3_magnet_over_temperature_tmp);
    break;
  }
  case 178 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu3_mcu_over_temperature_tmp;
    error_mlu3_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu3_mcu_over_temperature(error_mlu3_mcu_over_temperature_tmp);
    break;
  }
  case 179 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu3_mosfet_over_temperature_tmp;
    error_mlu3_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu3_mosfet_over_temperature(error_mlu3_mosfet_over_temperature_tmp);
    break;
  }
  case 180 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu3_control_failure_tmp;
    error_mlu3_control_failure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu3_control_failure(error_mlu3_control_failure_tmp);
    break;
  }
  case 181 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu3_magnet_over_temperature_tmp;
    warn_mlu3_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu3_magnet_over_temperature(warn_mlu3_magnet_over_temperature_tmp);
    break;
  }
  case 182 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu3_mcu_over_temperature_tmp;
    warn_mlu3_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu3_mcu_over_temperature(warn_mlu3_mcu_over_temperature_tmp);
    break;
  }
  case 183 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu3_mosfet_over_temperature_tmp;
    warn_mlu3_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu3_mosfet_over_temperature(warn_mlu3_mosfet_over_temperature_tmp);
    break;
  }
  case 184 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mlu_state mlu3_state_tmp;
    mlu3_state_tmp = (mlu_state)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mlu3_state(mlu3_state_tmp);
    break;
  }
  case 185 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mlu3_power_estimation_tmp;
    mlu3_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0);
    canzero_set_mlu3_power_estimation(mlu3_power_estimation_tmp);
    break;
  }
  case 186 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status mlu3_sdc_status_tmp;
    mlu3_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_mlu3_sdc_status(mlu3_sdc_status_tmp);
    break;
  }
  case 187 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu4_45V_over_voltage_tmp;
    error_mlu4_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu4_45V_over_voltage(error_mlu4_45V_over_voltage_tmp);
    break;
  }
  case 188 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu4_45V_under_voltage_tmp;
    error_mlu4_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu4_45V_under_voltage(error_mlu4_45V_under_voltage_tmp);
    break;
  }
  case 189 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu4_magnet_over_temperature_tmp;
    error_mlu4_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu4_magnet_over_temperature(error_mlu4_magnet_over_temperature_tmp);
    break;
  }
  case 190 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu4_mcu_over_temperature_tmp;
    error_mlu4_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu4_mcu_over_temperature(error_mlu4_mcu_over_temperature_tmp);
    break;
  }
  case 191 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu4_mosfet_over_temperature_tmp;
    error_mlu4_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu4_mosfet_over_temperature(error_mlu4_mosfet_over_temperature_tmp);
    break;
  }
  case 192 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu4_control_failure_tmp;
    error_mlu4_control_failure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu4_control_failure(error_mlu4_control_failure_tmp);
    break;
  }
  case 193 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu4_magnet_over_temperature_tmp;
    warn_mlu4_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu4_magnet_over_temperature(warn_mlu4_magnet_over_temperature_tmp);
    break;
  }
  case 194 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu4_mcu_over_temperature_tmp;
    warn_mlu4_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu4_mcu_over_temperature(warn_mlu4_mcu_over_temperature_tmp);
    break;
  }
  case 195 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu4_mosfet_over_temperature_tmp;
    warn_mlu4_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu4_mosfet_over_temperature(warn_mlu4_mosfet_over_temperature_tmp);
    break;
  }
  case 196 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mlu_state mlu4_state_tmp;
    mlu4_state_tmp = (mlu_state)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mlu4_state(mlu4_state_tmp);
    break;
  }
  case 197 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mlu4_power_estimation_tmp;
    mlu4_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0);
    canzero_set_mlu4_power_estimation(mlu4_power_estimation_tmp);
    break;
  }
  case 198 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status mlu4_sdc_status_tmp;
    mlu4_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_mlu4_sdc_status(mlu4_sdc_status_tmp);
    break;
  }
  case 199 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu5_45V_over_voltage_tmp;
    error_mlu5_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu5_45V_over_voltage(error_mlu5_45V_over_voltage_tmp);
    break;
  }
  case 200 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu5_45V_under_voltage_tmp;
    error_mlu5_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu5_45V_under_voltage(error_mlu5_45V_under_voltage_tmp);
    break;
  }
  case 201 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu5_magnet_over_temperature_tmp;
    error_mlu5_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu5_magnet_over_temperature(error_mlu5_magnet_over_temperature_tmp);
    break;
  }
  case 202 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu5_mcu_over_temperature_tmp;
    error_mlu5_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu5_mcu_over_temperature(error_mlu5_mcu_over_temperature_tmp);
    break;
  }
  case 203 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu5_mosfet_over_temperature_tmp;
    error_mlu5_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu5_mosfet_over_temperature(error_mlu5_mosfet_over_temperature_tmp);
    break;
  }
  case 204 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu5_control_failure_tmp;
    error_mlu5_control_failure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu5_control_failure(error_mlu5_control_failure_tmp);
    break;
  }
  case 205 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu5_magnet_over_temperature_tmp;
    warn_mlu5_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu5_magnet_over_temperature(warn_mlu5_magnet_over_temperature_tmp);
    break;
  }
  case 206 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu5_mcu_over_temperature_tmp;
    warn_mlu5_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu5_mcu_over_temperature(warn_mlu5_mcu_over_temperature_tmp);
    break;
  }
  case 207 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu5_mosfet_over_temperature_tmp;
    warn_mlu5_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu5_mosfet_over_temperature(warn_mlu5_mosfet_over_temperature_tmp);
    break;
  }
  case 208 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mlu_state mlu5_state_tmp;
    mlu5_state_tmp = (mlu_state)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mlu5_state(mlu5_state_tmp);
    break;
  }
  case 209 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mlu5_power_estimation_tmp;
    mlu5_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0);
    canzero_set_mlu5_power_estimation(mlu5_power_estimation_tmp);
    break;
  }
  case 210 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status mlu5_sdc_status_tmp;
    mlu5_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_mlu5_sdc_status(mlu5_sdc_status_tmp);
    break;
  }
  case 211 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu6_45V_over_voltage_tmp;
    error_mlu6_45V_over_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu6_45V_over_voltage(error_mlu6_45V_over_voltage_tmp);
    break;
  }
  case 212 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu6_45V_under_voltage_tmp;
    error_mlu6_45V_under_voltage_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu6_45V_under_voltage(error_mlu6_45V_under_voltage_tmp);
    break;
  }
  case 213 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu6_magnet_over_temperature_tmp;
    error_mlu6_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu6_magnet_over_temperature(error_mlu6_magnet_over_temperature_tmp);
    break;
  }
  case 214 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu6_mcu_over_temperature_tmp;
    error_mlu6_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu6_mcu_over_temperature(error_mlu6_mcu_over_temperature_tmp);
    break;
  }
  case 215 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu6_mosfet_over_temperature_tmp;
    error_mlu6_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu6_mosfet_over_temperature(error_mlu6_mosfet_over_temperature_tmp);
    break;
  }
  case 216 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag error_mlu6_control_failure_tmp;
    error_mlu6_control_failure_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_error_mlu6_control_failure(error_mlu6_control_failure_tmp);
    break;
  }
  case 217 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu6_magnet_over_temperature_tmp;
    warn_mlu6_magnet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu6_magnet_over_temperature(warn_mlu6_magnet_over_temperature_tmp);
    break;
  }
  case 218 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu6_mcu_over_temperature_tmp;
    warn_mlu6_mcu_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu6_mcu_over_temperature(warn_mlu6_mcu_over_temperature_tmp);
    break;
  }
  case 219 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    error_flag warn_mlu6_mosfet_over_temperature_tmp;
    warn_mlu6_mosfet_over_temperature_tmp = (error_flag)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_warn_mlu6_mosfet_over_temperature(warn_mlu6_mosfet_over_temperature_tmp);
    break;
  }
  case 220 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    mlu_state mlu6_state_tmp;
    mlu6_state_tmp = (mlu_state)((msg.data & (0xFFFFFFFF >> (32 - 3))));
    canzero_set_mlu6_state(mlu6_state_tmp);
    break;
  }
  case 221 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    float mlu6_power_estimation_tmp;
    mlu6_power_estimation_tmp = (float)((msg.data & (0xFFFFFFFF >> (32 - 16))) * 0.015259021896696421 + 0);
    canzero_set_mlu6_power_estimation(mlu6_power_estimation_tmp);
    break;
  }
  case 222 : {
    if (msg.header.sof != 1 || msg.header.toggle != 0 || msg.header.eof != 1) {
      return;
    }
    sdc_status mlu6_sdc_status_tmp;
    mlu6_sdc_status_tmp = (sdc_status)((msg.data & (0xFFFFFFFF >> (32 - 1))));
    canzero_set_mlu6_sdc_status(mlu6_sdc_status_tmp);
    break;
  }
  default:
    return;
  }
  resp.header.od_index = msg.header.od_index;
  resp.header.client_id = msg.header.client_id;
  resp.header.server_id = msg.header.server_id;
  resp.header.erno = set_resp_erno_Success;
  canzero_frame resp_frame;
  canzero_serialize_canzero_message_set_resp(&resp, &resp_frame);
  canzero_can1_send(&resp_frame);

}
static void canzero_handle_input_board_stream_state(canzero_frame* frame) {
  canzero_message_input_board_stream_state msg;
  canzero_deserialize_canzero_message_input_board_stream_state(frame, &msg);
  canzero_set_input_board_state(msg.state);
}
static void canzero_handle_input_board_stream_state_estimation(canzero_frame* frame) {
  canzero_message_input_board_stream_state_estimation msg;
  canzero_deserialize_canzero_message_input_board_stream_state_estimation(frame, &msg);
  canzero_set_position(msg.position_estimation);
  canzero_set_velocity(msg.velocity_estimation);
  canzero_set_acceleration(msg.acceleration_estimation);
}
static void canzero_handle_input_board_stream_errors(canzero_frame* frame) {
  canzero_message_input_board_stream_errors msg;
  canzero_deserialize_canzero_message_input_board_stream_errors(frame, &msg);
  canzero_set_error_input_board_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_error_invalid_position_estimation(msg.error_invalid_position_estimation);
  canzero_set_error_invalid_position(msg.error_invalid_position);
  canzero_set_error_invalid_velocity_profile(msg.error_invalid_velocity_profile);
  canzero_set_error_invalid_acceleration_profile(msg.error_invalid_acceleration_profile);
  canzero_set_error_battery_over_voltage(msg.error_battery_over_temperature);
  canzero_set_error_cooling_cycle_over_pressure(msg.error_cooling_cycle_over_pressure);
  canzero_set_error_cooling_cycle_low_pressure(msg.error_cooling_cycle_low_pressure);
  canzero_set_error_ebox_over_temperature(msg.error_ebox_over_temperature);
  canzero_set_error_cooling_cycle_over_temperature(msg.error_cooling_cycle_over_temperature);
  canzero_set_warn_invalid_position_estimation(msg.warn_invalid_position_estimation);
  canzero_set_warn_invalid_position(msg.warn_invalid_position);
  canzero_set_warn_invalid_velocity_profile(msg.warn_invalid_velocity_profile);
  canzero_set_warn_invalid_acceleration_profile(msg.warn_invalid_acceleration_profile);
  canzero_set_warn_battery_over_temperature(msg.warn_battery_over_temperature);
  canzero_set_warn_cooling_cycle_over_pressure(msg.warn_cooling_cycle_over_pressure);
  canzero_set_warn_cooling_cycle_low_pressure(msg.warn_cooling_cycle_low_pressure);
  canzero_set_warn_input_board_mcu_over_temperature(msg.warn_mcu_over_temperature);
  canzero_set_warn_ebox_over_temperature(msg.warn_ebox_over_temperature);
  canzero_set_warn_cooling_cycle_over_temperature(msg.warn_cooling_cycle_over_temperature);
}
static void canzero_handle_input_board_stream_sdc_status(canzero_frame* frame) {
  canzero_message_input_board_stream_sdc_status msg;
  canzero_deserialize_canzero_message_input_board_stream_sdc_status(frame, &msg);
  canzero_set_input_board_sdc_status(msg.sdc_status);
}
static void canzero_handle_pdu12_stream_state(canzero_frame* frame) {
  canzero_message_pdu12_stream_state msg;
  canzero_deserialize_canzero_message_pdu12_stream_state(frame, &msg);
  canzero_set_pdu12_state(msg.state);
}
static void canzero_handle_pdu12_stream_channel_status(canzero_frame* frame) {
  canzero_message_pdu12_stream_channel_status msg;
  canzero_deserialize_canzero_message_pdu12_stream_channel_status(frame, &msg);
  canzero_set_pdu12_lp_channel1_status(msg.lp_channel1_status);
  canzero_set_pdu12_lp_channel2_status(msg.lp_channel2_status);
  canzero_set_pdu12_lp_channel3_status(msg.lp_channel3_status);
  canzero_set_pdu12_lp_channel4_status(msg.lp_channel4_status);
  canzero_set_pdu12_lp_channel5_status(msg.lp_channel5_status);
  canzero_set_pdu12_lp_channel6_status(msg.lp_channel6_status);
  canzero_set_pdu12_lp_channel7_status(msg.lp_channel7_status);
  canzero_set_pdu12_lp_channel8_status(msg.lp_channel8_status);
  canzero_set_pdu12_lp_chanel9_status(msg.lp_channel9_status);
  canzero_set_pdu12_lp_channel10_status(msg.lp_channel10_status);
  canzero_set_pdu12_lp_channel11_status(msg.lp_channel11_status);
  canzero_set_pdu12_lp_channel12_status(msg.lp_channel12_status);
  canzero_set_pdu12_hp_channel1_status(msg.hp_channel1_status);
  canzero_set_pdu12_hp_channel2_status(msg.hp_channel2_status);
  canzero_set_pdu12_hp_channel3_status(msg.hp_channel3_status);
  canzero_set_pdu12_hp_channel4_status(msg.hp_channel4_status);
  canzero_set_pdu12_sdc_status(msg.sdc_status);
  canzero_set_error_pdu12_mcu_over_temperature(msg.mcu_over_temperature);
}
static void canzero_handle_pdu12_stream_power_estimation(canzero_frame* frame) {
  canzero_message_pdu12_stream_power_estimation msg;
  canzero_deserialize_canzero_message_pdu12_stream_power_estimation(frame, &msg);
}
static void canzero_handle_pdu24_stream_state(canzero_frame* frame) {
  canzero_message_pdu24_stream_state msg;
  canzero_deserialize_canzero_message_pdu24_stream_state(frame, &msg);
  canzero_set_pdu24_state(msg.state);
}
static void canzero_handle_pdu24_stream_channel_status(canzero_frame* frame) {
  canzero_message_pdu24_stream_channel_status msg;
  canzero_deserialize_canzero_message_pdu24_stream_channel_status(frame, &msg);
  canzero_set_pdu24_lp_channel1_status(msg.lp_channel1_status);
  canzero_set_pdu24_lp_channel2_status(msg.lp_channel2_status);
  canzero_set_pdu24_lp_channel3_status(msg.lp_channel3_status);
  canzero_set_pdu24_lp_channel4_status(msg.lp_channel4_status);
  canzero_set_pdu24_lp_channel5_status(msg.lp_channel5_status);
  canzero_set_pdu24_lp_channel6_status(msg.lp_channel6_status);
  canzero_set_pdu24_lp_channel7_status(msg.lp_channel7_status);
  canzero_set_pdu24_lp_channel8_status(msg.lp_channel8_status);
  canzero_set_pdu24_lp_chanel9_status(msg.lp_channel9_status);
  canzero_set_pdu24_lp_channel10_status(msg.lp_channel10_status);
  canzero_set_pdu24_lp_channel11_status(msg.lp_channel11_status);
  canzero_set_pdu24_lp_channel12_status(msg.lp_channel12_status);
  canzero_set_pdu24_hp_channel1_status(msg.hp_channel1_status);
  canzero_set_pdu24_hp_channel2_status(msg.hp_channel2_status);
  canzero_set_pdu24_hp_channel3_status(msg.hp_channel3_status);
  canzero_set_pdu24_hp_channel4_status(msg.hp_channel4_status);
  canzero_set_pdu24_sdc_status(msg.sdc_status);
  canzero_set_error_pdu24_mcu_over_temperature(msg.mcu_over_temperature);
}
static void canzero_handle_pdu24_stream_power_estimation(canzero_frame* frame) {
  canzero_message_pdu24_stream_power_estimation msg;
  canzero_deserialize_canzero_message_pdu24_stream_power_estimation(frame, &msg);
}
static void canzero_handle_motor_driver_stream_errors(canzero_frame* frame) {
  canzero_message_motor_driver_stream_errors msg;
  canzero_deserialize_canzero_message_motor_driver_stream_errors(frame, &msg);
  canzero_set_error_motor_driver_45V_over_voltage(msg.error_45V_over_voltage);
  canzero_set_error_motor_driver_45V_under_voltage(msg.error_45V_under_voltage);
  canzero_set_error_dslim_over_temperature(msg.error_dslim_over_temperature);
  canzero_set_error_motor_driver_mosfet_over_temperature(msg.error_mosfet_over_temperature);
  canzero_set_error_motor_control_failure(msg.error_control_failure);
  canzero_set_error_motor_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_warn_motor_dslim_over_temperature(msg.warn_dslim_over_temperature);
  canzero_set_warn_motor_mosfet_over_temperature(msg.warn_mosfet_over_temperature);
  canzero_set_warn_motor_mcu_over_temperature(msg.warn_mcu_over_temperature);
}
static void canzero_handle_motor_driver_stream_power_estimation(canzero_frame* frame) {
  canzero_message_motor_driver_stream_power_estimation msg;
  canzero_deserialize_canzero_message_motor_driver_stream_power_estimation(frame, &msg);
}
static void canzero_handle_motor_driver_stream_state(canzero_frame* frame) {
  canzero_message_motor_driver_stream_state msg;
  canzero_deserialize_canzero_message_motor_driver_stream_state(frame, &msg);
  canzero_set_motor_state(msg.state);
}
static void canzero_handle_motor_driver_stream_sdc_status(canzero_frame* frame) {
  canzero_message_motor_driver_stream_sdc_status msg;
  canzero_deserialize_canzero_message_motor_driver_stream_sdc_status(frame, &msg);
  canzero_set_motor_sdc_status(msg.sdc_status);
}
static void canzero_handle_mgu1_stream_errors(canzero_frame* frame) {
  canzero_message_mgu1_stream_errors msg;
  canzero_deserialize_canzero_message_mgu1_stream_errors(frame, &msg);
  canzero_set_error_mgu1_45V_over_voltage(msg.error_45V_over_voltage);
  canzero_set_error_mgu1_45V_under_voltage(msg.error_45V_under_voltage);
  canzero_set_error_mgu1_control_error(msg.error_control_error);
  canzero_set_error_mgu1_magnet_over_temperature_starboard(msg.error_magnet_over_temperature_starboard);
  canzero_set_error_mgu1_magnet_over_temperature_port(msg.error_magnet_over_temperature_port);
  canzero_set_error_mgu1_mosfet_over_temperature(msg.error_mosfet_over_temperature);
  canzero_set_error_mgu1_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_warn_mgu1_magnet_over_temperature_starboard(msg.warn_magnet_over_temperature_starboard);
  canzero_set_warn_mgu1_magnet_over_temperature_port(msg.warn_magnet_over_temperature_port);
  canzero_set_warn_mgu1_mosfet_over_temperature(msg.warn_mosfet_over_temperature);
  canzero_set_warn_mgu1_mcu_over_temperature(msg.warn_mcu_over_temperature);
}
static void canzero_handle_mgu1_stream_state(canzero_frame* frame) {
  canzero_message_mgu1_stream_state msg;
  canzero_deserialize_canzero_message_mgu1_stream_state(frame, &msg);
  canzero_set_mgu1_state(msg.state);
}
static void canzero_handle_mgu1_stream_power_estimation(canzero_frame* frame) {
  canzero_message_mgu1_stream_power_estimation msg;
  canzero_deserialize_canzero_message_mgu1_stream_power_estimation(frame, &msg);
}
static void canzero_handle_mgu1_stream_sdc_status(canzero_frame* frame) {
  canzero_message_mgu1_stream_sdc_status msg;
  canzero_deserialize_canzero_message_mgu1_stream_sdc_status(frame, &msg);
  canzero_set_mgu1_sdc_status(msg.sdc_status);
}
static void canzero_handle_mgu2_stream_errors(canzero_frame* frame) {
  canzero_message_mgu2_stream_errors msg;
  canzero_deserialize_canzero_message_mgu2_stream_errors(frame, &msg);
  canzero_set_error_mgu2_45V_over_voltage(msg.error_45V_over_voltage);
  canzero_set_error_mgu2_45V_under_voltage(msg.error_45V_under_voltage);
  canzero_set_error_mgu2_control_error(msg.error_control_error);
  canzero_set_error_mgu2_magnet_over_temperature_starboard(msg.error_magnet_over_temperature_starboard);
  canzero_set_error_mgu2_magnet_over_temperature_port(msg.error_magnet_over_temperature_port);
  canzero_set_error_mgu2_mosfet_over_temperature(msg.error_mosfet_over_temperature);
  canzero_set_error_mgu2_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_warn_mgu2_magnet_over_temperature_starboard(msg.warn_magnet_over_temperature_starboard);
  canzero_set_warn_mgu2_magnet_over_temperature_port(msg.warn_magnet_over_temperature_port);
  canzero_set_warn_mgu2_mosfet_over_temperature(msg.warn_mosfet_over_temperature);
  canzero_set_warn_mgu2_mcu_over_temperature(msg.warn_mcu_over_temperature);
}
static void canzero_handle_mgu2_stream_state(canzero_frame* frame) {
  canzero_message_mgu2_stream_state msg;
  canzero_deserialize_canzero_message_mgu2_stream_state(frame, &msg);
  canzero_set_mgu2_state(msg.state);
}
static void canzero_handle_mgu2_stream_power_estimation(canzero_frame* frame) {
  canzero_message_mgu2_stream_power_estimation msg;
  canzero_deserialize_canzero_message_mgu2_stream_power_estimation(frame, &msg);
}
static void canzero_handle_mgu2_stream_sdc_status(canzero_frame* frame) {
  canzero_message_mgu2_stream_sdc_status msg;
  canzero_deserialize_canzero_message_mgu2_stream_sdc_status(frame, &msg);
  canzero_set_mgu2_sdc_status(msg.sdc_status);
}
static void canzero_handle_mlu1_stream_errors(canzero_frame* frame) {
  canzero_message_mlu1_stream_errors msg;
  canzero_deserialize_canzero_message_mlu1_stream_errors(frame, &msg);
  canzero_set_error_mlu1_45V_over_voltage(msg.error_45V_over_voltage);
  canzero_set_error_mlu1_45V_under_voltage(msg.error_45V_under_voltage);
  canzero_set_error_mlu1_magnet_over_temperature(msg.error_magnet_over_temperature);
  canzero_set_error_mlu1_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_error_mlu1_mosfet_over_temperature(msg.error_mosfet_over_temperature);
  canzero_set_error_mlu1_control_failure(msg.error_control_failure);
  canzero_set_warn_mlu1_magnet_over_temperature(msg.warn_magnet_over_temperature);
  canzero_set_warn_mlu1_mcu_over_temperature(msg.warn_mcu_over_temperature);
  canzero_set_warn_mlu1_mosfet_over_temperature(msg.warn_mosfet_over_temperature);
}
static void canzero_handle_mlu1_stream_state(canzero_frame* frame) {
  canzero_message_mlu1_stream_state msg;
  canzero_deserialize_canzero_message_mlu1_stream_state(frame, &msg);
  canzero_set_mlu1_state(msg.state);
}
static void canzero_handle_mlu1_stream_power_estimation(canzero_frame* frame) {
  canzero_message_mlu1_stream_power_estimation msg;
  canzero_deserialize_canzero_message_mlu1_stream_power_estimation(frame, &msg);
}
static void canzero_handle_mlu1_stream_sdc_status(canzero_frame* frame) {
  canzero_message_mlu1_stream_sdc_status msg;
  canzero_deserialize_canzero_message_mlu1_stream_sdc_status(frame, &msg);
  canzero_set_mlu1_sdc_status(msg.sdc_status);
}
static void canzero_handle_mlu2_stream_errors(canzero_frame* frame) {
  canzero_message_mlu2_stream_errors msg;
  canzero_deserialize_canzero_message_mlu2_stream_errors(frame, &msg);
  canzero_set_error_mlu2_45V_over_voltage(msg.error_45V_over_voltage);
  canzero_set_error_mlu2_45V_under_voltage(msg.error_45V_under_voltage);
  canzero_set_error_mlu2_magnet_over_temperature(msg.error_magnet_over_temperature);
  canzero_set_error_mlu2_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_error_mlu2_mosfet_over_temperature(msg.error_mosfet_over_temperature);
  canzero_set_error_mlu2_control_failure(msg.error_control_failure);
  canzero_set_warn_mlu2_magnet_over_temperature(msg.warn_magnet_over_temperature);
  canzero_set_warn_mlu2_mcu_over_temperature(msg.warn_mcu_over_temperature);
  canzero_set_warn_mlu2_mosfet_over_temperature(msg.warn_mosfet_over_temperature);
}
static void canzero_handle_mlu2_stream_state(canzero_frame* frame) {
  canzero_message_mlu2_stream_state msg;
  canzero_deserialize_canzero_message_mlu2_stream_state(frame, &msg);
  canzero_set_mlu2_state(msg.state);
}
static void canzero_handle_mlu2_stream_power_estimation(canzero_frame* frame) {
  canzero_message_mlu2_stream_power_estimation msg;
  canzero_deserialize_canzero_message_mlu2_stream_power_estimation(frame, &msg);
}
static void canzero_handle_mlu2_stream_sdc_status(canzero_frame* frame) {
  canzero_message_mlu2_stream_sdc_status msg;
  canzero_deserialize_canzero_message_mlu2_stream_sdc_status(frame, &msg);
  canzero_set_mlu2_sdc_status(msg.sdc_status);
}
static void canzero_handle_mlu3_stream_errors(canzero_frame* frame) {
  canzero_message_mlu3_stream_errors msg;
  canzero_deserialize_canzero_message_mlu3_stream_errors(frame, &msg);
  canzero_set_error_mlu3_45V_over_voltage(msg.error_45V_over_voltage);
  canzero_set_error_mlu3_45V_under_voltage(msg.error_45V_under_voltage);
  canzero_set_error_mlu3_magnet_over_temperature(msg.error_magnet_over_temperature);
  canzero_set_error_mlu3_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_error_mlu3_mosfet_over_temperature(msg.error_mosfet_over_temperature);
  canzero_set_error_mlu3_control_failure(msg.error_control_failure);
  canzero_set_warn_mlu3_magnet_over_temperature(msg.warn_magnet_over_temperature);
  canzero_set_warn_mlu3_mcu_over_temperature(msg.warn_mcu_over_temperature);
  canzero_set_warn_mlu3_mosfet_over_temperature(msg.warn_mosfet_over_temperature);
}
static void canzero_handle_mlu3_stream_state(canzero_frame* frame) {
  canzero_message_mlu3_stream_state msg;
  canzero_deserialize_canzero_message_mlu3_stream_state(frame, &msg);
  canzero_set_mlu3_state(msg.state);
}
static void canzero_handle_mlu3_stream_power_estimation(canzero_frame* frame) {
  canzero_message_mlu3_stream_power_estimation msg;
  canzero_deserialize_canzero_message_mlu3_stream_power_estimation(frame, &msg);
}
static void canzero_handle_mlu3_stream_sdc_status(canzero_frame* frame) {
  canzero_message_mlu3_stream_sdc_status msg;
  canzero_deserialize_canzero_message_mlu3_stream_sdc_status(frame, &msg);
  canzero_set_mlu3_sdc_status(msg.sdc_status);
}
static void canzero_handle_mlu4_stream_errors(canzero_frame* frame) {
  canzero_message_mlu4_stream_errors msg;
  canzero_deserialize_canzero_message_mlu4_stream_errors(frame, &msg);
  canzero_set_error_mlu4_45V_over_voltage(msg.error_45V_over_voltage);
  canzero_set_error_mlu4_45V_under_voltage(msg.error_45V_under_voltage);
  canzero_set_error_mlu4_magnet_over_temperature(msg.error_magnet_over_temperature);
  canzero_set_error_mlu4_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_error_mlu4_mosfet_over_temperature(msg.error_mosfet_over_temperature);
  canzero_set_error_mlu4_control_failure(msg.error_control_failure);
  canzero_set_warn_mlu4_magnet_over_temperature(msg.warn_magnet_over_temperature);
  canzero_set_warn_mlu4_mcu_over_temperature(msg.warn_mcu_over_temperature);
  canzero_set_warn_mlu4_mosfet_over_temperature(msg.warn_mosfet_over_temperature);
}
static void canzero_handle_mlu4_stream_state(canzero_frame* frame) {
  canzero_message_mlu4_stream_state msg;
  canzero_deserialize_canzero_message_mlu4_stream_state(frame, &msg);
  canzero_set_mlu4_state(msg.state);
}
static void canzero_handle_mlu4_stream_power_estimation(canzero_frame* frame) {
  canzero_message_mlu4_stream_power_estimation msg;
  canzero_deserialize_canzero_message_mlu4_stream_power_estimation(frame, &msg);
}
static void canzero_handle_mlu4_stream_sdc_status(canzero_frame* frame) {
  canzero_message_mlu4_stream_sdc_status msg;
  canzero_deserialize_canzero_message_mlu4_stream_sdc_status(frame, &msg);
  canzero_set_mlu4_sdc_status(msg.sdc_status);
}
static void canzero_handle_mlu5_stream_errors(canzero_frame* frame) {
  canzero_message_mlu5_stream_errors msg;
  canzero_deserialize_canzero_message_mlu5_stream_errors(frame, &msg);
  canzero_set_error_mlu5_45V_over_voltage(msg.error_45V_over_voltage);
  canzero_set_error_mlu5_45V_under_voltage(msg.error_45V_under_voltage);
  canzero_set_error_mlu5_magnet_over_temperature(msg.error_magnet_over_temperature);
  canzero_set_error_mlu5_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_error_mlu5_mosfet_over_temperature(msg.error_mosfet_over_temperature);
  canzero_set_error_mlu5_control_failure(msg.error_control_failure);
  canzero_set_warn_mlu5_magnet_over_temperature(msg.warn_magnet_over_temperature);
  canzero_set_warn_mlu5_mcu_over_temperature(msg.warn_mcu_over_temperature);
  canzero_set_warn_mlu5_mosfet_over_temperature(msg.warn_mosfet_over_temperature);
}
static void canzero_handle_mlu5_stream_state(canzero_frame* frame) {
  canzero_message_mlu5_stream_state msg;
  canzero_deserialize_canzero_message_mlu5_stream_state(frame, &msg);
  canzero_set_mlu5_state(msg.state);
}
static void canzero_handle_mlu5_stream_power_estimation(canzero_frame* frame) {
  canzero_message_mlu5_stream_power_estimation msg;
  canzero_deserialize_canzero_message_mlu5_stream_power_estimation(frame, &msg);
}
static void canzero_handle_mlu5_stream_sdc_status(canzero_frame* frame) {
  canzero_message_mlu5_stream_sdc_status msg;
  canzero_deserialize_canzero_message_mlu5_stream_sdc_status(frame, &msg);
  canzero_set_mlu5_sdc_status(msg.sdc_status);
}
static void canzero_handle_mlu6_stream_errors(canzero_frame* frame) {
  canzero_message_mlu6_stream_errors msg;
  canzero_deserialize_canzero_message_mlu6_stream_errors(frame, &msg);
  canzero_set_error_mlu6_45V_over_voltage(msg.error_45V_over_voltage);
  canzero_set_error_mlu6_45V_under_voltage(msg.error_45V_under_voltage);
  canzero_set_error_mlu6_magnet_over_temperature(msg.error_magnet_over_temperature);
  canzero_set_error_mlu6_mcu_over_temperature(msg.error_mcu_over_temperature);
  canzero_set_error_mlu6_mosfet_over_temperature(msg.error_mosfet_over_temperature);
  canzero_set_error_mlu6_control_failure(msg.error_control_failure);
  canzero_set_warn_mlu6_magnet_over_temperature(msg.warn_magnet_over_temperature);
  canzero_set_warn_mlu6_mcu_over_temperature(msg.warn_mcu_over_temperature);
  canzero_set_warn_mlu6_mosfet_over_temperature(msg.warn_mosfet_over_temperature);
}
static void canzero_handle_mlu6_stream_state(canzero_frame* frame) {
  canzero_message_mlu6_stream_state msg;
  canzero_deserialize_canzero_message_mlu6_stream_state(frame, &msg);
  canzero_set_mlu6_state(msg.state);
}
static void canzero_handle_mlu6_stream_power_estimation(canzero_frame* frame) {
  canzero_message_mlu6_stream_power_estimation msg;
  canzero_deserialize_canzero_message_mlu6_stream_power_estimation(frame, &msg);
}
static void canzero_handle_mlu6_stream_sdc_status(canzero_frame* frame) {
  canzero_message_mlu6_stream_sdc_status msg;
  canzero_deserialize_canzero_message_mlu6_stream_sdc_status(frame, &msg);
  canzero_set_mlu6_sdc_status(msg.sdc_status);
}
__attribute__((weak)) void canzero_handle_heartbeat(canzero_frame* frame) {
  canzero_message_heartbeat msg;
  canzero_deserialize_canzero_message_heartbeat(frame, &msg);
}
void canzero_can0_poll() {
  canzero_frame frame;
  while (canzero_can0_recv(&frame)) {
    switch (frame.id) {
      case 0x13B:
        canzero_handle_get_req(&frame);
        break;
      case 0x15B:
        canzero_handle_set_req(&frame);
        break;
      case 0xBA:
        canzero_handle_input_board_stream_state_estimation(&frame);
        break;
      case 0x5A:
        canzero_handle_input_board_stream_errors(&frame);
        break;
      case 0x54:
        canzero_handle_pdu12_stream_state(&frame);
        break;
      case 0xF5:
        canzero_handle_pdu12_stream_channel_status(&frame);
        break;
      case 0x94:
        canzero_handle_pdu24_stream_power_estimation(&frame);
        break;
      case 0x95:
        canzero_handle_motor_driver_stream_power_estimation(&frame);
        break;
      case 0xB5:
        canzero_handle_motor_driver_stream_sdc_status(&frame);
        break;
      case 0xFA:
        canzero_handle_mgu1_stream_power_estimation(&frame);
        break;
      case 0x11A:
        canzero_handle_mgu1_stream_sdc_status(&frame);
        break;
      case 0x79:
        canzero_handle_mgu2_stream_errors(&frame);
        break;
      case 0xD9:
        canzero_handle_mgu2_stream_state(&frame);
        break;
      case 0xF9:
        canzero_handle_mlu1_stream_errors(&frame);
        break;
      case 0x78:
        canzero_handle_mlu1_stream_state(&frame);
        break;
      case 0x119:
        canzero_handle_mlu1_stream_power_estimation(&frame);
        break;
      case 0x58:
        canzero_handle_mlu1_stream_sdc_status(&frame);
        break;
      case 0x98:
        canzero_handle_mlu2_stream_errors(&frame);
        break;
      case 0x97:
        canzero_handle_mlu3_stream_state(&frame);
        break;
      case 0x57:
        canzero_handle_mlu3_stream_power_estimation(&frame);
        break;
      case 0x77:
        canzero_handle_mlu3_stream_sdc_status(&frame);
        break;
      case 0xB7:
        canzero_handle_mlu4_stream_errors(&frame);
        break;
      case 0xB6:
        canzero_handle_mlu5_stream_state(&frame);
        break;
      case 0x76:
        canzero_handle_mlu5_stream_power_estimation(&frame);
        break;
      case 0x96:
        canzero_handle_mlu5_stream_sdc_status(&frame);
        break;
      case 0xD6:
        canzero_handle_mlu6_stream_errors(&frame);
        break;
      case 0x17B:
        canzero_handle_heartbeat(&frame);
        break;
    }
  }
}
void canzero_can1_poll() {
  canzero_frame frame;
  while (canzero_can1_recv(&frame)) {
    switch (frame.id) {
      case 0x9A:
        canzero_handle_input_board_stream_state(&frame);
        break;
      case 0x7A:
        canzero_handle_input_board_stream_sdc_status(&frame);
        break;
      case 0x115:
        canzero_handle_pdu12_stream_power_estimation(&frame);
        break;
      case 0xB4:
        canzero_handle_pdu24_stream_state(&frame);
        break;
      case 0x74:
        canzero_handle_pdu24_stream_channel_status(&frame);
        break;
      case 0x75:
        canzero_handle_motor_driver_stream_errors(&frame);
        break;
      case 0xD5:
        canzero_handle_motor_driver_stream_state(&frame);
        break;
      case 0xDA:
        canzero_handle_mgu1_stream_errors(&frame);
        break;
      case 0x59:
        canzero_handle_mgu1_stream_state(&frame);
        break;
      case 0x99:
        canzero_handle_mgu2_stream_power_estimation(&frame);
        break;
      case 0xB9:
        canzero_handle_mgu2_stream_sdc_status(&frame);
        break;
      case 0xF8:
        canzero_handle_mlu2_stream_state(&frame);
        break;
      case 0xB8:
        canzero_handle_mlu2_stream_power_estimation(&frame);
        break;
      case 0xD8:
        canzero_handle_mlu2_stream_sdc_status(&frame);
        break;
      case 0x118:
        canzero_handle_mlu3_stream_errors(&frame);
        break;
      case 0x117:
        canzero_handle_mlu4_stream_state(&frame);
        break;
      case 0xD7:
        canzero_handle_mlu4_stream_power_estimation(&frame);
        break;
      case 0xF7:
        canzero_handle_mlu4_stream_sdc_status(&frame);
        break;
      case 0x56:
        canzero_handle_mlu5_stream_errors(&frame);
        break;
      case 0x55:
        canzero_handle_mlu6_stream_state(&frame);
        break;
      case 0xF6:
        canzero_handle_mlu6_stream_power_estimation(&frame);
        break;
      case 0x116:
        canzero_handle_mlu6_stream_sdc_status(&frame);
        break;
    }
  }
}
uint32_t canzero_update_continue(uint32_t time){
  schedule_jobs(time);
  return scheduler_next_job_timeout();
}
void canzero_init() {
  canzero_can0_setup(1000000, NULL, 0);
  canzero_can1_setup(1000000, NULL, 0);

  job_pool_allocator_init();
  scheduler.size = 0;
  schedule_heartbeat_job();
  schedule_mlu_control_interval_job();
  schedule_mgu_control_interval_job();
  schedule_motor_control_interval_job();
  schedule_pdu24_control_interval_job();
  schedule_pdu12_control_interval_job();
  schedule_global_state_interval_job();
  schedule_mlu_pid_sync_interval_job();

}
void canzero_set_mlu_command(mlu_command value) {
  extern mlu_command __oe_mlu_command;
  if (__oe_mlu_command != value) {
    __oe_mlu_command = value;
    uint32_t time = canzero_get_time();
    if (mlu_control_interval_job.climax > mlu_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      mlu_control_interval_job.climax = mlu_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&mlu_control_interval_job);
    }
  }
}
void canzero_set_mgu_command(mgu_command value) {
  extern mgu_command __oe_mgu_command;
  if (__oe_mgu_command != value) {
    __oe_mgu_command = value;
    uint32_t time = canzero_get_time();
    if (mgu_control_interval_job.climax > mgu_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      mgu_control_interval_job.climax = mgu_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&mgu_control_interval_job);
    }
  }
}
void canzero_set_motor_command(motor_command value) {
  extern motor_command __oe_motor_command;
  if (__oe_motor_command != value) {
    __oe_motor_command = value;
    uint32_t time = canzero_get_time();
    if (motor_control_interval_job.climax > motor_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      motor_control_interval_job.climax = motor_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&motor_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel1_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel1_control;
  if (__oe_pdu24_lp_channel1_control != value) {
    __oe_pdu24_lp_channel1_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel2_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel2_control;
  if (__oe_pdu24_lp_channel2_control != value) {
    __oe_pdu24_lp_channel2_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel3_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel3_control;
  if (__oe_pdu24_lp_channel3_control != value) {
    __oe_pdu24_lp_channel3_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel4_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel4_control;
  if (__oe_pdu24_lp_channel4_control != value) {
    __oe_pdu24_lp_channel4_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel5_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel5_control;
  if (__oe_pdu24_lp_channel5_control != value) {
    __oe_pdu24_lp_channel5_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel6_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel6_control;
  if (__oe_pdu24_lp_channel6_control != value) {
    __oe_pdu24_lp_channel6_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel7_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel7_control;
  if (__oe_pdu24_lp_channel7_control != value) {
    __oe_pdu24_lp_channel7_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel8_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel8_control;
  if (__oe_pdu24_lp_channel8_control != value) {
    __oe_pdu24_lp_channel8_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel9_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel9_control;
  if (__oe_pdu24_lp_channel9_control != value) {
    __oe_pdu24_lp_channel9_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel10_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel10_control;
  if (__oe_pdu24_lp_channel10_control != value) {
    __oe_pdu24_lp_channel10_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel11_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel11_control;
  if (__oe_pdu24_lp_channel11_control != value) {
    __oe_pdu24_lp_channel11_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_lp_channel12_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_lp_channel12_control;
  if (__oe_pdu24_lp_channel12_control != value) {
    __oe_pdu24_lp_channel12_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_hp_channel1_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_hp_channel1_control;
  if (__oe_pdu24_hp_channel1_control != value) {
    __oe_pdu24_hp_channel1_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_hp_channel2_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_hp_channel2_control;
  if (__oe_pdu24_hp_channel2_control != value) {
    __oe_pdu24_hp_channel2_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_hp_channel3_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_hp_channel3_control;
  if (__oe_pdu24_hp_channel3_control != value) {
    __oe_pdu24_hp_channel3_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu24_hp_channel4_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu24_hp_channel4_control;
  if (__oe_pdu24_hp_channel4_control != value) {
    __oe_pdu24_hp_channel4_control = value;
    uint32_t time = canzero_get_time();
    if (pdu24_control_interval_job.climax > pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu24_control_interval_job.climax = pdu24_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu24_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel1_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel1_control;
  if (__oe_pdu12_lp_channel1_control != value) {
    __oe_pdu12_lp_channel1_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel2_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel2_control;
  if (__oe_pdu12_lp_channel2_control != value) {
    __oe_pdu12_lp_channel2_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel3_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel3_control;
  if (__oe_pdu12_lp_channel3_control != value) {
    __oe_pdu12_lp_channel3_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel4_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel4_control;
  if (__oe_pdu12_lp_channel4_control != value) {
    __oe_pdu12_lp_channel4_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel5_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel5_control;
  if (__oe_pdu12_lp_channel5_control != value) {
    __oe_pdu12_lp_channel5_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel6_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel6_control;
  if (__oe_pdu12_lp_channel6_control != value) {
    __oe_pdu12_lp_channel6_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel7_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel7_control;
  if (__oe_pdu12_lp_channel7_control != value) {
    __oe_pdu12_lp_channel7_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel8_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel8_control;
  if (__oe_pdu12_lp_channel8_control != value) {
    __oe_pdu12_lp_channel8_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel9_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel9_control;
  if (__oe_pdu12_lp_channel9_control != value) {
    __oe_pdu12_lp_channel9_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel10_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel10_control;
  if (__oe_pdu12_lp_channel10_control != value) {
    __oe_pdu12_lp_channel10_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel11_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel11_control;
  if (__oe_pdu12_lp_channel11_control != value) {
    __oe_pdu12_lp_channel11_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_lp_channel12_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_lp_channel12_control;
  if (__oe_pdu12_lp_channel12_control != value) {
    __oe_pdu12_lp_channel12_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_hp_channel1_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_hp_channel1_control;
  if (__oe_pdu12_hp_channel1_control != value) {
    __oe_pdu12_hp_channel1_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_hp_channel2_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_hp_channel2_control;
  if (__oe_pdu12_hp_channel2_control != value) {
    __oe_pdu12_hp_channel2_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_hp_channel3_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_hp_channel3_control;
  if (__oe_pdu12_hp_channel3_control != value) {
    __oe_pdu12_hp_channel3_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_pdu12_hp_channel4_control(pdu_channel_control value) {
  extern pdu_channel_control __oe_pdu12_hp_channel4_control;
  if (__oe_pdu12_hp_channel4_control != value) {
    __oe_pdu12_hp_channel4_control = value;
    uint32_t time = canzero_get_time();
    if (pdu12_control_interval_job.climax > pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0) {
      pdu12_control_interval_job.climax = pdu12_control_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&pdu12_control_interval_job);
    }
  }
}
void canzero_set_global_state(global_state value) {
  extern global_state __oe_global_state;
  if (__oe_global_state != value) {
    __oe_global_state = value;
    uint32_t time = canzero_get_time();
    if (global_state_interval_job.climax > global_state_interval_job.job.stream_interval_job.last_schedule + 0) {
      global_state_interval_job.climax = global_state_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&global_state_interval_job);
    }
  }
}
void canzero_set_command(global_command value) {
  extern global_command __oe_command;
  if (__oe_command != value) {
    __oe_command = value;
    uint32_t time = canzero_get_time();
    if (global_state_interval_job.climax > global_state_interval_job.job.stream_interval_job.last_schedule + 0) {
      global_state_interval_job.climax = global_state_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&global_state_interval_job);
    }
  }
}
void canzero_set_mlu_pid_values(mlu_pid_values value) {
  extern mlu_pid_values __oe_mlu_pid_values;
  if (0 || __oe_mlu_pid_values.p_value != value.p_value || __oe_mlu_pid_values.i_value != value.i_value || __oe_mlu_pid_values.d_value != value.d_value) {
    __oe_mlu_pid_values = value;
    uint32_t time = canzero_get_time();
    if (mlu_pid_sync_interval_job.climax > mlu_pid_sync_interval_job.job.stream_interval_job.last_schedule + 0) {
      mlu_pid_sync_interval_job.climax = mlu_pid_sync_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&mlu_pid_sync_interval_job);
    }
  }
}
void canzero_set_sdc_status(sdc_status value) {
  extern sdc_status __oe_sdc_status;
  if (__oe_sdc_status != value) {
    __oe_sdc_status = value;
    uint32_t time = canzero_get_time();
    if (global_state_interval_job.climax > global_state_interval_job.job.stream_interval_job.last_schedule + 0) {
      global_state_interval_job.climax = global_state_interval_job.job.stream_interval_job.last_schedule + 0;
      scheduler_promote_job(&global_state_interval_job);
    }
  }
}
