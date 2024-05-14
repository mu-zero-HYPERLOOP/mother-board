#include "can/socketcan.h"
#include "canzero/canzero.h"
#include "util/timestamp.h"
#include <cassert>
#include <chrono>
#include <condition_variable>
#include <cstring>
#include <mutex>
#include <thread>

static socketcan_socket s_can0;

void canzero_can0_setup(uint32_t baudrate, canzero_can_filter *filters,
                        int filter_count) {
  assert(!socketcan_socket_open(&s_can0, SOCKETCAN_BUS_CAN0, nullptr, 0));
}
void canzero_can0_send(canzero_frame *frame) {
  socketcan_frame can_frame;
  can_frame.len = frame->dlc;
  memcpy(can_frame.data, frame->data, 8);
  can_frame.can_id = frame->id;
  socketcan_send_frame(&s_can0, &can_frame);
}
int canzero_can0_recv(canzero_frame *frame) {
  socketcan_frame can_frame;
  if (socketcan_recv_frame(&s_can0, &can_frame)) {
    return 0;
  }
  frame->dlc = can_frame.len;
  memcpy(frame->data, can_frame.data, 8);
  frame->id = can_frame.can_id;
  return 1;
}

static socketcan_socket s_can1;

void canzero_can1_setup(uint32_t baudrate, canzero_can_filter *filters,
                        int filter_count) {
  assert(!socketcan_socket_open(&s_can1, SOCKETCAN_BUS_CAN1, nullptr, 0));
}
void canzero_can1_send(canzero_frame *frame) {
  socketcan_frame can_frame;
  can_frame.len = frame->dlc;
  memcpy(can_frame.data, frame->data, 8);
  can_frame.can_id = frame->id;
  socketcan_send_frame(&s_can1, &can_frame);
}
int canzero_can1_recv(canzero_frame *frame) {
  socketcan_frame can_frame;
  if (socketcan_recv_frame(&s_can1, &can_frame)) {
    return 1;
  }
  frame->dlc = can_frame.len;
  memcpy(frame->data, can_frame.data, 8);
  frame->id = can_frame.can_id;
  return 1;
}

static Timestamp g_start = Timestamp::now();


static std::thread s_can0_poll_thread;
static std::thread s_can1_poll_thread;
static std::thread s_can_update_thread;

static std::condition_variable s_can_update_sleep_cv;
static std::mutex s_can_update_mutex;

static std::chrono::time_point<std::chrono::steady_clock> s_sor = std::chrono::steady_clock::now();

static std::chrono::time_point<std::chrono::steady_clock> s_can_next_update;


static void can_update_thread() {
  using namespace std::chrono;
  while (true) {
    std::unique_lock<std::mutex> lk(s_can_update_mutex);
    s_can_update_sleep_cv.wait_for(lk, s_can_next_update - steady_clock::now(), [] {
        return s_can_next_update < steady_clock::now();
    });
    canzero_update_continue(canzero_get_time());
  }
}

uint32_t canzero_get_time() { 
  using namespace std::chrono;
  return duration_cast<milliseconds>(steady_clock::now() - s_sor).count(); 
}

void canzero_request_update(uint32_t time) {
  using namespace std::chrono;
  {
    std::unique_lock<std::mutex> lk(s_can_update_mutex);
    s_can_next_update = s_sor + duration<long, std::milli>(time);
  }
  s_can_update_sleep_cv.notify_all();
  // pass
}

void can_threads_start() {
  s_can0_poll_thread = std::thread(canzero_can0_poll);
  s_can1_poll_thread = std::thread(canzero_can1_poll);
  s_can_update_thread = std::thread(can_update_thread);
}

static std::mutex s_canzero_critical;

void canzero_enter_critical() { s_canzero_critical.lock(); }
void canzero_exit_critical() { s_canzero_critical.unlock(); }
