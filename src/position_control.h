#pragma once


#include "canzero.h"
#include <chrono>
#include <mutex>
#include <thread>
struct PositionControl {
public:
  void start() {
    m_thread = std::thread(&PositionControl::control_thread, this);
  }

  void set_target_position(float position) {
    std::unique_lock<std::mutex> lck(m_mutex);
    m_target_position = position;
  }

  float get_target_position() {
    std::unique_lock<std::mutex> lck(m_mutex);
    return m_target_position;
  }


private:

  void control_thread() {
    using namespace std::chrono;
    while(true) {
      float position = canzero_get_position();
      float velocity = canzero_get_velocity();
      float acceleraiton = canzero_get_acceleration();
      
      float target_position = get_target_position();

      // controllerio

      canzero_set_target_acceleration(0.0);

      std::this_thread::sleep_for(10ms);
    }
  }

  float m_target_position;

  std::mutex m_mutex;
  std::thread m_thread;

};
