#include "canzero/canzero.h"
#include <algorithm>
#include <array>
#include <cinttypes>
#include <cstdio>

static std::array<bool, node_id_count + MAX_DYN_HEARTBEATS> miss_can0;
static std::array<bool, node_id_count + MAX_DYN_HEARTBEATS> miss_can1;

void check_full_recover() {
  if (!std::any_of(miss_can0.begin(), miss_can0.end(), [](auto m) { return m; }) &&
      !std::any_of(miss_can1.begin(), miss_can1.end(), [](auto m) { return m; })) {
    canzero_set_error_heartbeat_miss(error_flag_OK);
    printf("CANx Full Heartbeat Recovery\n");
  }
}

void canzero_can0_wdg_timeout(uint8_t node_id) {
  printf("CAN0 Heartbeat Miss [node=%u]\n", node_id);
  canzero_set_error_heartbeat_miss(error_flag_ERROR);
  miss_can0[node_id] = true;
}

void canzero_can1_wdg_timeout(uint8_t node_id) {
  printf("CAN1 Heartbeat Miss [node=%u]\n", node_id);
  canzero_set_error_heartbeat_miss(error_flag_ERROR);
  miss_can1[node_id] = true;
}

void canzero_can0_wdg_recovered(uint8_t node_id) {
  miss_can0[node_id] = false;
  printf("CAN0 Heartbeat Recovery [node=%u]\n", node_id);
  check_full_recover();
}

void canzero_can1_wdg_recovered(uint8_t node_id) {
  miss_can1[node_id] = false;
  printf("CAN Heartbeat Recovery [node=%u]\n", node_id);
  check_full_recover();
}

