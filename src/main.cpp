/**
 * @author      : kistenklaus (karlsasssie@gmail.com)
 * @file        : main
 * @created     : Freitag Apr 12, 2024 15:10:53 CEST
 */

#include "canzero/canzero.h"
#include "control/velocity.h"
#include "fsm/fsm.h"
#include <cstdio>
#include <unistd.h>

constexpr float MAX_ACCEL = 9.81;

int main() {

  canzero_init();

  fsm::begin();
  control::velocity::begin();

  while(true) {

    canzero_can0_poll();
    canzero_can1_poll();
  
    fsm::update();
    control::velocity::update();

    printf("%d\n", canzero_get_state());

    canzero_update_continue(canzero_get_time());

    usleep(10);

  }

}
