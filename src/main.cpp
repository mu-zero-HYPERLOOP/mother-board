/**
 * @author      : kistenklaus (karlsasssie@gmail.com)
 * @file        : main
 * @created     : Freitag Apr 12, 2024 15:10:53 CEST
 */

#include "canzero.h"
#include "can/can.hpp"
#include "states.h"
#include <unistd.h>

int main() {

  canzero_init();
  // starts the can0_poll, can1_poll and canzero update threads
  can_threads_start(); 

  printf("%lu\n", canzero_get_config_hash());

  while(true) {
    sleep(10);
  }

}
