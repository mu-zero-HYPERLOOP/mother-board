/**
 * @author      : kistenklaus (karlsasssie@gmail.com)
 * @file        : main
 * @created     : Freitag Apr 12, 2024 15:10:53 CEST
 */

#include "fsm.h"
#include "sdc.hpp"

int main() {

  fsm_init();

  while(true) {
    update_sdc_status();
    fsm_next();
  }

}
