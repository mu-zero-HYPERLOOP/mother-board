#pragma once

// updates the combined sdc status of all sdc switches
#include "canzero.h"
inline void update_sdc_status() {
  canzero_set_sdc_status(
      canzero_get_mlu1_sdc_status() == sdc_status_CLOSED &&
              canzero_get_mlu2_sdc_status() == sdc_status_CLOSED &&
              canzero_get_mlu3_sdc_status() == sdc_status_CLOSED &&
              canzero_get_mlu4_sdc_status() == sdc_status_CLOSED &&
              canzero_get_mlu5_sdc_status() == sdc_status_CLOSED &&
              canzero_get_mlu6_sdc_status() == sdc_status_CLOSED &&
              canzero_get_mgu1_sdc_status() == sdc_status_CLOSED &&
              canzero_get_mgu2_sdc_status() == sdc_status_CLOSED &&
              canzero_get_motor_sdc_status() == sdc_status_CLOSED &&
              canzero_get_pdu12_sdc_status() == sdc_status_CLOSED &&
              canzero_get_pdu24_sdc_status() == sdc_status_CLOSED &&
              canzero_get_input_board_sdc_status() == sdc_status_CLOSED
          ? sdc_status_CLOSED
          : sdc_status_OPEN);
}
