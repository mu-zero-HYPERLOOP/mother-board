#include "canzero.h"

/// returns true iff. the pod should decelerate
static bool velocity_profile_decelerate() {
  const float brake_position_front =
      canzero_get_track_length() - canzero_get_brake_margin();
  const float brake_position_back = canzero_get_brake_margin();
  const float expected_brake_position =
      canzero_get_position() +
      (canzero_get_velocity() * canzero_get_velocity()) /
          (canzero_get_target_acceleration() * 2.0f);

  if (brake_position_front < expected_brake_position &&
      canzero_get_velocity() > 0.5) {
    return true;
  }

  if (brake_position_back > expected_brake_position &&
      canzero_get_velocity() < 0.5) {
    return true;
  }
  return false;
}

