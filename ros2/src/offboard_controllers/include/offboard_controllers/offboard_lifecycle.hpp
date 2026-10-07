#pragma once

namespace offboard_controllers
{

enum class OffboardTransition
{
  NONE,
  ENTERED,
  LOST,
};


// Offboard-transition contract:
//
// Inputs:
//   previous and current interpretations of PX4's authoritative nav_state.
//
// Logic:
//   classify only edges of Offboard ownership.
//
// Output:
//   ENTERED, LOST, or NONE for lifecycle initialization and teardown.
inline OffboardTransition offboard_transition(
  bool was_active,
  bool is_active)
{
  if (!was_active && is_active) {
    return OffboardTransition::ENTERED;
  }

  if (was_active && !is_active) {
    return OffboardTransition::LOST;
  }

  return OffboardTransition::NONE;
}

}  // namespace offboard_controllers
