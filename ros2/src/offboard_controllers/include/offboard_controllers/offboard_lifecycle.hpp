#pragma once

namespace offboard_controllers
{

enum class OffboardTransition
{
  NONE,
  ENTERED,
  LOST,
};


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
