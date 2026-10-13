// Drive and safely center MAVProxy manual control during Offboard staging.

#include "node.hpp"

bool OffboardTakeoffHandoff::set_joystick(double throttle)
{
  try {
    virtual_joystick_.set(0.0, 0.0, throttle, 0.0);
    return true;

  } catch (const std::exception & error) {
    fail(error.what());
    return false;
  }
}


bool OffboardTakeoffHandoff::center_joystick()
{
  try {
    virtual_joystick_.center();
    return true;

  } catch (const std::exception & error) {
    fail(error.what());
    return false;
  }
}


void OffboardTakeoffHandoff::leave_joystick_safe()
{
  // An armed Position-mode vehicle is left with centered sticks so PX4 holds
  // its current state. Minimum throttle is restored only when disarmed.
  try {
    if (
      arming_state_ ==
      px4_msgs::msg::VehicleStatus::ARMING_STATE_ARMED)
    {
      virtual_joystick_.center();

    } else {
      virtual_joystick_.set(0.0, 0.0, kPrearmThrottle, 0.0);
    }

  } catch (const std::exception &) {
    // Failure reporting must not recursively fail while shutting down.
  }
}
