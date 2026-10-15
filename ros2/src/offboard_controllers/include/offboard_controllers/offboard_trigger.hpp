// Shared Offboard activation policy; the controller always observes PX4 mode.
#pragma once

#include <stdexcept>
#include <string>

namespace offboard_controllers {

enum class OffboardTrigger { AUTOMATIC, EXTERNAL };

inline OffboardTrigger parse_offboard_trigger(const std::string& value) {
  if (value == "automatic") return OffboardTrigger::AUTOMATIC;
  if (value == "external") return OffboardTrigger::EXTERNAL;
  throw std::invalid_argument("offboard_trigger must be automatic or external.");
}

inline bool requests_offboard(OffboardTrigger trigger) {
  return trigger == OffboardTrigger::AUTOMATIC;
}

inline bool offboard_ready(bool armed, bool state_ready, int prestream_count,
                           int required_prestream_count) {
  return armed && state_ready && prestream_count >= required_prestream_count;
}

}  // namespace offboard_controllers
