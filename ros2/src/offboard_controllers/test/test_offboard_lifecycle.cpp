#include <cstdlib>
#include <iostream>

#include <offboard_controllers/offboard_lifecycle.hpp>
#include <offboard_controllers/offboard_trigger.hpp>

using offboard_controllers::OffboardTransition;
using offboard_controllers::offboard_transition;

namespace
{

void require_equal(
  OffboardTransition actual,
  OffboardTransition expected,
  const char * label)
{
  if (actual != expected) {
    std::cerr << "Unexpected transition: " << label << '\n';
    std::exit(EXIT_FAILURE);
  }
}

}  // namespace

int main()
{
  require_equal(
    offboard_transition(false, false),
    OffboardTransition::NONE,
    "inactive -> inactive");

  require_equal(
    offboard_transition(false, true),
    OffboardTransition::ENTERED,
    "inactive -> active");

  require_equal(
    offboard_transition(true, true),
    OffboardTransition::NONE,
    "active -> active");

  require_equal(
    offboard_transition(true, false),
    OffboardTransition::LOST,
    "active -> inactive");

  using offboard_controllers::OffboardTrigger;
  using offboard_controllers::parse_offboard_trigger;
  using offboard_controllers::requests_offboard;
  using offboard_controllers::requests_position_return;
  using offboard_controllers::offboard_ready;

  if (!requests_offboard(parse_offboard_trigger("automatic")) ||
      requests_offboard(parse_offboard_trigger("external"))) {
    std::cerr << "Incorrect Offboard activation policy.\n";
    return EXIT_FAILURE;
  }

  if (!requests_position_return(OffboardTrigger::AUTOMATIC) ||
      requests_position_return(OffboardTrigger::EXTERNAL)) {
    std::cerr << "Incorrect Position return policy.\n";
    return EXIT_FAILURE;
  }

  bool rejected_invalid = false;
  try {
    (void)parse_offboard_trigger("unsupported");
  } catch (const std::invalid_argument&) {
    rejected_invalid = true;
  }
  if (!rejected_invalid) return EXIT_FAILURE;

  if (offboard_ready(false, true, 30, 30) ||
      offboard_ready(true, false, 30, 30) ||
      offboard_ready(true, true, 29, 30) ||
      !offboard_ready(true, true, 30, 30)) {
    std::cerr << "Incorrect Offboard readiness gate.\n";
    return EXIT_FAILURE;
  }

  std::cout << "Offboard lifecycle tests passed.\n";
  return EXIT_SUCCESS;
}
