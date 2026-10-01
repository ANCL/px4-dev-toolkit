#include <cstdlib>
#include <iostream>

#include <offboard_controllers/offboard_lifecycle.hpp>

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

  std::cout << "Offboard lifecycle tests passed.\n";
  return EXIT_SUCCESS;
}
