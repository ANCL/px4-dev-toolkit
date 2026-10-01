#include <chrono>
#include <iostream>
#include <stdexcept>

#include <offboard_controllers/runtime_timing.hpp>

namespace
{

void expect_equal(
  const char * name,
  long long actual,
  long long expected)
{
  if (actual != expected) {
    std::cerr
      << name
      << ": expected " << expected
      << ", got " << actual
      << '\n';

    throw std::runtime_error("Runtime timing test failed.");
  }
}


void test_100_hz_timing_contract()
{
  const auto timing =
    offboard_controllers::make_runtime_timing(
      100.0,  // control rate [Hz]
      1.5,    // Offboard prestream [s]
      1.0);   // Offboard retry interval [s]

  expect_equal(
    "control period us",
    std::chrono::duration_cast<std::chrono::microseconds>(
      timing.control_period).count(),
    10000);

  expect_equal(
    "warmup samples",
    timing.warmup_samples,
    150);

  expect_equal(
    "retry samples",
    timing.retry_samples,
    100);
}


void test_invalid_rate_is_rejected()
{
  bool threw = false;

  try {
    (void)offboard_controllers::make_runtime_timing(
      0.0,
      1.5,
      1.0);
  } catch (const std::invalid_argument &) {
    threw = true;
  }

  if (!threw) {
    throw std::runtime_error(
            "Runtime timing accepted a non-positive control rate.");
  }
}

}  // namespace


int main()
{
  test_100_hz_timing_contract();
  test_invalid_rate_is_rejected();

  std::cout << "Runtime timing tests passed.\n";
  return 0;
}
