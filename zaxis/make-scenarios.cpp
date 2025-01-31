#include "zaxis/make-scenarios.h"

#include <format>
#include <string>

namespace revoy {

namespace {
static constexpr double SPEED_TOLERANCE = 0.5;
static constexpr double SPEED_P_CONTROL_REGIME = 0.5;
static constexpr double HEADING_TOLERANCE = 0.05;
static constexpr double HEADING_P_CONTROL_REGIME = 0;

Goal MakeGoal(double speed, double heading, double duration) {
  return {
      {(float)speed, SPEED_TOLERANCE, SPEED_P_CONTROL_REGIME},
      {(float)heading, HEADING_TOLERANCE, HEADING_P_CONTROL_REGIME},
      (float)duration,
  };
};

} // namespace

Zaxis::Scenario MakeSwervingScenario(double speed, double absHeading,
                                     uint8_t count) {

  Zaxis::Scenario scenario;
  scenario.name = std::format("swerving-speed-{:.1f}-heading-{:.1f}-count-{}",
                              speed, absHeading, count);

  Goal startGoal = MakeGoal(speed, 0, 0);

  /// must reach the desired speed before oscillating
  scenario.goals.emplace_back(std::move(startGoal));

  /// oscillate
  for (uint8_t i = 0; i < count * 2; i++) {
    const float dir = ((i % 2) == 0) ? 1 : -1;
    const float heading = dir * (float)absHeading;
    Goal goal = MakeGoal(speed, heading, 0);
    scenario.goals.emplace_back(std::move(goal));
  }
  return scenario;
};

Zaxis::Scenario MakeStartStopScenario(double speed) {

  Zaxis::Scenario scenario;
  scenario.name = std::format("start-stop-{:.1f}", speed);

  /// must reach the desired speed before stopping
  Goal startGoal = MakeGoal(speed, 0, 0);
  scenario.goals.emplace_back(std::move(startGoal));
  Goal endGoal = MakeGoal(0, 0, 0);
  scenario.goals.emplace_back(std::move(endGoal));

  return scenario;
};

} // namespace revoy
