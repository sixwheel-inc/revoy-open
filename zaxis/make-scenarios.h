#pragma once

#include "zaxis/zaxis.h"

namespace revoy {

/// swerve back and forth.
/// Vehicle will oscillate between +/- `heading`, up to `count` times,
/// while maintaining `speed`.
Zaxis::Scenario MakeSwervingScenario(double speed, double heading,
                                     uint8_t count);

/// speed up and then stop
/// Vehicle will accelerate to `speed` and then stop.
Zaxis::Scenario MakeStartStopScenario(double speed);

} // namespace revoy
