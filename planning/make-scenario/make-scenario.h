#pragma once

#include "planning/types.h"

namespace planning {

/// @brief returns Scenario where there is an obstacle blocking the way, and it
/// disappears after a while.
Scenario MakeDisappearingObstacleScenario(double dir, double dist);

/// @brief return Yard Scenario (WIP)
Scenario MakeYardScenario();

} // namespace planning
