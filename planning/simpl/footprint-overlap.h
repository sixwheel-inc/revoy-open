#pragma once

#include "planning/types.h"

namespace planning {

bool AreFootprintsOverlapping(const Footprint &alpha, const Footprint &beta);
bool IsBodyCollidingAnyObstacles(const Footprints &body,
                                 const Footprints &obstacles);
bool IsPointInFootprint(const Point &point, const Footprint &footprint);

} // namespace planning
