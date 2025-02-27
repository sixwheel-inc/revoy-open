#pragma once

#include "planning/simpl-mcap.h"
#include "planning/simpl.h"

namespace planning {

// some massaging is required to go from Simpl to visualizable things,
// e.g. some paths need to be interpolated (to show curvature), graph nodes need
// to be collected, etc.
Scene SimplToScene(std::unique_ptr<Simpl> &simpl, int64_t time);

} // namespace planning
