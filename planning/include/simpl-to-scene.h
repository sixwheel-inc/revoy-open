#pragma once

#include "planning/mcap-utils.h"
#include "planning/simpl.h"

namespace planning {

Scene SimplToScene(std::unique_ptr<Simpl> &simpl, int64_t time);
} // namespace planning
