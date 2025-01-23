#pragma once

#include "planning/types.h"

#include <vector>

namespace planning {

Footprint TransformFootprint(const Footprint &box, const Pose &pose);
Footprint ReverseTransformFootprint(const Footprint &box, const Pose &pose);
Footprints FootprintsFromPose(const HookedPose &pose, const BodyParams &params);
Point RotatePoint(const Point &point, double yaw);

} // namespace planning
