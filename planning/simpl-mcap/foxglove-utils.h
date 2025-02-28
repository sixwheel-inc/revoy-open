#pragma once

#include "planning/simpl-mcap.h"
#include "planning/types.h"

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include "foxglove/Grid.pb.h"
#include "foxglove/LinePrimitive.pb.h"
#include "foxglove/SceneUpdate.pb.h"

namespace planning {

/// eigen helper
Eigen::Quaterniond MakeQuaternion(double angle);

/// foxglove proto helpers, convert planner geometries into visualizer protobufs
foxglove::SceneUpdate MakeActorSceneUpdate(const Scene &scene, int64_t time);
foxglove::SceneUpdate MakeGraphSceneUpdate(const Graph &graph, int64_t time);
foxglove::SceneUpdate MakePathSceneUpdate(const Path &path, int64_t time);
foxglove::SceneUpdate MakeScenarioSceneUpdate(const Scenario &scenario,
                                              double writeTime);
foxglove::LinePrimitive MakePath(const Path &path);
foxglove::LinePrimitive MakeFootprint(const Footprint &footprint);
foxglove::ArrowPrimitive MakeArrowPrimitive(const Pose &pose);
foxglove::Grid MakeGrid(const std::shared_ptr<OccupancyGrid> &grid,
                        const HookedPose &revoyPose, int64_t writeTime);

// will be used
// foxglove::SceneUpdate MakePathSceneUpdate(const std::vector<Pose> &poses,
//                                           int64_t writeTime);
// foxglove::SceneUpdate MakePathSceneUpdate(const std::vector<HookedPose>
// &poses,
//                                           int64_t writeTime);

} // namespace planning
