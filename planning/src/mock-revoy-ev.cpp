#include "planning/mock-revoy-ev.h"
#include "planning/footprint-transform.h"
#include "planning/types.h"

#include <iostream>
#include <limits>

namespace planning {

MockRevoyEv::MockRevoyEv(HookedPose start) : hookedPose_(start) {};

void MockRevoyEv::update(const Controls &controls, const Bounds &bounds,
                         double duration) {
  const double travel = controls.speed * duration; /// K;

  double x = hookedPose_.position.x();
  double y = hookedPose_.position.y();
  double yaw = hookedPose_.yaw;
  double trailerYaw = hookedPose_.trailerYaw;

  static const double K = 100;
  for (int i = 0; i < K; i++) {
    x += travel * cos(yaw) / K;
    y += travel * sin(yaw) / K;
    yaw += travel * tan(controls.steer) / K;
    trailerYaw -= sin(trailerYaw - yaw) * duration / K;
  }

  hookedPose_.position.x() = fmax(fmin(x, bounds.upperX), bounds.lowerX);
  hookedPose_.position.y() = fmax(fmin(y, bounds.upperY), bounds.lowerY);
  hookedPose_.yaw = fixRadian(yaw);
  hookedPose_.trailerYaw = fixRadian(trailerYaw);
}

const HookedPose MockRevoyEv::getHookedPose() const { return hookedPose_; }

const Footprints MockRevoyEv::getBody(const BodyParams &params) const {

  return FootprintsFromPose(getHookedPose(), params);
}

} // namespace planning
