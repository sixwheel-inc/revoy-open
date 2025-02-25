#include "planning/mock-revoy-ev.h"
#include "planning/footprint-transform.h"
#include "planning/include/mock-revoy-ev.h"
#include "planning/types.h"

#include <iostream>
#include <limits>

namespace planning {

MockRevoyEv::MockRevoyEv(HookedPose start) : hookedPose_(start) {};

void MockRevoyEv::update(const Controls &controls, double duration) {

  double x = hookedPose_.position.x();
  double y = hookedPose_.position.y();
  double yaw = hookedPose_.yaw;
  double trailerYaw = hookedPose_.trailerYaw;

  // integrate to try to make it more accurate
  static const double K = 100;
  const double travel = controls.speed * duration / K;
  /// hack todo fix
  const BodyParams param;

  for (int i = 0; i < K; i++) {
    x += travel * cos(yaw);
    y += travel * sin(yaw);
    yaw += (travel / param.revoyLength) * tan(controls.steer);
    trailerYaw -= (travel / param.trailerLength) * sin(trailerYaw - yaw);
  }

  hookedPose_.position.x() = x;
  hookedPose_.position.y() = y;
  hookedPose_.yaw = fixRadian(yaw);
  hookedPose_.trailerYaw = fixRadian(trailerYaw);
}

const HookedPose MockRevoyEv::getHookedPose() const { return hookedPose_; }

const Footprints MockRevoyEv::getBody(const BodyParams &params) const {

  return FootprintsFromPose(getHookedPose(), params);
}

void MockRevoyEv::print(const BodyParams &params) const {
  std::cout << "revoy feet" << std::endl;
  for (const auto &bodyPart : getBody(params)) {
    for (const auto &point : bodyPart) {
      std::cout << "    (" << point.x() << "," << point.y() << ")\n";
    }
  }
  std::cout << std::endl;
}

} // namespace planning
