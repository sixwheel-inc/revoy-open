#pragma once

#include "planning/types.h"

namespace planning {

class MockRevoyEv {

public:
  MockRevoyEv() = default;
  MockRevoyEv(HookedPose start);

  void update(const Controls &controls, const Bounds &bounds, double duration);

  const Footprints getBody(const BodyParams &params) const;
  const HookedPose getHookedPose() const;

private:
  HookedPose hookedPose_;
};

} // namespace planning
