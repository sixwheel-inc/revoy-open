#pragma once

#include "planning/types.h"

namespace planning {

class MockRevoyEv {

public:
  MockRevoyEv() = default;
  MockRevoyEv(HookedPose start);

  void update(const Controls &controls, double duration);

  const Footprints getBody(const BodyParams &params) const;
  const HookedPose getHookedPose() const;

  void print(const BodyParams &params) const;

private:
  HookedPose hookedPose_;
};

} // namespace planning
