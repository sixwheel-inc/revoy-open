#pragma once

#include "planning/mock-revoy-ev.h"
#include "planning/proximity-planner.h"
#include "planning/types.h"

namespace planning {

class MockProximityObserver {
public:
  Footprints getFootprints(const Scenario &scenario, const HookedPose &pose,
                           int64_t time) const;
};

class Simpl {
  Scenario scenario_;
  MockProximityObserver proximityObserver_;
  MockRevoyEv revoyEv_;
  ProximityPlanner proximityPlanner_;

public:
  Simpl() = delete;
  Simpl(Scenario scenario);
  void update(int64_t time, double actualSpeed, double actualSteer);
  const Footprints getVisibleFootprints(int64_t time) const;
  const ProximityPlanner &getProximityPlanner() const;
  const MockRevoyEv &getRevoyEv() const;
  const Scenario &getScenario() const;
  bool isDone() const;
};

} // namespace planning
