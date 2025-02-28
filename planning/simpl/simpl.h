#pragma once

#include "planning/mock-revoy-ev.h"
#include "planning/proximity-planner.h"
#include "planning/types.h"

namespace planning {

// Simpl: Simulator for OMPL, focusing on SE(2) and similar "2D" spaces
class Simpl {
  Scenario scenario_;
  MockRevoyEv revoyEv_;
  ProximityPlanner proximityPlanner_;

public:
  Simpl() = delete;
  Simpl(Scenario scenario);

  // called in a tight loop
  void update(int64_t time, double actualSpeed, double actualSteer);
  const Footprints getVisibleFootprints(int64_t time) const;
  const ProximityPlanner &getProximityPlanner() const;
  const MockRevoyEv &getRevoyEv() const;
  const Scenario &getScenario() const;
  bool isDone() const;
};

} // namespace planning
