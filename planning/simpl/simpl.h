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

  // using shared_ptr because of OMPL design patterns
  std::shared_ptr<OccupancyGrid> grid_ = nullptr;

public:
  Simpl() = delete;
  Simpl(Scenario scenario);

  // called in a tight loop
  void update(int64_t time);
  const Footprints getVisibleFootprints(int64_t time) const;
  const std::shared_ptr<OccupancyGrid> &getLastOccupancyGrid() const;
  const ProximityPlanner &getProximityPlanner() const;
  const MockRevoyEv &getRevoyEv() const;
  const Scenario &getScenario() const;
  bool isDone(int64_t time) const;
};

} // namespace planning
