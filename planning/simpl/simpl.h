#pragma once

#include "planning/mock-revoy-ev.h"
#include "planning/proximity-planner.h"
#include "planning/types.h"

namespace planning {

// Simpl: Simulator for OMPL, focusing on SE(2) and similar "2D" spaces
class Simpl {

  // defines what will happen in the simulation
  Scenario scenario_;

  // simple vehicle model to move around in the simulation
  MockRevoyEv revoyEv_;

  // the planner, outputs commands to the vehicle model
  ProximityPlanner proximityPlanner_;

  // obstacles
  // using shared_ptr because of OMPL design patterns
  std::shared_ptr<OccupancyGrid> grid_ = nullptr;

public:
  // initialize simulation with the given scenario, the only permitted
  // way to initialize Simpl
  Simpl(Scenario scenario);
  Simpl() = delete;

  // called in a tight loop
  void update(int64_t time);

  // true when exit condition is met
  bool isDone(int64_t time) const;

  // TODO: leaky abstraction, directly access the planner, somewha
  const ProximityPlanner &getProximityPlanner() const;

  // the following ar used for debug visualization
  const MockRevoyEv &getRevoyEv() const;
  const Scenario &getScenario() const;
  const Footprints getVisibleFootprints(int64_t time) const;
  const std::shared_ptr<OccupancyGrid> &getLastOccupancyGrid() const;
};

} // namespace planning
