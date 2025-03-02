#include "planning/simpl.h"
#include "planning/add-footprint-to-grid.h"
#include "planning/footprint-overlap.h"
#include "planning/footprint-transform.h"
#include "planning/occupancy-grid.h"
#include "planning/types.h"

#include <cstdint>
#include <memory>
#include <string>

namespace planning {

namespace {
/// 200 * 200 cells = 40000 cells
/// 0.1m * 200 cells means grid total diagonal is ~28m, effective range ~14m
static constexpr uint16_t NUM_CELLS = 200;
static constexpr double CELL_LENGTH = 0.1; // meters
static constexpr double GRID_OFFSET =
    CELL_LENGTH * ((float)NUM_CELLS) / 2.0; // meters

} // namespace

Simpl::Simpl(Scenario scenario)
    : scenario_(scenario), revoyEv_(scenario_.start),
      proximityPlanner_(scenario_.bounds, scenario_.bodyParams),
      grid_(std::make_shared<OccupancyGrid>(NUM_CELLS, NUM_CELLS, CELL_LENGTH,
                                            CELL_LENGTH, GRID_OFFSET,
                                            GRID_OFFSET)) {};

void Simpl::update(int64_t time) {

  // get simulated obstacle footprints from simulated observers
  const Footprints footprints = getVisibleFootprints(time);

  // insert footprints into occupancy grid
  FootprintsToOccupancyGrid(*grid_, footprints, revoyEv_.getHookedPose());

  // update plan w/ latest revoy pose and occupancy grid
  proximityPlanner_.plan(revoyEv_.getHookedPose(), scenario_.goal, grid_);

  // get controls from caller
  const Controls controls = proximityPlanner_.getControls();

  // update revoy w/ controls
  revoyEv_.update(controls, scenario_.timeParams.dt / 1e6);
}

bool Simpl::isDone(int64_t time) const {

  // timeout
  if (time > scenario_.timeParams.timeout + scenario_.timeParams.startTime) {
    return true;
  }

  // revoy footprint at the origin, used to check against grid cells
  const Footprints bodyZero = FootprintsFromPose(
      {{0, 0},
       0,
       revoyEv_.getHookedPose().trailerYaw - revoyEv_.getHookedPose().yaw},
      scenario_.bodyParams);

  // check if revoy footprint cells are occupied by obstacles,
  // if so, then no planning is possible.
  const auto grid = getLastOccupancyGrid();
  bool invalidStart = false;
  if (grid && grid->areFootprintsOccupied(bodyZero)) {
    invalidStart = true;
  }

  // goal is met, scenario is complete
  // TODO: use GoalRegion comparison
  static const double GOAL_TOLERANCE = 0.1;
  const bool isGoal =
      (revoyEv_.getHookedPose().position - scenario_.goal.position).norm() <=
          GOAL_TOLERANCE &&
      fabs(fixRadian(revoyEv_.getHookedPose().yaw - scenario_.goal.yaw)) <=
          GOAL_TOLERANCE &&
      fabs(fixRadian(revoyEv_.getHookedPose().trailerYaw -
                     scenario_.goal.trailerYaw)) <= GOAL_TOLERANCE;

  return isGoal || invalidStart;
}

const Footprints Simpl::getVisibleFootprints(int64_t time) const {
  Footprints footprints;
  for (const auto &entity : scenario_.entities) {
    if (static_cast<double>(time) >
        scenario_.timeParams.startTime + entity.lifetime) {
      continue;
    }

    const auto tfFootprint = TransformFootprint(entity.footprint, entity.pose);
    footprints.push_back(tfFootprint);
  }
  return footprints;
}

const std::shared_ptr<OccupancyGrid> &Simpl::getLastOccupancyGrid() const {
  return grid_;
}

const Scenario &Simpl::getScenario() const { return scenario_; }
const ProximityPlanner &Simpl::getProximityPlanner() const {
  return proximityPlanner_;
};
const MockRevoyEv &Simpl::getRevoyEv() const { return revoyEv_; };

} // namespace planning
