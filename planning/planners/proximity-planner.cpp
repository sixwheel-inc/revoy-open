#include "planning/proximity-planner.h"

#include "planning/footprint-transform.h"
#include "planning/occupancy-grid.h"
#include "planning/revoy-space.h"
#include "planning/types.h"

#include <ompl/base/State.h>
#include <ompl/control/planners/rrt/RRT.h>

namespace planning {

ProximityPlanner::ProximityPlanner(const Bounds &bounds,
                                   const BodyParams &bodyParams)
    : bounds_(bounds), space_(std::make_shared<RevoySpace>()),
      cspace_(
          std::make_shared<ompl::control::DiscreteControlSpace>(space_, 0, 1)),
      setup_(cspace_), validityChecker_(std::make_shared<ValidityChecker>(
                           setup_.getSpaceInformation(), bodyParams)),
      propagator_(std::make_shared<Propagator>(setup_.getSpaceInformation())) {

  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  // set bounds for speed, m/s
  static constexpr uint8_t CONTROL_MIN_SPEED = 0;
  static constexpr uint8_t CONTROL_MAX_SPEED = 1;
  cspace_->setBounds(CONTROL_MIN_SPEED, CONTROL_MAX_SPEED);

  // set the state propagation routine
  setup_.setStatePropagator(propagator_);

  // set state validity checking for this space
  setup_.setStateValidityChecker(validityChecker_);

  // register default projection
  static constexpr double PROJECTION_CELL_SIZE = 0.1;
  setup_.getStateSpace()->registerProjections();
  setup_.getStateSpace()->getDefaultProjection()->setCellSizes(
      {PROJECTION_CELL_SIZE, PROJECTION_CELL_SIZE});

  // set the planner
  setup_.setPlanner(
      std::make_shared<ompl::control::RRT>(setup_.getSpaceInformation()));

  // control steps per propogation
  static constexpr uint8_t CONTROL_MIN_DURATION = 1;
  static constexpr uint8_t CONTROL_MAX_DURATION = 5;
  setup_.getSpaceInformation()->setMinMaxControlDuration(CONTROL_MIN_DURATION,
                                                         CONTROL_MAX_DURATION);

  // set the bounds for the R^2 part of SE(2)
  ompl::base::RealVectorBounds rbounds(2);
  rbounds.low[0] = bounds_.lowerX;
  rbounds.low[1] = bounds_.lowerY;
  rbounds.high[0] = bounds_.upperX;
  rbounds.high[1] = bounds_.upperY;
  space_->setBounds(rbounds);
};

void ProximityPlanner::plan(const HookedPose &start_, const HookedPose &_,
                            std::shared_ptr<OccupancyGrid> grid) {

  setup_.clear();

  // create a start state
  ompl::base::ScopedState<RevoySpace> start(space_);
  start->setX(start_.position.x());
  start->setY(start_.position.y());
  start->setYaw(start_.yaw);
  start->setTrailerYaw(start_.trailerYaw);

  // create goal state, move forward a little only
  ompl::base::ScopedState<RevoySpace> goal(space_);
  goal->setX(start->getX() + cos(start->getYaw()));
  goal->setY(start->getY() + sin(start->getYaw()));
  goal->setYaw(start->getYaw());
  goal->setYaw(start->getTrailerYaw());

  // update start and goal
  setup_.setStartAndGoalStates(start, goal, 1);

  // caching this for debugging and visualization
  grid_ = grid;

  // updating the collision checker with the latest occupancy grid
  const Pose &gridPose{{start->getX(), start->getY()}, start->getYaw()};
  validityChecker_->setOccupancyGrid(grid_, gridPose);

  // ompl setup
  setup_.setup();

  // run ompl
  static constexpr double PLANNER_SOLVE_DURATION = 0.1;
  ompl::base::PlannerStatus solved = setup_.solve(PLANNER_SOLVE_DURATION);

  // check if solution was found
  if (solved != ompl::base::PlannerStatus::EXACT_SOLUTION &&
      solved != ompl::base::PlannerStatus::APPROXIMATE_SOLUTION) {
    std::cout << "not exact solution: " << solved << std::endl;
  }
}

// simple getters
const ompl::control::SimpleSetup &ProximityPlanner::getSetup() const {
  return setup_;
}
const std::shared_ptr<OccupancyGrid> &
ProximityPlanner::getLastOccupancyGrid() const {
  return grid_;
}

// get the controls from the solution of the last plan.
// capture results for output to downstream controls system.
// we will use the very first control action, since we will re-plan
// and get new controls before this duration runs out.
const Controls ProximityPlanner::getControls() const {

  std::cout << "get controls" << std::endl;
  Controls out;

  // no solution was found, output default controls, all 0s
  if (!setup_.haveSolutionPath()) {
    return out;
  }

  std::cout << "has solution" << std::endl;

  // since a solution was found, update controls
  auto &solution = setup_.getSolutionPath();

  // ensure solution has elements in it, before indexing into 0th element
  if (solution.getStateCount() > 0) {

    const auto ctrl =
        solution.getControl(0)
            ->as<ompl::control::DiscreteControlSpace::ControlType>();
    out.speed = ctrl->value;
    out.steer = 0;
    out.duration = solution.getControlDuration(0);
  }

  std::cout << std::format("solution {} {}", out.speed, out.steer) << std::endl;
  return out;
}

// Validity Checker: collision detection, bounds checking

ProximityPlanner::ValidityChecker::ValidityChecker(
    const ompl::control::SpaceInformationPtr &si, const BodyParams &bodyParams)
    : ompl::base::StateValidityChecker(si), bodyParams_(bodyParams) {}

void ProximityPlanner::ValidityChecker::setOccupancyGrid(
    const std::shared_ptr<OccupancyGrid> grid, const Pose &pose) {
  grid_ = grid;
  currentPose_ = pose;
}

bool ProximityPlanner::ValidityChecker::isValid(
    const ompl::base::State *state_) const {
  if (!grid_) {
    std::cout << "ERROR: no occupancy grid set, no validity check" << std::endl;
    assert(false);
    return false;
  }
  const auto *state = state_->as<RevoySpace::StateType>();
  bool isValid = si_->satisfiesBounds(state);

  const HookedPose pose = {
      {state->getX(), state->getY()}, state->getYaw(), state->getTrailerYaw()};
  const Footprints body = FootprintsFromPose(pose, bodyParams_);

  /// put the footprint into the occupancy frame to check if its hitting
  /// anything
  for (const Footprint &part : body) {
    const Footprint partInRevoyFrame =
        ReverseTransformFootprint(part, currentPose_);
    isValid &= !grid_->isFootprintOccupied(partInRevoyFrame);
  }

  isValid &= fabs(state->getHitchAngle()) < (M_PI / 2.0);
  return isValid;
}

// Propagator: vehicle kinematic propagation using state space

ProximityPlanner::Propagator::Propagator(
    const std::shared_ptr<ompl::control::SpaceInformation> si)
    : ompl::control::StatePropagator(si) {}

void ProximityPlanner::Propagator::propagate(
    const ompl::base::State *start, const ompl::control::Control *control,
    const double duration, ompl::base::State *result) const {

  // Use the sampled controls to create a new node in the search graph
  const auto ctrl =
      control->as<ompl::control::DiscreteControlSpace::ControlType>();
  const double speed = ctrl->value;
  RevoySpace::Propagate(start->as<RevoySpace::StateType>(), {speed, 0},
                        duration, result->as<RevoySpace::StateType>());
};

} // namespace planning
