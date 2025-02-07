#include "planning/proximity-planner.h"

#include "planning/fill-graph.h"
#include "planning/footprint-transform.h"
#include "planning/occupancy-grid.h"
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
      propagator_(std::make_shared<Propagator>(setup_.getSpaceInformation(),
                                               bodyParams)) {

  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  // set bounds for speed and steer
  cspace_->setBounds(0, 1);

  // set the state propagation routine
  setup_.setStatePropagator(propagator_);

  // set state validity checking for this space
  setup_.setStateValidityChecker(validityChecker_);

  // register default projection
  setup_.getStateSpace()->registerProjections();
  setup_.getStateSpace()->getDefaultProjection()->setCellSizes({0.1, 0.1});

  // set the planner
  setup_.setPlanner(
      std::make_shared<ompl::control::RRT>(setup_.getSpaceInformation()));

  // control steps per propogation
  setup_.getSpaceInformation()->setMinMaxControlDuration(1, 5);

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

  setup_.setStartAndGoalStates(start, goal, 1);

  path_.clear();
  controls_ = {};
  grid_ = grid;

  const Pose &gridPose{{start->getX(), start->getY()}, start->getYaw()};
  validityChecker_->setOccupancyGrid(grid_, gridPose);

  setup_.setup();
  // setup_.print();
  ompl::base::PlannerStatus solved = setup_.solve(0.1);

  if (solved != ompl::base::PlannerStatus::EXACT_SOLUTION) {
    std::cout << "not exact solution: " << solved << std::endl;
  } else {
    auto &solution = setup_.getSolutionPath();
    for (const auto baseState : solution.getStates()) {
      const auto state = baseState->as<RevoySpace::StateType>();
      path_.push_back({state->getX(), state->getY()});
    }

    if (solution.getStateCount() > 0) {
      const auto ctrl =
          solution.getControl(0)
              ->as<ompl::control::DiscreteControlSpace::ControlType>();
      controls_.speed = ctrl->value;
      controls_.steer = 0;
      controls_.duration = solution.getControlDuration(0);
    }

    FillGraph(graph_, setup_);
  }
  setup_.clear();
}

const Path &ProximityPlanner::getLastSolution() const { return path_; };
const Controls &ProximityPlanner::getControls() const { return controls_; }
const Graph &ProximityPlanner::getLastGraph() const { return graph_; }
const std::shared_ptr<OccupancyGrid> &
ProximityPlanner::getLastOccupancyGrid() const {
  return grid_;
}

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

ProximityPlanner::Propagator::Propagator(
    const std::shared_ptr<ompl::control::SpaceInformation> si,
    const BodyParams &bodyParams)
    : ompl::control::StatePropagator(si), bodyParams_(bodyParams) {}

void ProximityPlanner::Propagator::propagate(
    const ompl::base::State *start, const ompl::control::Control *control,
    const double duration, ompl::base::State *result) const {

  const auto ctrl =
      control->as<ompl::control::DiscreteControlSpace::ControlType>();
  const double speed = ctrl->value;
  RevoySpace::Propagate(start->as<RevoySpace::StateType>(), {speed, 0},
                        bodyParams_, duration,
                        result->as<RevoySpace::StateType>());
};

} // namespace planning
