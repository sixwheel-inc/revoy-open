#pragma once

#include "planning/occupancy-grid.h"
#include "planning/revoy-space.h"

#include "planning/types.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>

namespace planning {

/// Plans to move forward slowly unless an obstacle is in the way.
///
class ProximityPlanner {
public:
  ProximityPlanner() = delete;
  ProximityPlanner(const Bounds &bounds, const BodyParams &bodyParams);

  /// Completes ompl setup, calls ompl solve
  void plan(const HookedPose &start, const HookedPose &goal,
            std::shared_ptr<OccupancyGrid> grid);

  /// getters used for output / debug
  const Path &getLastSolution() const;
  const Graph &getLastGraph() const;
  const Controls &getControls() const;
  const std::shared_ptr<OccupancyGrid> &getLastOccupancyGrid() const;

  /// ompl hooks that we define, pass data / query the StateSpace
  class ValidityChecker;
  class Propagator;

private:
  /// Params, Inputs, Outputs
  Bounds bounds_ = {};
  Path path_ = {};
  Graph graph_ = {};
  Controls controls_ = {};

  /// OMPL stuff
  std::shared_ptr<RevoySpace> space_;
  std::shared_ptr<ompl::control::DiscreteControlSpace> cspace_;
  ompl::control::SimpleSetup setup_;
  std::shared_ptr<ValidityChecker> validityChecker_;
  std::shared_ptr<Propagator> propagator_;

  /// Obstacles
  std::shared_ptr<OccupancyGrid> grid_;

public:
  /// OMPL will use this to decide if a State in the StateSpace is
  /// valid, i.e. outside all obstacles.
  class ValidityChecker : public ompl::base::StateValidityChecker {
  public:
    ValidityChecker(const ompl::control::SpaceInformationPtr &si,
                    const BodyParams &bodyParams);
    /// used by OMPL during search
    bool isValid(const ompl::base::State *state_) const override;

    /// used by us to pass in latest obstacle positions
    void setOccupancyGrid(const std::shared_ptr<OccupancyGrid> grid,
                          const Pose &revoyPose);

    // void setFootprints(const Footprints &obstacles);

  private:
    BodyParams bodyParams_ = {};
    std::shared_ptr<OccupancyGrid> grid_;
    Pose currentPose_ = {};
  };

  /// Used by Control-based Planners, produce a new search node given a possible
  /// action the Robot could take
  class Propagator : public ompl::control::StatePropagator {
  public:
    Propagator(const std::shared_ptr<ompl::control::SpaceInformation> si,
               const BodyParams &bodyParams);

    /// called by OMPL during search
    virtual void propagate(const ompl::base::State *start,
                           const ompl::control::Control *control,
                           const double duration,
                           ompl::base::State *result) const override;

  private:
    BodyParams bodyParams_;
  };
};

} // namespace planning
