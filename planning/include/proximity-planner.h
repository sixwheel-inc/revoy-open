#pragma once

#include "planning/occupancy-grid.h"
#include "planning/revoy-space.h"

#include "planning/types.h"

#include <ompl/base/SpaceInformation.h>
#include <ompl/control/SimpleSetup.h>
#include <ompl/control/StatePropagator.h>
#include <ompl/control/spaces/DiscreteControlSpace.h>

namespace planning {

class ProximityPlanner {
public:
  ProximityPlanner() = delete;
  ProximityPlanner(const Bounds &bounds, const BodyParams &bodyParams);
  void plan(const HookedPose &start, const HookedPose &goal,
            std::shared_ptr<OccupancyGrid> grid);
  const Path &getLastSolution() const;
  const Graph &getLastGraph() const;
  const Controls &getControls() const;
  const std::shared_ptr<OccupancyGrid> &getLastOccupancyGrid() const;

  class ValidityChecker;
  class Propagator;

private:
  Bounds bounds_ = {};
  Path path_ = {};
  Graph graph_ = {};
  Controls controls_ = {};
  std::shared_ptr<RevoySpace> space_;
  std::shared_ptr<ompl::control::DiscreteControlSpace> cspace_;
  ompl::control::SimpleSetup setup_;
  std::shared_ptr<ValidityChecker> validityChecker_;
  std::shared_ptr<Propagator> propagator_;
  std::shared_ptr<OccupancyGrid> grid_;

public:
  class ValidityChecker : public ompl::base::StateValidityChecker {
  public:
    ValidityChecker(const ompl::control::SpaceInformationPtr &si,
                    const BodyParams &bodyParams);
    bool isValid(const ompl::base::State *state_) const override;
    // void setFootprints(const Footprints &obstacles);
    void setOccupancyGrid(const std::shared_ptr<OccupancyGrid> grid,
                          const Pose &revoyPose);

  private:
    BodyParams bodyParams_ = {};
    std::shared_ptr<OccupancyGrid> grid_;
    Pose currentPose_ = {};
  };

  class Propagator : public ompl::control::StatePropagator {
  public:
    Propagator(const std::shared_ptr<ompl::control::SpaceInformation> si,
               const BodyParams &bodyParams);
    virtual void propagate(const ompl::base::State *start,
                           const ompl::control::Control *control,
                           const double duration,
                           ompl::base::State *result) const override;

  private:
    BodyParams bodyParams_;
  };
};

} // namespace planning
