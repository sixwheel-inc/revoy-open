#pragma once

#include <ompl/control/SimpleSetup.h>
#include <ompl/geometric/SimpleSetup.h>

#include "planning/types.h"

namespace planning {

/// Captures the interpolated path between states, in order to show
/// curvature.
template <typename SimpleSetup, typename StateType>
void FillPath(Path &path, const SimpleSetup &setup) {

  if (!setup.haveSolutionPath()) {
    return;
  }
  auto &solution = setup.getSolutionPath();
  for (const auto baseState : solution.getStates()) {
    const auto state = baseState->template as<StateType>();
    path.push_back({state->getX(), state->getY()});
  }
};

} // namespace planning
