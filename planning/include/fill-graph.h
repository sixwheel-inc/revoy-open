#pragma once

#include <ompl/control/SimpleSetup.h>

#include "planning/types.h"

namespace planning {
/// Captures all the nodes and edges in the Search Graph.
/// Needed to debug plot graph nodes and edges in foxglove
void FillGraph(Graph &graph, const ompl::control::SimpleSetup &setup);
} // namespace planning
