#pragma once

#include <ompl/control/SimpleSetup.h>
#include <ompl/geometric/SimpleSetup.h>

#include "planning/types.h"

namespace planning {

/// Captures all the nodes and edges in the Search Graph.
/// Needed to debug plot graph nodes and edges in foxglove
template <typename SimpleSetup, typename StateType>
void FillGraph(Graph &graph, const SimpleSetup &setup) {
  ompl::base::PlannerData pd(setup.getSpaceInformation());
  setup.getPlannerData(pd);

  // graph.edges.clear();
  // graph.nodes.clear();
  const size_t prevSize = graph.nodes.size();
  graph.nodes.reserve(prevSize + pd.numVertices());

  /// capture nodes
  for (unsigned int i = 0; i < pd.numVertices(); ++i) {
    const auto node = pd.getVertex(i).getState()->as<StateType>();
    const Point p{node->getX(), node->getY()};
    graph.nodes.push_back(p);
  }

  /// capture edges
  std::vector<unsigned int> edgeList;
  for (unsigned int i = 0; i < pd.numVertices(); ++i) {
    pd.getEdges(i, edgeList);
    for (unsigned int target : edgeList) {
      graph.edges.push_back({i + prevSize, target + prevSize});
    }
  }
};

} // namespace planning
