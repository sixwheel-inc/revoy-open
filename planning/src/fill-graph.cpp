#include "planning/fill-graph.h"
#include "planning/revoy-space.h"

namespace planning {
void FillGraph(Graph &graph, const ompl::control::SimpleSetup &setup) {

  graph.edges.clear();
  graph.nodes.clear();

  ompl::base::PlannerData pd(setup.getSpaceInformation());
  setup.getPlannerData(pd);
  graph.nodes.reserve(pd.numVertices());

  /// capture nodes
  for (unsigned int i = 0; i < pd.numVertices(); ++i) {
    const auto node = pd.getVertex(i).getState()->as<RevoySpace::StateType>();
    const Point p{node->getX(), node->getY()};
    graph.nodes.push_back(p);
  }

  /// capture edges
  std::vector<unsigned int> edgeList;
  for (unsigned int i = 0; i < pd.numVertices(); ++i) {
    pd.getEdges(i, edgeList);
    for (unsigned int target : edgeList) {
      graph.edges.push_back({i, target});
    }
  }
};
} // namespace planning
