#include "planning/simpl-to-scene.h"

namespace planning {
Scene SimplToScene(std::unique_ptr<Simpl> &simpl, int64_t time) {
  assert(simpl);

  Scene scene;
  scene.scenario = simpl->getScenario();
  scene.plannedPath = simpl->getProximityPlanner().getLastSolution();
  scene.graph = simpl->getProximityPlanner().getLastGraph();
  scene.revoy = simpl->getRevoyEv().getBody(scene.scenario.bodyParams);
  scene.revoyPose = simpl->getRevoyEv().getHookedPose();
  scene.visibleEntities = simpl->getVisibleFootprints(time);
  scene.grid = simpl->getProximityPlanner().getLastOccupancyGrid();

  return scene;
}
} // namespace planning
