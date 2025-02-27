#include "planning/simpl-to-scene.h"

namespace planning {
Scene SimplToScene(std::unique_ptr<Simpl> &simpl, int64_t time) {
  assert(simpl);

  Scene scene;
  scene.scenario = simpl->getScenario();
  scene.revoy = simpl->getRevoyEv().getBody(scene.scenario.bodyParams);
  scene.revoyPose = simpl->getRevoyEv().getHookedPose();
  scene.visibleEntities = simpl->getVisibleFootprints(time);
  scene.grid = simpl->getProximityPlanner().getLastOccupancyGrid();

  // TODO for now this is hardcoded but we can make this configurable
  scene.planners["proximity"].solution =
      simpl->getProximityPlanner().getLastSolution();
  scene.planners["proximity"].graph =
      simpl->getProximityPlanner().getLastGraph();

  return scene;
}
} // namespace planning
