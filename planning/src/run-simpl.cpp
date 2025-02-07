
#include "planning/make-scenario.h"

#include "planning/footprint-overlap.h"
#include "planning/mcap-utils.h"
#include "planning/simpl-to-scene.h"
#include "planning/simpl.h"

using namespace planning;

int main(int argc, char **argv) {

  const std::vector<std::string> args(argv + 1, argv + argc);

  const Scenario scenario = MakeYardScenario();

  std::unique_ptr<Simpl> simpl = std::make_unique<Simpl>(scenario);

  std::unique_ptr<McapWrapper> mcapWrapper =
      std::make_unique<McapWrapper>("catch2-" + scenario.name + ".mcap");

  int64_t time = scenario.timeParams.startTime;
  bool collision = false;
  double actualSpeed = 0;
  double actualSteer = 0;

  while (time <= scenario.timeParams.timeout + scenario.timeParams.startTime &&
         !simpl->isDone()) {

    // sim + plan
    simpl->update(time, actualSpeed, actualSteer);

    // record mcap
    const Scene scene = SimplToScene(simpl, time);
    mcapWrapper->write(scene, time);

    // Use full polygon intersection, from the occupancy grid used in
    // planning. this check fails correctly for edge cases not caught by
    // occupancy grid.
    const Footprints revoy = simpl->getRevoyEv().getBody(scenario.bodyParams);
    const Footprints obsts = simpl->getVisibleFootprints(time);
    collision |= IsBodyCollidingAnyObstacles(revoy, obsts);

    // for this test, instantly apply speed / steer to actual speed /
    // steer
    const Controls controls = simpl->getProximityPlanner().getControls();

    actualSpeed = controls.speed;
    actualSteer = controls.steer;

    // tick
    time += scenario.timeParams.dt;
  }

  simpl.reset();
  mcapWrapper.reset();

  std::cout << "collision: " << (collision ? "yes" : "no") << std::endl;
  return 0;
}
