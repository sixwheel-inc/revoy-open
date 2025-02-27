
#include "planning/make-scenario.h"

#include "planning/footprint-overlap.h"
#include "planning/simpl-mcap.h"
#include "planning/simpl-to-scene.h"
#include "planning/simpl.h"

using namespace planning;

int main(int argc, char **argv) {

  // TODO: the Yard scenaro is currently just a placeholder
  // replace with argparse logic and choose a scenario
  const std::vector<std::string> args(argv + 1, argv + argc);
  const Scenario scenario = MakeYardScenario();

  // initialize simulator
  std::unique_ptr<Simpl> simpl = std::make_unique<Simpl>(scenario);

  // initialize debug visualizer
  std::unique_ptr<SimplMcap> mcap =
      std::make_unique<SimplMcap>("run-simpl-" + scenario.name + ".mcap");

  // setup simulation boundary / exit conditions
  int64_t time = scenario.timeParams.startTime;
  bool collision = false;

  // setup simulation initial condition
  double actualSpeed = 0;
  double actualSteer = 0;

  // loop until scenario is done or timeout
  while (time <= scenario.timeParams.timeout + scenario.timeParams.startTime &&
         !simpl->isDone()) {

    // sim + plan
    simpl->update(time, actualSpeed, actualSteer);

    // record mcap
    const Scene scene = SimplToScene(simpl, time);
    mcap->write(scene, time);

    // Use full polygon intersection, from the occupancy grid used in
    // planning. this check fails correctly for edge cases not caught by
    // occupancy grid.
    const Footprints revoy = simpl->getRevoyEv().getBody(scenario.bodyParams);
    const Footprints obsts = simpl->getVisibleFootprints(time);
    collision |= IsBodyCollidingAnyObstacles(revoy, obsts);

    // for this test, instantly apply speed / steer to actual speed /
    // steer
    const Controls controls = simpl->getProximityPlanner().getControls();

    // will be used to plan the next frame
    actualSpeed = controls.speed;
    actualSteer = controls.steer;

    // tick time
    time += scenario.timeParams.dt;
  }

  // graceful exit
  simpl.reset();
  mcap.reset();

  // error report
  std::cout << "collision: " << (collision ? "yes" : "no") << std::endl;
  return collision ? 0 : 1;
}
