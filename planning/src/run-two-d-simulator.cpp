#include "planning/include/types.h"
#include "planning/mcap-utils.h"
#include "planning/simpl.h"

#include <cstdint>
#include <iostream>

// todo use units.h
static const double R_BOUNDS = 10;

Entities MakeHallway() {

  Entities entities;

  static const double HALLWAY_X0 = -R_BOUNDS + R_BOUNDS;
  static const double HALLWAY_X1 = HALLWAY_X0 + 2.5;
  static const double HALLWAY_Y0 = -R_BOUNDS + 2.5;
  static const double HALLWAY_Y1 = R_BOUNDS - 2.5;

  Entity entity0 = {{{0, 0}, 0},
                    {
                        {-R_BOUNDS, HALLWAY_Y0},
                        {HALLWAY_X0, HALLWAY_Y0},
                        {HALLWAY_X0, HALLWAY_Y1},
                        {-R_BOUNDS, HALLWAY_Y1},
                        {-R_BOUNDS, HALLWAY_Y0},
                    }};
  entities.push_back(entity0);
  Entity entity1 = {{{0, 0}, 0},
                    {
                        {HALLWAY_X1, HALLWAY_Y0},
                        {R_BOUNDS, HALLWAY_Y0},
                        {R_BOUNDS, HALLWAY_Y1},
                        {HALLWAY_X1, HALLWAY_Y1},
                        {HALLWAY_X1, HALLWAY_Y0},
                    }};

  entities.push_back(entity1);
  return entities;
};

int main() {
  Scenario scenario;
  scenario.bounds = {R_BOUNDS, R_BOUNDS, -R_BOUNDS, -R_BOUNDS};
  scenario.start = {{-9, -9}, 0, 0};
  scenario.goal = {{9, 9}, -M_PI, -M_PI};
  scenario.timeParams = {DT, START_TIME, TIMEOUT};
  scenario.entities = MakeHallway();
  Simpl simpl(scenario);

  ompl::msg::setLogLevel(ompl::msg::LogLevel::LOG_WARN);

  // TODO use units.h library
  int64_t time = START_TIME;
  McapWrapper wrapper("run-two-d-sim.mcap");
  Scene scene;
  scene.scenario = scenario;
  while (!simpl.isDone() &&
         time <= scenario.timeParams.TIMEOUT + scenario.timeParams.START_TIME) {
    simpl.update(time);
    scene.plannedPath = simpl.getProximityPlanner().getLastSolution();
    scene.body = simpl.getBody();
    // for (const auto &p : scene.plannedPath) {
    //   std::cout << "(" << p.x() << "," << p.y() << ") -> ";
    // }
    // std::cout << std::endl;
    wrapper.write(scene, time);
    time += scenario.timeParams.DT;
  }

  std::cout << "end: " << std::to_string(time - START_TIME) << std::endl;
  return 0;
}
