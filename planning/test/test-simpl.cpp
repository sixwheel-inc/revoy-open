#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>

#include "planning/footprint-overlap.h"
#include "planning/make-scenario.h"
#include "planning/mcap-utils.h"
#include "planning/mock-revoy-ev.h"
#include "planning/occupancy-grid.h"
#include "planning/simpl-to-scene.h"
#include "planning/simpl.h"
#include "planning/types.h"

#include "planning/add-footprint-to-grid.h"

using namespace planning;

TEST_CASE("test occupancy") {

  constexpr uint8_t N = 255;
  constexpr float D = 0.1;
  constexpr float O = ((float)N) / 2.0 * D;

  OccupancyGrid grid(N, N, D, D, O, O);

  {
    // not overlapping
    Footprint a{{-10, -10}, {-5, -5}, {-10, -5}};
    Footprint b{{10, 10}, {5, 5}, {10, 5}};
    AddFootprintToGrid(a, grid);
    CHECK(!grid.isFootprintOccupied(b));
    grid.reset();
  }
  {
    // overlapping
    Footprint a{{9, 9}, {4, 4}, {9, 4}};
    Footprint b{{10, 10}, {5, 5}, {10, 5}};
    AddFootprintToGrid(a, grid);
    CHECK(grid.isFootprintOccupied(b));
    grid.reset();
  }
  {
    // one inside the other
    Footprint a{{10, 10}, {5, 5}, {10, 5}};
    Footprint b{{8, 8}, {7, 7}, {8, 7}};
    AddFootprintToGrid(a, grid);
    CHECK(grid.isFootprintOccupied(b));
    grid.reset();
  }
  {
    // test that reset worked
    Footprint b{{10, 10}, {5, 5}, {10, 5}};
    CHECK(!grid.isFootprintOccupied(b));
    grid.reset();
  }
}

TEST_CASE("test collision detection") {
  {
    // not overlapping
    Footprint a{{-10, -10}, {-5, -5}, {-10, -5}};
    Footprint b{{10, 10}, {5, 5}, {10, 5}};
    CHECK(!AreFootprintsOverlapping(a, b));
  }
  {
    // overlapping
    Footprint a{{9, 9}, {4, 4}, {9, 4}};
    Footprint b{{10, 10}, {5, 5}, {10, 5}};
    CHECK(AreFootprintsOverlapping(a, b));
  }
  {
    // one inside the other
    Footprint a{{8, 8}, {7, 7}, {8, 7}};
    Footprint b{{10, 10}, {5, 5}, {10, 5}};
    CHECK(AreFootprintsOverlapping(a, b));
  }
}

TEST_CASE("test point in footprint") {
  {
    // not inside
    Point a{-10, -10};
    Footprint b{{10, 10}, {5, 5}, {10, 5}};
    CHECK(!IsPointInFootprint(a, b));
  }
  {
    // inside
    Point a{9, 6};
    Footprint b{{10, 10}, {5, 5}, {10, 5}};
    CHECK(IsPointInFootprint(a, b));
  }
  {
    // on
    Point a{10, 7};
    Footprint b{{10, 10}, {5, 5}, {10, 5}};
    CHECK(IsPointInFootprint(a, b));
  }
}

TEST_CASE("test obstacle scenario parameterized direction and distance") {

  const double MIN_AVOIDABLE_DIST = (3 * sqrt(BodyParams().revoyLength));
  const double EAST = 0;
  const double SOUTH_WEST = -3 * M_PI / 4.0;

  static const std::vector<double> DIRS{EAST, SOUTH_WEST};
  static const std::vector<double> DISTS{MIN_AVOIDABLE_DIST - 1,
                                         MIN_AVOIDABLE_DIST + 1};

  for (const double dir : DIRS) {
    for (const double dist : DISTS) {

      Scenario scenario = MakeDisappearingObstacleScenario(dir, dist);

      std::unique_ptr<Simpl> simpl = std::make_unique<Simpl>(scenario);

      std::unique_ptr<McapWrapper> mcapWrapper =
          std::make_unique<McapWrapper>("catch2-" + scenario.name + ".mcap");

      int64_t time = scenario.timeParams.startTime;
      bool collision = false;
      double actualSpeed = 0;
      double actualSteer = 0;

      while (time <=
                 scenario.timeParams.timeout + scenario.timeParams.startTime &&
             !simpl->isDone()) {

        // sim + plan
        simpl->update(time, actualSpeed, actualSteer);

        // record mcap
        const Scene scene = SimplToScene(simpl, time);
        mcapWrapper->write(scene, time);

        // Use full polygon intersection, from the occupancy grid used in
        // planning. this check fails correctly for edge cases not caught by
        // occupancy grid.
        const Footprints revoy =
            simpl->getRevoyEv().getBody(scenario.bodyParams);
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

      if (dist > MIN_AVOIDABLE_DIST) {
        CHECK(!collision);
      } else {
        CHECK(collision);
      }

      simpl.reset();
      mcapWrapper.reset();
    }
  }
}
