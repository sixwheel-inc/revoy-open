#include <catch2/catch_test_macros.hpp>

#include <cstdint>
#include <memory>

#include "planning/footprint-overlap.h"
#include "planning/make-scenario.h"
#include "planning/mock-revoy-ev.h"
#include "planning/simpl-mcap.h"
#include "planning/simpl.h"
#include "planning/types.h"

#include "planning/add-footprint-to-grid.h"

using namespace planning;

TEST_CASE("test obstacle scenario parameterized direction and distance") {

  const double MIN_AVOIDABLE_DIST = (3 * sqrt(BodyParams().revoyLength));
  const double EAST = 0;
  const double SOUTH_WEST = -3 * M_PI / 4.0;

  static const std::vector<double> DIRS{
      EAST,
      SOUTH_WEST,
  };
  static const std::vector<double> DISTS{
      MIN_AVOIDABLE_DIST - 1,
      MIN_AVOIDABLE_DIST + 1,
  };

  for (const double dir : DIRS) {
    for (const double dist : DISTS) {

      Scenario scenario = MakeDisappearingObstacleScenario(dir, dist);

      std::unique_ptr<Simpl> simpl = std::make_unique<Simpl>(scenario);

      std::unique_ptr<SimplMcap> mcap =
          std::make_unique<SimplMcap>("test-simpl-" + scenario.name + ".mcap");

      int64_t time = scenario.timeParams.startTime;
      bool collision = false;
      double actualSpeed = 0;
      double actualSteer = 0;
      double maxActualSpeed = 0;

      while (time <=
                 scenario.timeParams.timeout + scenario.timeParams.startTime &&
             !simpl->isDone()) {

        // sim + plan
        simpl->update(time, actualSpeed, actualSteer);

        // record mcap
        mcap->write(*simpl, time);

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

        maxActualSpeed = std::fmax(maxActualSpeed, actualSpeed);

        // tick
        time += scenario.timeParams.dt;
      }

      if (dist > MIN_AVOIDABLE_DIST) {
        CHECK(!collision);
        CHECK(maxActualSpeed > 0);
      } else {
        CHECK(collision);

        // NOTE: purposefully using == to compare double: the value should be
        // set to exactly 0
        CHECK(maxActualSpeed == 0);
      }

      simpl.reset();
      mcap.reset();
    }
  }
}
