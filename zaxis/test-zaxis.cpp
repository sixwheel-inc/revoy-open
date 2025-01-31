#include "zaxis/zaxis.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>

#include <catch2/catch_test_macros.hpp>

using namespace revoy;

TEST_CASE("run scenarios") {
  std::cout << "top test" << std::endl;

  /// TODO: use scenario-list.h, convert to m/s
  const std::vector<Goal> lightSwervingSlow{
      {
          {5, 0.5, 0.5},
          {0, 0.05, 0},
          0,
      },
      {
          {5, 0.5, 0.5},
          {0.2, 0.05, 0},
          0,
      },
      {
          {5, 0.5, 0.5},
          {-0.2, 0.05, 0},
          0,
      },
      {
          {5, 0.5, 0.5},
          {0.2, 0.05, 0},
          0,
      },
      {
          {5, 0.5, 0.5},
          {-0.2, 0.05, 0},
          0,
      },
      {
          {5, 0.5, 0.5},
          {0.2, 0.05, 0},
          0,
      },
      {
          {5, 0.5, 0.5},
          {-0.2, 0.05, 0},
          0,
      },
      {
          {5, 0.5, 0.5},
          {0.2, 0.05, 0},
          0,
      },
      {
          {5, 0.5, 0.5},
          {-0.2, 0.05, 0},
          0,
      },
  };

  const std::vector<Goal> heavySwervingFast{
      {
          {15, 0.5, 0.5},
          {0, 0.05, 0},
          0,
      },
      {
          {15, 0.5, 0.5},
          {0.4, 0.1, 0.1},
          0,
      },
      {
          {15, 0.5, 0.5},
          {-0.4, 0.05, 0.05},
          0,
      },
      {
          {15, 0.5, 0.5},
          {0.4, 0.1, 0.1},
          0,
      },
      {
          {15, 0.5, 0.5},
          {-0.4, 0.05, 0.05},
          0,
      },
      {
          {15, 0.5, 0.5},
          {0.4, 0.1, 0.1},
          0,
      },
      {
          {15, 0.5, 0.5},
          {-0.4, 0.05, 0.05},
          0,
      },
      {
          {15, 0.5, 0.5},
          {0.4, 0.1, 0.1},
          0,
      },
      {
          {15, 0.5, 0.5},
          {-0.4, 0.05, 0.05},
          0,
      },
      {
          {15, 0.5, 0.5},
          {0.4, 0.1, 0.1},
          0,
      },
      {
          {15, 0.5, 0.5},
          {-0.4, 0.05, 0.05},
          0,
      },
  };

  {
    Zaxis::Scenario scenario;
    scenario.name = "light-swerving-slow";
    scenario.goals = lightSwervingSlow;
    std::cout << "init scenario w/ TractorRevoyTrailer" << std::endl;
    Zaxis zaxis(std::move(scenario),
                Zaxis::AssemblyType::TractorRevoyTrailerType);

    std::cout << "run light-swerving, slow, tractor-revoy-trailer" << std::endl;
    const bool didComplete = zaxis.runThroughAllGoals();

    CHECK(didComplete);
  }

  {
    Zaxis::Scenario scenario;
    scenario.goals = heavySwervingFast;
    scenario.name = "heavy-swerving-fast";
    std::cout << "init scenario w/ TractorRevoyTrailer" << std::endl;
    Zaxis zaxis(std::move(scenario),
                Zaxis::AssemblyType::TractorRevoyTrailerType);

    std::cout
        << "run heavy-swerving, fast, tractor-revoy-trailer (expect rollover)"
        << std::endl;
    const bool didComplete = zaxis.runThroughAllGoals();

    CHECK(!didComplete);
  }
}
