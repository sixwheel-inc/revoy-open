#include <catch2/catch_test_macros.hpp>

#include "planning/footprint-overlap.h"
#include "planning/occupancy-grid.h"
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
