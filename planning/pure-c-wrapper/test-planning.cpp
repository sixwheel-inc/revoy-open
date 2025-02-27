#include <catch2/catch_test_macros.hpp>

#include <cstddef>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>

#include "planning/planning.h"

// 100,000 us == 100ms == 0.1s
static const double DT = 1e5;

// using 0 sometimes causes problems
// 1.7e15 is GMT Tuesday, November 14, 2023 10:13:20 PM
static const double START_TIME = 1.7e15;

// 1,000,000 us == 1,000 ms == 1s
static const double TIMEOUT = 1e6;

TEST_CASE("test planning interface") {

  StartPlanning();

  Perception perception;
  perception.numBytes = 20000;
  perception.numBytesInRow = 50;
  perception.bitsPerMeter = 8;

  /// this offset will put the center of the actual grid at the ego center
  /// if the grid is to be "in front" of the vehicle, correspondingly increase
  /// offsetX.
  perception.offsetX = 10;
  perception.offsetY = 25;

  perception.time = START_TIME;
  bool didGo = false;

  /// all occupied, plan should be 0 speed
  memset(perception.occupancy, 0xFFFF, MAX_OCCUPANCY_NUM_BYTES);

  /// Make sure we do not plan to move ahead when there is an obstacle
  while (perception.time <= START_TIME + TIMEOUT) {
    Plan plan = GetNextPlan(&perception);
    didGo |= (plan.setSpeed > 0);
    perception.time += DT;
  }
  CHECK(!didGo);

  /// Make sure we plan to move ahead when there is no obstacle
  /// no occupied, plan should be > 0 speed
  memset(perception.occupancy, 0, MAX_OCCUPANCY_NUM_BYTES);

  /// occupy furthest away row, won't stop us
  for (uint32_t i = 0; i < perception.numBytesInRow; i++) {
    perception.occupancy[i] = 0xFF;
  }

  /// occupy leftmost column, won't stop us
  for (uint32_t i = 0; i < perception.numBytes; i++) {
    if (i % perception.numBytesInRow == 0) {
      perception.occupancy[i] = 0x1;
    }
  }

  double newStart = perception.time;
  didGo = false;
  while (perception.time <= newStart + TIMEOUT) {
    Plan plan = GetNextPlan(&perception);
    didGo |= (plan.setSpeed > 0);
    perception.time += DT;
  }
  CHECK(didGo);

  EndPlanning();
}

// void PrintPerception(const Perception *perception) {}
