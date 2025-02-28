#include "planning/planning.h"

#include "planning/occupancy-grid.h"
#include "planning/proximity-planner.h"
#include "planning/types.h"

#include <bitset>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <ctime>
#include <iostream>
#include <memory>
#include <string>

using namespace planning;

static std::unique_ptr<ProximityPlanner> planner = nullptr;

/// replace shared w/ unique
static std::shared_ptr<OccupancyGrid> grid = nullptr;

static const Bounds BOUNDS;
static const BodyParams BODY;

Perception MakeEmptyPerception() { return {}; }

void StartPlanning() {
  std::cout << "Initalize Planning" << std::endl;
  planner = std::make_unique<ProximityPlanner>(BOUNDS, BODY);
  grid = std::make_shared<OccupancyGrid>();
};

void EndPlanning() {
  std::cout << "Terminate Planning" << std::endl;
  planner.reset();
};

/// for now, these dont matter because there is no localization, and we assume
/// to move forward a little
static const HookedPose START{{0, 0}, 0, 0};
static const HookedPose GOAL{{1, 0}, 0, 0};

Plan GetNextPlan(const Perception *perception) {

  if (!perception) {
    return {};
  }

  // std::cout << "perception->numBytes: " <<
  // std::to_string(perception->numBytes)
  //           << std::endl;
  // std::cout << "perception->numBytesInRow: "
  //           << std::to_string(perception->numBytesInRow) << std::endl;
  // std::cout << "perception->bitsPerMeter: "
  //           << std::to_string(perception->bitsPerMeter) << std::endl;
  // std::cout << "perception->offsetX: " << std::to_string(perception->offsetX)
  //           << std::endl;
  // std::cout << "perception->offsetY: " << std::to_string(perception->offsetY)
  //           << std::endl;

  /// prevent divide-by-zero, assert because execution won't be possible
  assert(perception->numBytes != 0);
  assert(perception->numBytesInRow != 0);
  assert(perception->bitsPerMeter != 0);

  const uint32_t N = perception->numBytes / perception->numBytesInRow;
  const uint32_t M = perception->numBytesInRow * 8;

  const double cellX = 1.0 / perception->bitsPerMeter;
  const double cellY = 1.0 / perception->bitsPerMeter;
  const double offsetX = perception->offsetX;
  const double offsetY = perception->offsetY;

  // std::cout << "N: " << std::to_string(N) << std::endl;
  // std::cout << "M: " << std::to_string(M) << std::endl;
  // std::cout << "cellX: " << std::to_string(cellX) << std::endl;
  // std::cout << "cellY: " << std::to_string(cellY) << std::endl;
  // std::cout << "offsetX: " << std::to_string(offsetX) << std::endl;
  // std::cout << "offsetY: " << std::to_string(offsetY) << std::endl;

  grid->reset(perception->occupancy, perception->numBytes, N, M, cellX, cellY,
              offsetX, offsetY);
  planner->plan(START, GOAL, grid);

  const Controls controls = planner->getControls();
  const auto &grid = planner->getLastOccupancyGrid();

  Plan plan = {.setSpeed = controls.speed,
               .setSteer = controls.steer,
               .gridWidth = grid->getN(),
               .gridHeight = grid->getM(),
               .cellSizeX = grid->getCellX(),
               .cellSizeY = grid->getCellY(),
               .originX = grid->getOffsetX(),
               .originY = grid->getOffsetY(),
               .gridData = nullptr,
               .gridDataSize = 0};

  // Copy grid data
  std::string gridData = grid->fillData();
  if (!gridData.empty()) {
    plan.gridData = new char[gridData.size()];
    memcpy(plan.gridData, gridData.data(), gridData.size());
    plan.gridDataSize = gridData.size();
  }

  return plan;
}
