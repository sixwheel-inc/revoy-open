#include "planning/occupancy-grid.h"

#include <bitset>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <limits>

namespace planning {

OccupancyGrid::OccupancyGrid(uint16_t N, uint16_t M, double cellX, double cellY,
                             double offsetX, double offsetY)
    : N_(N), M_(M), cellX_(cellX), cellY_(cellY), offsetX_(offsetX),
      offsetY_(offsetY) {
  reset();
};

uint16_t OccupancyGrid::getN() const { return N_; }
uint16_t OccupancyGrid::getM() const { return M_; }
double OccupancyGrid::getCellX() const { return cellX_; }
double OccupancyGrid::getCellY() const { return cellY_; }
double OccupancyGrid::getOffsetX() const { return offsetX_; }
double OccupancyGrid::getOffsetY() const { return offsetY_; }

/// reuse the grid, de-occupy all cells
void OccupancyGrid::reset() {
  std::fill(occupancy_.begin(), occupancy_.end(), 0);
}

void OccupancyGrid::reset(const char *array, size_t numChars, uint16_t N,
                          uint16_t M, double cellX, double cellY,
                          double offsetX, double offsetY) {

  assert(numChars < MAX_NUM_BYTES_OCCUPANCY);

  std::memcpy(occupancy_.data(), array, numChars);
  N_ = N;
  M_ = M;
  cellX_ = cellX;
  cellY_ = cellY;
  offsetX_ = offsetX;
  offsetY_ = offsetY;
}

bool OccupancyGrid::isFootprintOccupied(const Footprint &footprint) const {
  for (size_t index = 0; index < footprint.size() - 1; index++) {

    /// TODO: possible false negative, when occupied cells lie perfectly inside
    /// the footprint, without touching the footprint.
    /// Fix by doing something else.

    const std::vector<Idx> boundaryCells =
        getListOfCellsAlongSegment(footprint[index], footprint[index + 1]);

    for (const Idx &idx : boundaryCells) {
      if (isIdxInGrid(idx) && isOccupied(idx)) {
        return true;
      }
    }
  }

  return false;
};

bool OccupancyGrid::areFootprintsOccupied(const Footprints &footprints) const {
  bool occupied = false;
  for (const Footprint &footprint : footprints) {
    occupied |= isFootprintOccupied(footprint);
  }
  return occupied;
};

std::string OccupancyGrid::fillData() const {
  std::string res;

  /// string concat is fast enough in C++ because std:string is mutable
  /// slow, only used for converting to mcap
  for (uint16_t i = N_ - 1; i < N_; i--) {
    for (uint16_t j = 0; j < M_; j++) {
      const Idx idx{i, j};
      res += isOccupied(idx) ? 0xFF : '\0';
    }
  }
  return res;
}

void OccupancyGrid::occupyCells(const std::vector<Idx> &idxs) {
  for (const Idx idx : idxs) {
    occupy(idx);
  }
}

OccupancyGrid::Idx OccupancyGrid::pointToIdx(const Point &point) const {

  double i_ = std::floor(-(point.x() - offsetX_)) / cellX_;
  double j_ = std::floor(-(point.y() - offsetY_)) / cellY_;
  const uint16_t i = (i_ >= 0) ? i_ : std::numeric_limits<uint16_t>::max();
  const uint16_t j = (j_ >= 0) ? j_ : std::numeric_limits<uint16_t>::max();
  return {i, j};
}

Point OccupancyGrid::idxToPoint(const Idx &idx) const {

  const double x = -(idx.i * cellX_) + offsetX_;
  const double y = -(idx.j * cellY_) + offsetY_;

  return {x, y};
}

/// use this during planning to validate sampled states against occupancy grid
/// NOTE: assumes this  coords are in Revoy's frame
const std::vector<OccupancyGrid::Idx>
OccupancyGrid::getListOfCellsAlongSegment(const Point &alpha,
                                          const Point &beta) const {

  std::vector<Idx> res;
  const Point delta = beta - alpha;

  /// TODO using the smaller of cellX and cellY to estamate cell size,
  /// considered using sqrt(cellX^2 + cellY^2), but that could potentially
  /// cause us to skip a cell.
  const double dCell = std::min(cellX_, cellY_);
  const uint16_t numSteps = delta.norm() / dCell;

  for (uint16_t stepCount = 0; stepCount < numSteps; stepCount++) {
    const Point step = (delta.normalized() * stepCount * dCell);
    const Point point = alpha + step;
    const Idx idx = pointToIdx(point);

    /// TODO: occasionally an idx will be duplicated in here
    res.push_back(idx);
  }
  return res;
}
OccupancyGrid::BitPos OccupancyGrid::idxToBitPos(const Idx &idx) const {
  const uint32_t bitIndex = (idx.i * M_) + idx.j;
  const uint32_t byte = bitIndex / 8;
  const uint16_t bit = bitIndex % 8;
  return {byte, bit};
}

bool OccupancyGrid::isOccupied(const Idx &idx) const {
  const BitPos bitPos = idxToBitPos(idx);
  const std::bitset<8> bits = occupancy_.at(bitPos.byte);
  return bits.test(bitPos.bit);
};

void OccupancyGrid::occupy(const Idx &idx) {
  const BitPos bitPos = idxToBitPos(idx);
  std::bitset<8> bits = occupancy_.at(bitPos.byte);
  bits.set(bitPos.bit);
  occupancy_.at(bitPos.byte) = static_cast<char>(bits.to_ulong());
}

bool OccupancyGrid::isIdxInGrid(const Idx &idx) const {
  return idx.i < N_ && idx.j < M_;
};

bool OccupancyGrid::isPointOccupied(const Point &point) const {
  const Idx idx = pointToIdx(point);
  return isIdxInGrid(idx) && isOccupied(idx);
}

} // namespace planning
