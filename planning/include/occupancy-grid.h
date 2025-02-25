#pragma once

#include "planning/types.h"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace planning {

/// Represents the area that is occupied (undrivable).
/// convention used:
/// N, M:             uint8, represents the size of the grid
/// cellX, cellY:     double, size of each cell in each dimension
/// offsetX, offsetY: double, xy position of cell (0, 0)
/// x, y:             double, geometric coordinates, assumed to be in ego frame
///                      -x -> N, -y -> M
///                      x = -(i * cellX) + offsetX
///                      y = -(j * cellX) + offsetY
/// i, j:             int8, indexes used for iteration
///                      in range [0, N), [0, M)
///                      i = (-x - offsetX) / cellX
///                      j = (-y - offsetY) / cellY
class OccupancyGrid {
public:
  OccupancyGrid() = default;

  OccupancyGrid(uint16_t N, uint16_t M, double cellX, double cellY,
                double offsetX, double offsetY);

  /// the viz logic needs to use these to correctly render the occupancy grid
  uint16_t getN() const;
  uint16_t getM() const;
  double getCellX() const;
  double getCellY() const;
  double getOffsetX() const;
  double getOffsetY() const;

  /// reuse the grid, de-occupy all cells
  void reset();

  /// reuse the grid, fill w/ new char array
  void reset(const char *array, size_t numChars, uint16_t N, uint16_t M,
             double cellX, double cellY, double offsetX, double offsetY);

  /// check if the footprint lies over an occupied cell
  /// use this to check if Revoy footprint is in a collision.
  /// NOTE: assumes this footprint coords are in the Occupancy Grid frame
  bool isFootprintOccupied(const Footprint &footprint) const;

  /// same as above but in a loop over a list
  /// NOTE: assumes this footprint coords are in the Occupancy Grid frame
  bool areFootprintsOccupied(const Footprints &footprints) const;

  /// check if point is occupied, true if point falls into occupied cell OR
  /// outside grid bounds.
  bool isPointOccupied(const Point &point) const;

  /// useful for debugging smaller unittests with fewer cells
  void print() const;

  /// this is not a printable / human-readable string, this is used
  /// to set the data buffer in the outgoing visualization messages
  std::string fillData() const;

  /// points to a potential cell in the OccupancyGrid
  struct Idx {
    uint16_t i = 0;
    uint16_t j = 0;
    bool operator==(const Idx &other) const {
      return i == other.i && j == other.j;
    }
    bool operator!=(const Idx &other) const { return !(*this == other); }
  };

  /// returns true if the idx is within OccupancyGrid bounds
  bool isIdxInGrid(const Idx &idx) const;

  /// maps xy to ij
  Idx pointToIdx(const Point &point) const;
  Point idxToPoint(const Idx &idx) const;

  void occupyCells(const std::vector<Idx> &cells);

private:
  uint16_t N_ = 1;
  uint16_t M_ = 1;
  double cellX_ = 1;
  double cellY_ = 1;
  double offsetX_ = 0;
  double offsetY_ = 0;

  /// the actual occupancy grid data,
  static constexpr uint32_t MAX_NUM_BYTES_OCCUPANCY = 50000;
  std::array<char, MAX_NUM_BYTES_OCCUPANCY> occupancy_;

  /// true when cell at idx is occupied by obstacle
  bool isOccupied(const Idx &idx) const;

  /// marks cell at idx as occupied by obstacle
  void occupy(const Idx &idx);

  /// byte + bit will lead to a specific bit in an array
  struct BitPos {
    uint32_t byte;
    uint16_t bit;
  };

  /// used to access the specific bit that represents this idx
  BitPos idxToBitPos(const Idx &idx) const;

  /// choose any corner of the footprint that is within the OccupancyGrid
  /// bounds, we need this to start the floodfill
  const std::vector<Idx> pickSeedsInGrid(const Footprint &footprint) const;

  /// identify all possible neighboring grid cells, up, down, left, right
  static std::vector<Idx> GetAllNeighbors(const Idx &idx);

  /// use this during planning to validate sampled states against occupancy grid
  /// NOTE: assumes this  coords are in Revoy's frame
  const std::vector<Idx> getListOfCellsAlongSegment(const Point &alpha,
                                                    const Point &beta) const;

  /// use this w/ simulator obstacles to get list of cells to mark
  /// occupied.
  /// NOTE: assumes this footprint coords are in Revoy's frame
  const std::vector<Idx>
  getListOfCellsForFootprint(const Footprint &footprint) const;

  /// returns true if the the cell at idx is inside footprint
  bool isIdxInFootprint(const Idx &idx, const Footprint &footprint) const;
};

} // namespace planning
