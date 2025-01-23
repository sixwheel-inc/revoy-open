#include "planning/add-footprint-to-grid.h"

#include "planning/footprint-overlap.h"
#include "planning/occupancy-grid.h"

#include <deque>

namespace planning {

namespace {
using Idx = OccupancyGrid::Idx;

/// use this w/ simulator obstacles to get list of cells to mark
/// occupied.
/// NOTE: assumes this footprint coords are in Revoy's frame
const std::vector<Idx> GetListOfCellsForFootprint(const Footprint &footprint,
                                                  const OccupancyGrid &grid);

/// choose any corner of the footprint that is within the OccupancyGrid
/// bounds, we need this to start the floodfill
const std::vector<Idx> PickSeedsInGrid(const Footprint &footprint,
                                       const OccupancyGrid &grid);

/// identify all possible neighboring grid cells, up, down, left, right
std::vector<Idx> GetAllNeighbors(const Idx &idx);

/// returns true if the specified idx of the grid is inside the footprint
bool IsIdxInFootprint(const Idx &idx, const Footprint &footprint,
                      const OccupancyGrid &grid);

} // namespace

void AddFootprintToGrid(const Footprint &footprint, OccupancyGrid &grid) {
  const std::vector<Idx> res = GetListOfCellsForFootprint(footprint, grid);
  grid.occupyCells(res);
};

namespace {

/// choose any corner of the footprint that is within the OccupancyGrid
/// bounds, we need this to start the floodfill
const std::vector<Idx> PickSeedsInGrid(const Footprint &footprint,
                                       const OccupancyGrid &grid) {
  std::vector<Idx> seeds;
  for (const Point &point : footprint) {
    const Idx idx = grid.pointToIdx(point);
    if (grid.isIdxInGrid(idx)) {
      seeds.push_back(idx);
    }
  }
  return seeds;
};

/// identify all possible neighboring grid cells, up, down, left, right
std::vector<Idx> GetAllNeighbors(const Idx &idx) {
  return {
      {static_cast<uint16_t>(idx.i + 1), idx.j},
      {static_cast<uint16_t>(idx.i - 1), idx.j},
      {idx.i, static_cast<uint16_t>(idx.j + 1)},
      {idx.i, static_cast<uint16_t>(idx.j - 1)},

      // {static_cast<int16_t>(idx.i + 1), static_cast<int16_t>(idx.j +
      // 1)},
      // {static_cast<int16_t>(idx.i - 1), static_cast<int16_t>(idx.j +
      // 1)},
      // {static_cast<int16_t>(idx.i + 1), static_cast<int16_t>(idx.j -
      // 1)},
      // {static_cast<int16_t>(idx.i - 1), static_cast<int16_t>(idx.j -
      // 1)},
  };
};

bool IsIdxInFootprint(const Idx &idx, const Footprint &footprint,
                      const OccupancyGrid &grid) {
  const Point point = grid.idxToPoint(idx);

  /// check these neighboring points as well
  return IsPointInFootprint(point, footprint);
};

/// use this w/ simulator obstacles to get list of cells to mark
/// occupied.
/// NOTE: assumes this footprint coords are in Revoy's frame
const std::vector<Idx> GetListOfCellsForFootprint(const Footprint &footprint,
                                                  const OccupancyGrid &grid) {

  std::vector<Idx> res;
  auto seeds = PickSeedsInGrid(footprint, grid);
  if (seeds.size() == 0) {
    /// none of the footprint points are in the grid's range
    return res;
  }
  // DFS frontier
  std::deque<Idx> frontier = {};
  frontier.insert(frontier.end(), seeds.begin(), seeds.end());
  std::vector<Idx> visited = {};

  /// failsafe is here to prevent bug in code from causing infinite loop
  uint16_t failsafe = 0;
  while (++failsafe > 0 && !frontier.empty()) {

    const Idx curr = frontier.front();
    frontier.pop_front();

    for (const Idx &next : GetAllNeighbors(curr)) {
      if (std::find(visited.begin(), visited.end(), next) != visited.end()) {
        /// already visited
        continue;
      }
      visited.push_back(next);
      if (IsIdxInFootprint(next, footprint, grid) && grid.isIdxInGrid(next)) {
        /// TODO optimize this copy, could use indexes and walk up a queue,
        /// that way we can return all visited cells at the end w/o extra
        /// copy.
        res.push_back(next);
        frontier.emplace_back(std::move(next));
      }
    }
  }

  return res;
};

} // namespace

}; // namespace planning
