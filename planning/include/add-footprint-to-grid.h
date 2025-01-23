#pragma once

#include "planning/occupancy-grid.h"
#include "planning/types.h"

namespace planning {

/// (used in simulator) sets the cells overlayed by footprint to Occupied
/// i.e. "rasterizes" the footprint into the grid, using private floodfill
/// method.
/// NOTE: assumes this footprint coords are in the Occupancy Grid frame
void AddFootprintToGrid(const Footprint &footprint, OccupancyGrid &grid);

}; // namespace planning
