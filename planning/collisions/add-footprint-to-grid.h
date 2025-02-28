#pragma once

#include "planning/occupancy-grid.h"
#include "planning/types.h"

namespace planning {

/// (used in simulator) sets the cells overlayed by footprint to Occupied
/// i.e. "rasterizes" the footprint into the grid, using private floodfill
/// method.
/// NOTE: assumes this footprint coords are in the Occupancy Grid frame
void AddFootprintToGrid(const Footprint &footprint, OccupancyGrid &grid);

// Rasterizes the polygons into the grid, using AddFootprintToGrid in a loop,
// and after transforming the footprints into the pose frame (i.e. put them in
// the grid relative to the pose).
void FootprintsToOccupancyGrid(OccupancyGrid &grid,
                               const Footprints &footprints,
                               const HookedPose &pose);

}; // namespace planning
