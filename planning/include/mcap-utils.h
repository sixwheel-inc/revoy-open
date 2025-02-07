#pragma once

#include "planning/occupancy-grid.h"
#include "planning/types.h"

#include <map>
#include <vector>

#include "mcap/writer.hpp"

namespace planning {

struct Scene {
  Footprints revoy;
  HookedPose revoyPose;
  Path plannedPath;
  Scenario scenario;
  Footprints visibleEntities;
  std::shared_ptr<OccupancyGrid> grid;
  Graph graph;
};

class McapWrapper {
private:
  mcap::McapWriter writer;
  std::map<std::string, mcap::ChannelId> channelIds;
  size_t frameIndex = 0;

public:
  McapWrapper(const std::string outputFilename);
  ~McapWrapper();
  void write(const Scene &scene, int64_t writeTime);
};
} // namespace planning
