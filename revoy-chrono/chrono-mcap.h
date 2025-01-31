#pragma once

#include <fstream>
#include <map>
#include <vector>

#include <chrono_vehicle/ChVehicle.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledTrailer.h>

#include "mcap/writer.hpp"

namespace revoy {

class ChronoMcap final {
public:
  /// sets up relevant topics
  ChronoMcap(mcap::McapWriter &writer);

  /// call once for each vehicle that needs to be rendered
  void attachVehicle(const std::string &vehicleName,
                     std::shared_ptr<chrono::vehicle::ChVehicle> vehicle);

  /// FIXME HACK trailer does not inherit from ChVehicle for some reason
  /// call once for each vehicle that needs to be rendered
  void
  attachTrailer(const std::string &vehicleName,
                std::shared_ptr<chrono::vehicle::ChWheeledTrailer> trailer);

  /// add the next set of vehicle poses to the mcap
  void step(double writeTime);

private:
  /// mcap writer
  mcap::McapWriter &writer;
  std::map<std::string, mcap::ChannelId> channelIds;
  size_t frameIndex = 0;

  /// chrono
  std::map<std::string, std::shared_ptr<chrono::vehicle::ChVehicle>> vehicles;
  /// FIXME HACK trailer does not inherit from ChVehicle for some reason
  std::map<std::string, std::shared_ptr<chrono::vehicle::ChWheeledTrailer>>
      trailers;
};

} // namespace revoy
