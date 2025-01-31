#pragma once

#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h>

#include "assembly/assembly.h"
#include "helm/helm.h"
#include "revoy-chrono/chrono-mcap.h"

namespace revoy {

/// encapsulates a single complete execution of the 3D Simulator
class Zaxis final {

public:
  enum AssemblyType {
    TractorTrailerType,
    TractorRevoyTrailerType,
  };

  /// describes what the vehicle will attempt to do
  struct Scenario {
    std::string name;
    std::vector<Goal> goals;
    double tStep = 0.004;
    double timeout = 100;
    // todo:
    // obstactles
    // terrain
  };

  /// the goals provided will inform how the tractor is operated.
  /// the tractor will attempt to reach each goal state.
  Zaxis(Scenario &&scenario, AssemblyType assemblyType);
  virtual ~Zaxis();

  /// returns false if timedout or other error prevents reaching all goals
  /// returns true if all goals were met.
  bool runThroughAllGoals();

  /// returns false if the goal is not reached after the step is taken
  /// returns true if the goal is met.
  bool step(Helm &helm, const Goal &goal);

private:
  Scenario scenario;
  std::shared_ptr<Assembly> assembly;
  std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>
      irrlicht;

  /// mcap stuff TODO clean up a bit
  mcap::McapWriter mcapWriter;
  ChronoMcap chronoMcap;
};

} // namespace revoy
