#define MCAP_IMPLEMENTATION

#include "assembly/include/tractor-revoy-trailer.h"
#include "assembly/include/tractor-trailer.h"
#include "revoy-chrono/chrono-mcap.h"
#include "revoy-chrono/env-check.h"
#include "revoy-chrono/irr-helper.h"

#include <chrono/core/ChGlobal.h>
#include <chrono_vehicle/ChVehicleModelData.h>
#include <chrono_vehicle/terrain/FlatTerrain.h>

#include <cstdint>
#include <filesystem>
#include <iostream>
#include <memory>

#include <catch2/catch_test_macros.hpp>

#include "assembly/assembly.h"

using namespace revoy;

/// use a shared_ptr<Assembly> in this test-loop to mimic how this will be used
void DoLoop(std::shared_ptr<Assembly> assembly);

TEST_CASE("assembly") {

  SetDataPaths();

  {
    /// Tractor-Trailer
    auto terrain = std::make_shared<chrono::vehicle::FlatTerrain>(0);
    auto assembly = std::make_shared<TractorTrailer>(terrain);
    DoLoop(std::static_pointer_cast<Assembly>(assembly));

    /// vehicle can accelerate and turn
    CHECK(assembly->state().tractorSpeed > 5);
    CHECK(assembly->state().tractorHeading > std::numbers::pi / 4.0);
  }

  {
    /// Tractor-Revoy-Trailer
    auto terrain = std::make_shared<chrono::vehicle::FlatTerrain>(0);
    auto assembly = std::make_shared<TractorRevoyTrailer>(terrain);
    DoLoop(std::static_pointer_cast<Assembly>(assembly));

    /// vehicle can accelerate and turn
    CHECK(assembly->state().tractorSpeed > 5);
    CHECK(assembly->state().tractorHeading > std::numbers::pi / 4.0);
  }
}

void DoLoop(std::shared_ptr<Assembly> assembly) {

  double step = 2e-3;
  double endTime = 5;

  const std::string out_dir = chrono::GetChronoOutputPath();

  /// init live 3D viz if possible
  std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>
      irrlicht = MakeIrrlichtIfPossible();
  if (irrlicht) {
    assembly->attachToIrrlicht(*irrlicht);
  }

  /// init mcap logging for offline viz
  std::string mcapFilename = out_dir + "test-assembly-tractor-trailer.mcap";
  auto options = mcap::McapWriterOptions("");
  mcap::McapWriter writer;
  const auto res = writer.open(mcapFilename, options);
  if (!res.ok()) {
    std::cerr << "Failed to open " << mcapFilename
              << " for writing: " << res.message << std::endl;
    exit(1);
  }
  ChronoMcap mcap(writer);
  assembly->attachToMcap(mcap);

  /// loop
  double time = 0;
  while (true) {
    if (time > endTime) {
      break;
    }
    Assembly::Feedback feedback;
    feedback.tractorThrottle = 1;
    feedback.tractorSteer = 1;
    feedback.step = step;
    assembly->step(feedback);
    time += step;

    if (DidUserExitIrrlicht(irrlicht)) {
      break;
    }
    const auto tractorInputs = FeedbackToTractorInputs(feedback);
    UpdateIrrlicht(time, step, tractorInputs, irrlicht);

    mcap.step(time);
  }
  writer.close();
}
