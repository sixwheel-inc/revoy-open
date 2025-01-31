#define MCAP_IMPLEMENTATION

#include "zaxis/zaxis.h"

#include <cstdlib>
#include <iostream>
#include <numbers>

#include <chrono_vehicle/terrain/FlatTerrain.h>

#include "helm/instrument-reading.h"

#include "assembly/tractor-revoy-trailer.h"
#include "assembly/tractor-trailer.h"

#include "revoy-chrono/chrono-mcap.h"
#include "revoy-chrono/env-check.h"
#include "revoy-chrono/irr-helper.h"

namespace revoy {

namespace {

/// Allows the top-level user code to specify which specific assembly we want to
/// run
std::shared_ptr<Assembly> MakeAssembly(Zaxis::AssemblyType assemblyType,
                                       const Zaxis::Scenario &scenario) {

  SetDataPaths();

  assert(scenario.goals.size() > 0);
  auto terrain = std::make_shared<chrono::vehicle::FlatTerrain>(0);
  switch (assemblyType) {
  case Zaxis::TractorTrailerType:
    return std::make_shared<TractorTrailer>(terrain);
  case Zaxis::TractorRevoyTrailerType:
    return std::make_shared<TractorRevoyTrailer>(terrain);
  default:
    std::cout << "Unrecognized AssemblyType" << std::endl;
    exit(1);
  }
};

/// TODO: this is a bit weird, passing in and returning the same refernce in
/// order to perform all of this in the Initializer-List of Zaxis default
/// constructor
mcap::McapWriter &InitMcapWriter(mcap::McapWriter &mcapWriter,
                                 const std::string name) {
  const std::string outputFilename =
      chrono::GetChronoOutputPath() + name + std::string(".mcap");
  auto options = mcap::McapWriterOptions("");
  const auto res = mcapWriter.open(outputFilename, options);
  if (!res.ok()) {
    std::cerr << "Failed to open " << outputFilename
              << " for writing: " << res.message << std::endl;
    exit(1);
  }
  return mcapWriter;
}

double FakeLongControl(double hitchTorque) {
  static constexpr double MAX_HITCH_TORQUE = 5000;
  static constexpr double MAX_DEMAND_TORQUE = 1000;

  const int dir = (hitchTorque >= 0) ? 1 : -1;
  const double ratio = std::fabs(hitchTorque) / MAX_HITCH_TORQUE;
  const double demand = MAX_DEMAND_TORQUE * ratio * dir;
  return std::fmin(1, std::fmax(-1, demand));
}

double FakeLatControl(double _) { return 0; }

} // namespace

Zaxis::Zaxis(Zaxis::Scenario &&scenario_, Zaxis::AssemblyType assemblyType)
    : scenario(std::move(scenario_)),
      assembly(MakeAssembly(assemblyType, scenario)),
      irrlicht(MakeIrrlichtIfPossible()),
      chronoMcap(InitMcapWriter(mcapWriter, scenario.name)) {

  std::cout << "scenario: " << scenario.name << std::endl;

  if (irrlicht) {
    /// optional online-video, useful while developing but very slow and CPU/GPU
    /// intesive, not always available.
    assembly->attachToIrrlicht(*irrlicht);
  }

  /// log everything to mcap
  /// TODO currently only logging xyz and rpy for all vehicles in assembly
  assembly->attachToMcap(chronoMcap);
};

Zaxis::~Zaxis() { mcapWriter.close(); }

bool Zaxis::runThroughAllGoals() {

  std::cout << "start running through all goals." << std::endl;

  /// reset helm
  Helm helm;
  helmReset(&helm);

  /// start goal iteration
  auto goalIter = scenario.goals.begin();

  /// run until all goals met, rollover, or timeout
  while (true) {

    // update and synchronize the whole simulation
    bool goalMet = step(helm, *goalIter);

    // advance goals
    if (goalMet) {
      std::cout << "goal met! " << std::to_string(assembly->getTime()) << "s, "
                << std::to_string(goalIter->velocity.target) << " m/s, "
                << std::to_string(goalIter->heading.target) << " rad"
                << std::endl;
      helmReset(&helm);
      goalIter++;
    }

    // check if all goals complete
    if (goalIter == scenario.goals.end()) {
      std::cout << "all goals met! " << std::to_string(assembly->getTime())
                << "s" << std::endl;
      break;
    }

    // check if timeout
    if (assembly->getTime() > scenario.timeout) {
      std::cout << "timeout! " << std::to_string(assembly->state().tractorSpeed)
                << " m/s, " << std::to_string(assembly->state().tractorHeading)
                << " rad" << std::endl;
      break;
    }

    // check if rollover
    if (assembly->isRolledOver()) {
      std::cout << "rollover! " << std::to_string(assembly->getTime()) << "s"
                << std::endl;
      break;
    }
    // check if turned around
    if (assembly->isTurnedAround()) {
      std::cout << "turned around! " << std::to_string(assembly->getTime())
                << "s" << std::endl;
      break;
    }

    /// in the special case where the user is online-visualizing the
    /// simulation, then exit the simulation when the user exits the
    /// viz window
    if (DidUserExitIrrlicht(irrlicht)) {
      std::cout << "viz exit!" << std::endl;
      break;
    }

  } // while (true)

  return goalIter == scenario.goals.end();
}

bool Zaxis::step(Helm &helm, const Goal &goal) {

  ///
  /// === GET ASSEMBLY STATE ===
  ///

  const auto state = assembly->state();

  ///
  /// === HELM UPDATE ===
  ///

  InstrumentReading reading{};

  reading.clock = state.time;
  reading.speedometer = state.tractorSpeed;
  reading.compass = state.tractorHeading;

  Action helmAction = {};
  const bool goalMet = helmUpdate(&helm, &reading, &goal, &helmAction);

  ///
  /// === CHRONO (ASSEMBLY) UPDATE ===
  ///

  Assembly::Feedback feedback;

  feedback.step = scenario.tStep;
  feedback.tractorThrottle = helmAction.throttle;
  feedback.tractorBrake = helmAction.brake;
  feedback.tractorSteer = helmAction.turn;
  feedback.revoyDemandTorque = FakeLongControl(state.revoyTractorHitchTorque);
  feedback.revoyDemandSteerAngle = FakeLatControl(state.revoyTractorHitchAngle);

  assembly->step(feedback);

  ///
  /// === IRRLICHT ===
  ///

  const auto tractorInputs = FeedbackToTractorInputs(feedback);
  UpdateIrrlicht(reading.clock, feedback.step, tractorInputs, irrlicht);

  ///
  /// === MCAP ===
  ///

  /// TODO (maybe): make ChronoMcap inherit from ChVisualSystem, and pass it
  /// into chrono
  chronoMcap.step(reading.clock);

  ///
  /// === STDOUT DEBUG ===
  ///

  // static uint8_t count = 0;
  // if (count++ % 1000 == 1) {

  //   std::cout << "--------------------" << std::endl;
  //   std::cout << "time:  " << std::to_string(reading.clock) << std::endl;
  //   std::cout << "speed:  " << std::to_string(reading.speedometer)
  //             << std::endl;
  //   std::cout << "heading:  " << std::to_string(reading.compass)
  //             << std::endl;
  // }

  return goalMet;
}

} // namespace revoy
