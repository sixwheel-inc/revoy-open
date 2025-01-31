
#include "assembly/assembly.h"

#include "chrono_models/vehicle/kraz/Kraz_tractor_AutomaticTransmissionSimpleMap.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_EngineSimpleMap.h"
#include "chrono_models/vehicle/kraz/Kraz_tractor_Tire.h"
#include "chrono_models/vehicle/kraz/Kraz_trailer_Tire.h"
#include "chrono_models/vehicle/kraz/Revoy.h"
#include "chrono_vehicle/wheeled_vehicle/driveline/ChShaftsDriveline4WD.h"

#include <numbers>

namespace revoy {

Assembly::Assembly(std::shared_ptr<chrono::vehicle::ChTerrain> terrain_)
    : initFwdVel(0),
      initPos(chrono::ChCoordsys<>(chrono::ChVector3d(0, 0, 1), chrono::QUNIT)),
      terrain(terrain_) {}

void Assembly::initialize() {

  // a lot of this initialization code is copied / adapted from projectchrono
  // kraz demo

  /// one of these must be present, trailer() only is not possible
  assert(tractor() || revoy());

  using namespace chrono;
  using namespace chrono::vehicle;
  using namespace chrono::vehicle::kraz;

  const double tireStepSize = -1;
  if (tractor()) {
    // Create and initialize the tractor
    tractor()->SetCollisionSystemType(ChCollisionSystem::Type::BULLET);

    auto drvLine = std::static_pointer_cast<ChShaftsDriveline4WD>(
        tractor()->GetDriveline());
    drvLine->LockCentralDifferential(0, false);

    // Create and initialize the powertrain system
    std::shared_ptr<ChEngine> engine =
        chrono_types::make_shared<Kraz_tractor_EngineSimpleMap>("Engine");

    std::shared_ptr<ChTransmission> transmission =
        chrono_types::make_shared<Kraz_tractor_AutomaticTransmissionSimpleMap>(
            "Transmission");

    auto powertrain =
        chrono_types::make_shared<ChPowertrainAssembly>(engine, transmission);
    tractor()->InitializePowertrain(powertrain);

    // Create the tractor tires
    auto tire_FL =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_FL");
    auto tire_FR =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_FR");

    auto tire_RL1i =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RL1i");
    auto tire_RR1i =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RR1i");
    auto tire_RL1o =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RL1o");
    auto tire_RR1o =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RR1o");

    auto tire_RL2i =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RL2i");
    auto tire_RR2i =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RR2i");
    auto tire_RL2o =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RL2o");
    auto tire_RR2o =
        chrono_types::make_shared<Kraz_tractor_Tire>("TractorTire_RR2o");

    tractor()->InitializeTire(tire_FL, tractor()->GetAxle(0)->m_wheels[0],
                              VisualizationType::NONE);
    tractor()->InitializeTire(tire_FR, tractor()->GetAxle(0)->m_wheels[1],
                              VisualizationType::NONE);

    tractor()->InitializeTire(tire_RL1i, tractor()->GetAxle(1)->m_wheels[0],
                              VisualizationType::NONE);
    tractor()->InitializeTire(tire_RR1i, tractor()->GetAxle(1)->m_wheels[1],
                              VisualizationType::NONE);
    tractor()->InitializeTire(tire_RL1o, tractor()->GetAxle(1)->m_wheels[2],
                              VisualizationType::NONE);
    tractor()->InitializeTire(tire_RR1o, tractor()->GetAxle(1)->m_wheels[3],
                              VisualizationType::NONE);

    tractor()->InitializeTire(tire_RL2i, tractor()->GetAxle(2)->m_wheels[0],
                              VisualizationType::NONE);
    tractor()->InitializeTire(tire_RR2i, tractor()->GetAxle(2)->m_wheels[1],
                              VisualizationType::NONE);
    tractor()->InitializeTire(tire_RL2o, tractor()->GetAxle(2)->m_wheels[2],
                              VisualizationType::NONE);
    tractor()->InitializeTire(tire_RR2o, tractor()->GetAxle(2)->m_wheels[3],
                              VisualizationType::NONE);

    for (auto &axle : tractor()->GetAxles()) {
      for (auto &wheel : axle->GetWheels()) {
        if (tireStepSize > 0)
          wheel->GetTire()->SetStepsize(tireStepSize);
      }
    }

    // Recalculate vehicle mass, to properly account for all subsystems
    tractor()->InitializeInertiaProperties();
  }

  if (revoy()) {
    // Create the revoy tires
    auto r_tire_Li =
        chrono_types::make_shared<Kraz_tractor_Tire>("RevoyTire_Li");
    auto r_tire_Lo =
        chrono_types::make_shared<Kraz_tractor_Tire>("RevoyTire_Lo");
    auto r_tire_Ri =
        chrono_types::make_shared<Kraz_tractor_Tire>("RevoyTire_Ri");
    auto r_tire_Ro =
        chrono_types::make_shared<Kraz_tractor_Tire>("RevoyTire_Ro");

    revoy()->InitializeTire(r_tire_Li, revoy()->GetAxle(0)->m_wheels[0],
                            VisualizationType::PRIMITIVES);
    revoy()->InitializeTire(r_tire_Lo, revoy()->GetAxle(0)->m_wheels[1],
                            VisualizationType::PRIMITIVES);
    revoy()->InitializeTire(r_tire_Ri, revoy()->GetAxle(0)->m_wheels[2],
                            VisualizationType::PRIMITIVES);
    revoy()->InitializeTire(r_tire_Ro, revoy()->GetAxle(0)->m_wheels[3],
                            VisualizationType::PRIMITIVES);

    for (auto &axle : revoy()->GetAxles()) {
      for (auto &wheel : axle->GetWheels()) {
        if (tireStepSize > 0)
          wheel->GetTire()->SetStepsize(tireStepSize);
      }
    }
  }

  if (trailer()) {
    // Create the trailer tires
    auto tr_tire_FL = chrono_types::make_shared<Kraz_trailer_Tire>("FL");
    auto tr_tire_FR = chrono_types::make_shared<Kraz_trailer_Tire>("FR");
    auto tr_tire_ML = chrono_types::make_shared<Kraz_trailer_Tire>("ML");
    auto tr_tire_MR = chrono_types::make_shared<Kraz_trailer_Tire>("MR");
    auto tr_tire_RL = chrono_types::make_shared<Kraz_trailer_Tire>("RL");
    auto tr_tire_RR = chrono_types::make_shared<Kraz_trailer_Tire>("RR");

    trailer()->InitializeTire(tr_tire_FL, trailer()->GetAxle(0)->m_wheels[0],
                              VisualizationType::NONE);
    trailer()->InitializeTire(tr_tire_FR, trailer()->GetAxle(0)->m_wheels[1],
                              VisualizationType::NONE);
    trailer()->InitializeTire(tr_tire_ML, trailer()->GetAxle(1)->m_wheels[0],
                              VisualizationType::NONE);
    trailer()->InitializeTire(tr_tire_MR, trailer()->GetAxle(1)->m_wheels[1],
                              VisualizationType::NONE);
    trailer()->InitializeTire(tr_tire_RL, trailer()->GetAxle(2)->m_wheels[0],
                              VisualizationType::NONE);
    trailer()->InitializeTire(tr_tire_RR, trailer()->GetAxle(2)->m_wheels[1],
                              VisualizationType::NONE);
    for (auto &axle : trailer()->GetAxles()) {
      for (auto &wheel : axle->GetWheels()) {
        if (tireStepSize > 0)
          wheel->GetTire()->SetStepsize(tireStepSize);
      }
    }
  }

  // set viz type, only matters if irrlicht is up
  if (tractor()) {
    tractor()->SetChassisVisualizationType(VisualizationType::MESH);
    tractor()->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    tractor()->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    tractor()->SetWheelVisualizationType(VisualizationType::MESH);
    tractor()->SetTireVisualizationType(VisualizationType::MESH);
  }
  if (revoy()) {
    revoy()->SetChassisVisualizationType(VisualizationType::MESH);
    revoy()->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    revoy()->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
    revoy()->SetWheelVisualizationType(VisualizationType::MESH);
    revoy()->SetTireVisualizationType(VisualizationType::MESH);
  }
  if (trailer()) {
    trailer()->SetChassisVisualizationType(VisualizationType::MESH);
    trailer()->SetSuspensionVisualizationType(VisualizationType::PRIMITIVES);
    trailer()->SetWheelVisualizationType(VisualizationType::MESH);
    trailer()->SetTireVisualizationType(VisualizationType::MESH);
  }
}

double Assembly::getTime() const {
  assert(tractor() || revoy());
  if (tractor()) {
    return tractor()->GetChTime();
  }
  return revoy()->GetChTime();
}

Assembly::State Assembly::state() const {
  State state;

  /// one of these must be present, trailer() only is not possible
  assert(tractor() || revoy());

  state.time = getTime();

  if (tractor()) {
    state.tractorSpeed = tractor()->GetSpeed();
    state.tractorHeading = tractor()->GetRot().GetCardanAnglesXYZ().z();
  }
  if (revoy()) {
    state.revoySpeed = revoy()->GetSpeed();
    const double revoyHeading = revoy()->GetRot().GetCardanAnglesXYZ().z();
    state.revoyTractorHitchAngle = state.tractorHeading - revoyHeading;
    state.revoyTractorHitchTorque = revoy()->GetHitchTorque();
    state.revoySteerAngle = revoy()->GetSteeringAngle(0, chrono::vehicle::LEFT);
  }

  return state;
}

void Assembly::step(const Feedback &feedback) {

  /// one of these must be present, trailer() only is not possible
  assert(tractor() || revoy());

  /// can't really do much w/o this
  assert(terrain);

  const double time = getTime();
  if (tractor()) {
    const auto inputs = FeedbackToTractorInputs(feedback);
    tractor()->Synchronize(time, inputs, *terrain);
  }
  if (revoy()) {
    const auto inputs = FeedbackToRevoyInputs(feedback);
    revoy()->Synchronize(time, inputs, *terrain);
  }
  if (trailer()) {
    trailer()->Synchronize(time, {}, *terrain);
  }
  terrain->Synchronize(time);

  if (tractor()) {
    tractor()->Advance(feedback.step);
  }
  if (revoy()) {
    revoy()->Advance(feedback.step);
  }
  if (trailer()) {
    trailer()->Advance(feedback.step);
  }
  terrain->Advance(feedback.step);
};

std::shared_ptr<chrono::vehicle::kraz::Kraz_tractor> Assembly::tractor() const {
  return nullptr;
};
std::shared_ptr<chrono::vehicle::kraz::Revoy> Assembly::revoy() const {
  return nullptr;
};
std::shared_ptr<chrono::vehicle::kraz::Kraz_trailer> Assembly::trailer() const {
  return nullptr;
};

chrono::vehicle::DriverInputs
FeedbackToTractorInputs(const Assembly::Feedback &feedback) {

  static constexpr double NO_CLUTCH = 0;
  return {
      feedback.tractorSteer,
      feedback.tractorThrottle,
      feedback.tractorBrake,
      NO_CLUTCH,
  };
};

chrono::vehicle::DriverInputs
FeedbackToRevoyInputs(const Assembly::Feedback &feedback) {

  static constexpr double MAX_DEMAND_TORQUE = 1500;

  /// TODO instead of using DriverInput, actually model an electric motor
  /// and provide the torque directly to the motor.
  const bool regen = feedback.revoyDemandTorque < 0;
  const double intensity =
      fmin(fabs(feedback.revoyDemandTorque / MAX_DEMAND_TORQUE), 1);
  const double revoyThrottle = regen ? 0 : intensity;
  const double revoyBrake = regen ? intensity : 0;

  return {
      feedback.revoyDemandSteerAngle,
      revoyThrottle,
      revoyBrake,
      0,
  };
};

void Assembly::attachToIrrlicht(
    chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht &irrlicht) const {
  if (tractor()) {
    irrlicht.AttachVehicle(&*(tractor()));
  } else if (revoy()) {
    irrlicht.AttachVehicle(&*(revoy()));
  }
}

void Assembly::attachToMcap(ChronoMcap &chronoMcap) const {
  if (tractor()) {
    chronoMcap.attachVehicle(
        std::string("tractor"),
        std::static_pointer_cast<chrono::vehicle::ChVehicle>(tractor()));
  }
  if (revoy()) {
    chronoMcap.attachVehicle(
        std::string("revoy"),
        std::static_pointer_cast<chrono::vehicle::ChVehicle>(revoy()));
  }
  if (trailer()) {
    chronoMcap.attachTrailer(
        std::string("trailer"),
        std::static_pointer_cast<chrono::vehicle::ChWheeledTrailer>(trailer()));
  }
}

bool Assembly::isRolledOver() const {

  std::vector<double> rolls{
      tractor() ? std::fabs(tractor()->GetRot().GetCardanAnglesXYZ().x()) : 0,
      revoy() ? std::fabs(revoy()->GetRot().GetCardanAnglesXYZ().x()) : 0,
      trailer()
          ? std::fabs(
                trailer()->GetChassis()->GetRot().GetCardanAnglesXYZ().x())
          : 0,
  };
  double maxRoll = *std::max_element(rolls.begin(), rolls.end());
  static constexpr double ROLLOVER = std::numbers::pi / 4.0;

  return maxRoll > ROLLOVER;
}

bool Assembly::isTurnedAround() const {

  std::vector<double> yaws{
      tractor() ? std::fabs(tractor()->GetRot().GetCardanAnglesXYZ().z()) : 0,
      revoy() ? std::fabs(revoy()->GetRot().GetCardanAnglesXYZ().z()) : 0,
      trailer()
          ? std::fabs(
                trailer()->GetChassis()->GetRot().GetCardanAnglesXYZ().x())
          : 0,
  };

  if (yaws.size() >= 2) {
    for (uint8_t index = 0; index < yaws.size() - 1; index++) {
      const double relativeYaw = yaws[index] - yaws[index + 1];
      static constexpr double TURNED_AROUND = (2.0 / 3.0) * std::numbers::pi;
      if (fabs(relativeYaw) > TURNED_AROUND) {
        return true;
      }
    }
  }

  return false;
}

} // namespace revoy
