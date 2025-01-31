#pragma once

#include "chrono_models/vehicle/kraz/Kraz_tractor.h"
#include "chrono_models/vehicle/kraz/Kraz_trailer.h"
#include "chrono_models/vehicle/kraz/Revoy.h"

#include <chrono/collision/ChCollisionSystem.h>
#include <chrono/physics/ChContactMaterial.h>
#include <chrono_models/vehicle/ChVehicleModelDefs.h>
#include <chrono_vehicle/ChSubsysDefs.h>
#include <chrono_vehicle/ChTerrain.h>
#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h>

#include "revoy-chrono/chrono-mcap.h"

#include <cassert>
#include <memory>

/*
  Starting w/ the following beliefs:
  - initializing the different combinations of vehicles _may_ be generalizable,
    but it would be tricky and I don't have the knowledge yet to try.
  - there is a relatively small number of vehicle configurations that we want
    to test in the next year (like 5-8 configurations)

  Assembly will provide the following interface:
  - void step(inputs);
    synchronize and advance all the parts of the assembly
  - State getState() const;
    collect the specific things we care about like hitchForce, wheelSpeed, etc.

  Each implementation of Assembly will be responsible for correctly assembling
  the required vehicles together.

*/

namespace revoy {

class Assembly {

public:
  Assembly(std::shared_ptr<chrono::vehicle::ChTerrain> terrain_);

  /// This is the minimum data needed to run controls and helm
  /// any value that doesn't apply to a given assembly (e.g. revoySpeed when
  /// there is no revoy) will be 0.
  struct State {
    double time = 0;
    double tractorSpeed = 0;
    double tractorHeading = 0;
    double revoySpeed = 0;
    double revoyTractorHitchTorque = 0;
    double revoyTractorHitchAngle = 0;
    double revoySteerAngle = 0;
  };

  /// these values will come from controls and helm
  struct Feedback {
    double step = 0;
    double tractorThrottle = 0;
    double tractorSteer = 0;
    double tractorBrake = 0;
    double revoyDemandTorque = 0;
    double revoyDemandSteerAngle = 0;
  };

  /// get the current Assembly::State
  virtual State state() const final;

  /// used to get the time from whichever lead vehicle is available
  virtual double getTime() const final;

  /// use Assembly::Feedback to inform how the state should evolve
  virtual void step(const Feedback &feedback) final;

  /// useful during early development, may remove as our mcap/foxglove stuff
  /// becomes more mature
  virtual void attachToIrrlicht(
      chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht &irrlicht)
      const final;

  /// currently only used for approximate offline visualization, will be used
  /// for "dense log" type data capture
  virtual void attachToMcap(ChronoMcap &chronoMcap) const final;

  /// use these to terminate the simulation, these are failed unrecoverable
  /// situations, it will not be possible for the assembly to do much, and there
  /// is no value is simulating any further

  /// any of the vehicles in the assembly are rolled over (i.e. "flipped", "on
  /// its side")
  virtual bool isRolledOver() const final;

  /// any two vehicles in the assembly are facing opposite each other
  virtual bool isTurnedAround() const final;

protected:
  double initFwdVel = 0;
  chrono::ChCoordsys<> initPos;
  std::shared_ptr<chrono::vehicle::ChTerrain> terrain;

  /// called by child classes, to initialize all common things
  virtual void initialize() final;

  /// return nullptr if this component is not part of the assembly
  /// child classes that have these vehicles will override these and return the
  /// vehicle.
  virtual std::shared_ptr<chrono::vehicle::kraz::Kraz_tractor> tractor() const;
  virtual std::shared_ptr<chrono::vehicle::kraz::Revoy> revoy() const;
  virtual std::shared_ptr<chrono::vehicle::kraz::Kraz_trailer> trailer() const;
};

chrono::vehicle::DriverInputs
FeedbackToTractorInputs(const Assembly::Feedback &feedback);
chrono::vehicle::DriverInputs
FeedbackToRevoyInputs(const Assembly::Feedback &feedback);

} // namespace revoy
