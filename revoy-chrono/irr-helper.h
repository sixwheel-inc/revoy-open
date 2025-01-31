#pragma once

#include <chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h>

namespace revoy {

std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>
MakeIrrlichtIfPossible();

bool DidUserExitIrrlicht(
    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>
        irrlicht);

void UpdateIrrlicht(
    double time, double step, const chrono::vehicle::DriverInputs &driverInputs,
    std::shared_ptr<chrono::vehicle::ChWheeledVehicleVisualSystemIrrlicht>
        irrlicht);

} // namespace revoy
