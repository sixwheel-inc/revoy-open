#include "revoy-chrono/irr-helper.h"
#include "revoy-chrono/env-check.h"

namespace revoy {

using namespace chrono::vehicle;

std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> MakeIrrlichtIfPossible() {

  if (IsIrrlichtDisabled()) {
    std::cout << "user has disabled irrlicht visualizations" << std::endl;
    return nullptr;
  }

  if (IsRunningInTest()) {
    std::cout << "in a test, no irrlicht possible" << std::endl;
    return nullptr;
  }

  std::cout << "live irrlicht visualizer will be initialized" << std::endl;
  auto vis_irr =
      chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
  vis_irr->SetWindowTitle("Rollover Demo");
  vis_irr->SetChaseCamera(chrono::ChVector3d(0.0, 0.0, 2.0), 5.0, 0.05);
  vis_irr->Initialize();
  vis_irr->AddLightDirectional(70, 20);
  vis_irr->AddSkyBox();
  vis_irr->AddLogo();
  return vis_irr;
}

bool DidUserExitIrrlicht(
    std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> irrlicht) {
  return irrlicht && !irrlicht->Run();
}

void UpdateIrrlicht(
    double time, double step, const DriverInputs &driverInputs,
    std::shared_ptr<ChWheeledVehicleVisualSystemIrrlicht> irrlicht) {
  if (irrlicht) {

    irrlicht->BeginScene();
    irrlicht->Render();
    irrlicht->EndScene();

    irrlicht->Synchronize(time, driverInputs);
    irrlicht->Advance(step);
  }
}

} // namespace revoy
