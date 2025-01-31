
/// Most of the code in this test is adapted from projectchrono demos
/// the RevoyChrono wrapper was a helpul abstraction for this test, but
/// won't serve the user.

#include "chrono_vehicle/ChVehicleModelData.h"
#include "chrono_vehicle/ChWorldFrame.h"
#include "chrono_vehicle/driver/ChInteractiveDriverIRR.h"
#include "chrono_vehicle/terrain/RigidTerrain.h"
#include "chrono_vehicle/wheeled_vehicle/ChWheeledVehicleVisualSystemIrrlicht.h"

#include "RevoyKraz.h"
#include "revoy-chrono/env-check.h"

using namespace chrono::irrlicht;

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::kraz;

using namespace revoy;

class RevoyChrono {

public:
  struct State {
    double time = 0;
    double hitchForce = 0;
    double speed = 0;
  };

  State getState() const {
    return {
        revoy->GetSystem()->GetChTime(),
        revoy->GetHitchTorque(),
        revoy->GetRevoy().GetSpeed(),
    };
  }

  RevoyChrono();

  void step(double step_size, const chrono::vehicle::DriverInputs &driverInputs,
            const chrono::vehicle::DriverInputs &revoyInputs);

  std::shared_ptr<chrono::vehicle::kraz::RevoyKraz> getRevoy() {
    return revoy;
  };

private:
  std::shared_ptr<chrono::vehicle::kraz::RevoyKraz> revoy;
  std::shared_ptr<chrono::vehicle::RigidTerrain> terrain;
};

/// partially copied from projectchrono
std::shared_ptr<RigidTerrain>
MakeDefaultTerrain(std::shared_ptr<RevoyKraz> revoy);
std::shared_ptr<RevoyKraz> MakeDefaultRevoy();

/// copied from projectchrono demos
void AddObstacle(ChSystem *sys);

RevoyChrono::RevoyChrono()
    : revoy(MakeDefaultRevoy()), terrain(MakeDefaultTerrain(revoy)) {

  // Add a hardcoded mesh obstacle
  // TODO pass in obstacle info, set later
  AddObstacle(revoy->GetSystem());
}

void RevoyChrono::step(double step_size, const DriverInputs &driverInputs,
                       const DriverInputs &revoyInputs) {

  double time = revoy->GetSystem()->GetChTime();

  if (Vdot(revoy->GetTractor().GetChassisBody()->GetRotMat().GetAxisZ(),
           ChWorldFrame::Vertical()) < 0) {
    std::cout << "rollover!" << std::endl;
  }

  // Update modules (process inputs from other modules)
  terrain->Synchronize(time);
  revoy->Synchronize(time, driverInputs, revoyInputs, *terrain);

  // Advance simulation for one timestep for all modules
  terrain->Advance(step_size);
  revoy->Advance(step_size);
}

/// most of this function is copied from projectchrono
std::shared_ptr<RevoyKraz> MakeDefaultRevoy() {

  // Create vehicle
  auto revoy = std::make_shared<RevoyKraz>();

  revoy->Initialize();

  // Optionally, enable collision for the vehicle wheels.
  // In this case, you must also disable collision between the chassis and
  // wheels (unless the chassis collision model is accurate enough to account
  // for the wheel wells).
  revoy->GetTractor().SetWheelCollide(true);
  revoy->GetRevoy().SetWheelCollide(true);

  revoy->GetTractor().EnableRealtime(true);

  return revoy;
}

/// most of this function is copied from projectchrono
std::shared_ptr<RigidTerrain>
MakeDefaultTerrain(std::shared_ptr<RevoyKraz> revoy) {
  // Create the terrain
  auto terrain = std::make_shared<RigidTerrain>(revoy->GetSystem());

  ChContactMaterialData minfo;
  minfo.mu = 0.9f;
  minfo.cr = 0.1f;
  minfo.Y = 2e7f;
  auto terrain_mat =
      minfo.CreateMaterial(revoy->GetSystem()->GetContactMethod());

  auto patch = terrain->AddPatch(terrain_mat, CSYSNORM, 100.0, 100.0);
  patch->SetTexture(vehicle::GetDataFile("terrain/textures/dirt.jpg"), 20, 20);

  auto slope = QuatFromAngleY(-15 * CH_DEG_TO_RAD);
  auto ramp = terrain->AddPatch(
      terrain_mat, ChCoordsys<>(ChVector3d(20, 3, 0), slope), 20, 6);
  ramp->SetTexture(vehicle::GetDataFile("terrain/textures/concrete.jpg"), 2, 2);

  terrain->Initialize();
  return terrain;
}

/// this function is copied from projectchrono
void AddObstacle(ChSystem *sys) {

  // Obstacle collision model (nbo obstacle if CollisionType::NONE)
  auto obstacle_coll_type = CollisionType::NONE;

  if (obstacle_coll_type == CollisionType::NONE) {
    return;
  }

  auto pos = ChVector3d(8, 0, 1);
  double radius = 1;
  double length = 2;

  std::string text_filename = "textures/rock.jpg";

  auto body = chrono_types::make_shared<ChBody>();
  body->SetPos(pos);
  body->SetFixed(true);
  body->EnableCollision(true);
  sys->Add(body);

  std::shared_ptr<ChContactMaterial> mat =
      ChContactMaterial::DefaultMaterial(sys->GetContactMethod());

  if (obstacle_coll_type == CollisionType::PRIMITIVES) {
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeCylinder>(
        mat, radius, length);
    body->AddCollisionShape(ct_shape);

    auto vis_shape =
        chrono_types::make_shared<ChVisualShapeCylinder>(radius, length);
    vis_shape->SetTexture(GetChronoDataFile(text_filename));
    body->AddVisualShape(vis_shape);

    return;
  }

  auto mesh = ChTriangleMeshConnected::CreateFromWavefrontFile(
      GetChronoDataFile("models/cylinderZ.obj"), false, true);
  mesh->Transform(ChVector3d(0, 0, 0),
                  ChMatrix33<>(ChVector3d(radius, radius, length)));

  if (obstacle_coll_type == CollisionType::MESH) {
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeTriangleMesh>(
        mat, mesh, false, false, 0.005);
    body->AddCollisionShape(ct_shape);
  } else {
    auto ct_shape = chrono_types::make_shared<ChCollisionShapeConvexHull>(
        mat, mesh->GetCoordsVertices());
    body->AddCollisionShape(ct_shape);
  }

  auto vis_shape = chrono_types::make_shared<ChVisualShapeTriangleMesh>();
  vis_shape->SetMesh(mesh);
  vis_shape->SetBackfaceCull(true);
  vis_shape->SetTexture(GetChronoDataFile(text_filename));
  body->AddVisualShape(vis_shape);
}

// Simulation step size
double step_size = 2e-3;

// End time (used only if no run-time visualization)
double t_end = 20;

// =============================================================================

int main() {
  std::cout << "Chrono version: " << CHRONO_VERSION << std::endl;

  SetDataPaths();

  // Create vehicle
  RevoyChrono revoy;

  std::cout << "AAAAAA!" << std::endl;

  // revoy.getRevoy()->SetChassisVisualizationType(VisualizationType::MESH,
  //                                               VisualizationType::MESH);
  // revoy.getRevoy()->SetSuspensionVisualizationType(
  //     VisualizationType::PRIMITIVES, VisualizationType::PRIMITIVES);
  // revoy.getRevoy()->SetSteeringVisualizationType(VisualizationType::PRIMITIVES);
  // revoy.getRevoy()->SetWheelVisualizationType(VisualizationType::MESH,
  //                                             VisualizationType::MESH);
  // revoy.getRevoy()->SetTireVisualizationType(VisualizationType::MESH,
  //                                            VisualizationType::MESH);

  // std::shared_ptr<ChVehicleVisualSystem> vis;
  // if (!IsRunningInTest()) {
  //   auto vis_irr =
  //       chrono_types::make_shared<ChWheeledVehicleVisualSystemIrrlicht>();
  //   vis_irr->SetWindowTitle("Rollover Demo");
  //   vis_irr->SetChaseCamera(ChVector3d(0.0, 0.0, 2.0), 5.0, 0.05);
  //   vis_irr->Initialize();
  //   vis_irr->AddLightDirectional(70, 20);
  //   vis_irr->AddSkyBox();
  //   vis_irr->AddLogo();
  //   vis_irr->AttachVehicle(&revoy.getRevoy()->GetTractor());
  //   // vis_irr->AttachVehicle(&revoy.GetTrailer());
  //   vis = vis_irr;
  // }

  double time = 0;
  while (true) {

    // // Render scene
    // if (!IsRunningInTest() && vis) {
    //   if (!vis->Run()) {
    //     break;
    //   }

    //   vis->BeginScene();
    //   vis->Render();
    //   vis->EndScene();
    // }

    if (time > t_end) {
      break;
    }

    /// this will come from Helm
    DriverInputs driverInputs = {0, 0.5, 0, 0};

    revoy.step(step_size, driverInputs, driverInputs);

    // if (!IsRunningInTest() && vis) {
    //   vis->Synchronize(time, driverInputs);
    //   vis->Advance(step_size);
    // }

    time += step_size;
  }

  std::cout << "BBBBBB!" << std::endl;

  return 0;
};
