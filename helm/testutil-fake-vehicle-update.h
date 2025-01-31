#ifndef _HELM_FAKE_VEHICLE_UPDATE_H_
#define _HELM_FAKE_VEHICLE_UPDATE_H_

#include "helm/action.h"
#include "helm/instrument-reading.h"

#ifdef __cplusplus
extern "C" {
#endif

/// used to test the Helm, by responding to Helm Action by updating
/// current Velocity and Heading appropriately.
/// Basic point-mass kinematics only, no vehicle dynamics, rough drag approx
void FakeVehicleUpdate(const Action *action, const float dt,
                       InstrumentReading *reading);

/// used to test the HelmMain, same as above but with different values.
void FakeVehicleUpdateMain(const Action *action, const float dt,
                           InstrumentReading *reading);

#ifdef __cplusplus
}
#endif

#endif // _HELM_FAKE_VEHICLE_UPDATE_H_
