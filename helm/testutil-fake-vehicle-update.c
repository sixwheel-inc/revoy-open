#include "testutil-fake-vehicle-update.h"

#include <math.h>
#include <stdbool.h>
#include <stdio.h>

const float ACCEL = 5;
const float YAW_RATE = M_PI / 4.0;
const float EPSILON = 0.001;

void FakeVehicleUpdate(const Action *action, const float dt,
                       InstrumentReading *reading) {

  // fake vehicle response
  if (action->direction == STOP) {
    return;
  }

  const float accel = action->direction == FWD ? ACCEL : -ACCEL;
  if (action->throttle > EPSILON) {
    reading->speedometer += accel * dt;
  } else {
    // fake friction
    reading->speedometer *= (expf(-dt));
  }
  if (action->brake > EPSILON) {
    reading->speedometer -= accel * dt;
  }

  const float yawRate = action->direction == FWD ? YAW_RATE : -YAW_RATE;
  const bool isTurn = fabs(action->turn) > EPSILON;
  if (isTurn) {
    const bool isLeft = action->turn > EPSILON;
    if (isLeft) {
      reading->compass += yawRate * dt;
    } else {
      reading->compass -= yawRate * dt;
    }
  }

  static const float TWO_PI = 2.0 * M_PI;
  while (reading->compass > M_PI) {
    reading->compass -= TWO_PI;
  }
  while (reading->compass < -M_PI) {
    reading->compass += TWO_PI;
  }
}

void FakeVehicleUpdateMain(const Action *action, const float dt,
                           InstrumentReading *reading) {

  Action normalizedAction = *action;

  // helm main uses direction to flip throttle, ignore that
  normalizedAction.brake = fabs(action->brake);
  normalizedAction.throttle = fabs(action->throttle);
  FakeVehicleUpdate(&normalizedAction, dt, reading);
}
