#ifndef _HELM_PEDAL_H_
#define _HELM_PEDAL_H_

// Newton-meters
#define MAX_THROTTLE_TORQUE 25000.0

// Newton-meters
#define MAX_BRAKE_TORQUE 40000.0

typedef struct _Pedal {
  // [0, 1]
  float intensity;

  // depends
  float output;
} Pedal;

void ResetPedal(Pedal *pedal);

float UpdateGasPedal(Pedal *pedal, float intensity);

float UpdateBrakePedal(Pedal *pedal, float intensity);

#endif
