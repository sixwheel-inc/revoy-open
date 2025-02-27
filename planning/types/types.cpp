
#include "planning/types.h"

#include <cmath>

double fixRadian(double value) {
  double v = fmod(value, 2.0 * M_PI);
  if (v < -M_PI)
    v += 2.0 * M_PI;
  else if (v >= M_PI)
    v -= 2.0 * M_PI;
  return v;
};
