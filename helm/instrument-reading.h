#ifndef _HELM_C_INSTRUMENT_READING_H_
#define _HELM_C_INSTRUMENT_READING_H_

#if (!defined(M_PI))
#define M_PI 3.141592653589793238462643383279502884
#endif

#include <stdint.h>

/**
 * @brief represents how a human driver determines important vehicle states
 */
typedef struct _InstrumentReading {

  /**
   * @brief forward velocity | speed
   */
  float speedometer;

  /**
   * @brief heading | yaw | bearing | direction
   */
  float compass;

  /**
   * @brief time
   */
  float clock;

} InstrumentReading;

#define INIT_INSTRUMENT_READING {0, 0, 0}

#endif