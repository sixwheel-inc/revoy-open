#ifndef _HELM_H_
#define _HELM_H_

#if (!defined(M_PI))
#define M_PI 3.141592653589793238462643383279502884
#endif

#ifdef __cplusplus
extern "C" {
#endif

#include "action.h"
#include "goal.h"
#include "instrument-reading.h"

/**
 * @brief keeps track of the current scenario, as well as our place in it
 */
typedef struct _Helm {

  /**
   * @brief informs how long the current targets should be maintained
   */
  TargetHoldDuration holdDuration;

} Helm;

/// INSTANCE METHODS OF HELM

/**
 * @bref this is a "constructor" for "class Helm".
 */
void helmReset(Helm *helm);

/**
 * @brief use these Actions to operate the plant
 */
bool helmUpdate(Helm *helm, const InstrumentReading *reading, const Goal *goal,
                Action *action);

/**
 * @brief use this to know when its done
 * (no more Actions will come from helmUpdate)
 */
bool helmIsScenarioComplete(const Helm *helm);

/// STATIC FUNCTIONS

/**
 * @brief delta = beta - alpha
 *        alpha + delta = beta
 */
float GetHeadingDelta(float alpha, float beta);

/**
 * @brief true if helm has reached the Targets and held them for specified
 * duration
 */
bool IsGoalReached(const Goal *goal, const InstrumentReading *reading,
                   TargetHoldDuration *holdDuration);

/**
 * @brief Produces an Action (i.e. throttle, brake, turn) such that, when
 * applied to some pre-supposed compatible vehicle model, should in theory
 * adjust the reading towards the goal.
 */
Action CalculateAction(const Goal *goal, const InstrumentReading *reading);

#ifdef __cplusplus
} // extern "C"
#endif

#endif
