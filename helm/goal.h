#ifndef _HELM_C_GOAL_H_
#define _HELM_C_GOAL_H_

#if (!defined(M_PI))
#define M_PI 3.141592653589793238462643383279502884
#endif

#include <stdbool.h>
#include <stdint.h>

typedef struct _Target {
  float target;
  float tolerance;
  float pControl;
} Target;

/**
 * @brief represents a human-driver's desired state, e.g. moving at constant
 * speed
 */
typedef struct _Goal {

  /**
   * @brief try to accelerate or deccelerate the vehicle towards this velocity.
   *
   */
  Target velocity;

  /**
   * @brief try to steer the vehicle towards this heading.
   *
   */
  Target heading;

  /**
   * @brief defines how long the Targets must be maintained
   */
  float duration;

} Goal;

/*
 * @brief if set, the current Goal's Targets will be held until time, then the
 * current Goal will be reached.
 */
typedef struct _TargetHoldDuration {
  bool hasReachedTarget;
  float expiryTime;
} TargetHoldDuration;

#endif
