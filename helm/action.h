#ifndef _HELM_ACTION_H_
#define _HELM_ACTION_H_

typedef enum _Dir { STOP, FWD, REV } Dir;

/**
 * @brief represents the actions that a human driver can make with a vehicle
 */
typedef struct _Action {

  /**
   * @brief seconds, how long in simulation time to apply this action.neeee
   *
   */
  float duration;

  /**
   * @brief [-1.0, 1.0], the percent of steer angle w.r.t. maximum steer
   * angle i.e. corresponds to (-maxSteerAngle, maxSteerAngle).
   */
  float turn;

  /**
   * @brief [0.0, 1.0], the percent of throttle w.r.t. maximum throttle i.e.
   * corresponds to [0, maxThrottle]
   */
  float throttle;

  /**
   * @brief [0.0, 1.0], the percent of brake w.r.t. maximum brake i.e.
   * corresponds to [0, maxBrakeTorque].
   */
  float brake;

  /**
   * @brief STOP FWD REV
   */
  Dir direction;

} Action;

#define INIT_ACTION {0, 0, 0, 0}

#endif // _HELM_ACTION_H_
