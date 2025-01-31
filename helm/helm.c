// M_PI is not defined on all systems
#if (!defined(M_PI))
#define M_PI 3.141592653589793238462643383279502884
#endif

#include "helm.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <string.h>

float GetHeadingDelta(float alpha, float beta) {
  static const float TWO_PI = 2.0 * M_PI;
  float delta = fmod(beta - alpha, TWO_PI);
  if (delta > M_PI) {
    delta -= TWO_PI;
  } else if (delta < -M_PI) {
    delta += TWO_PI;
  }
  return delta;
};

Dir getDir(float val, float tol) {
  if (val > tol) {
    return FWD;
  } else if (val < -tol) {
    return REV;
  } else {
    return STOP;
  }
}

bool AreTargetsReached(const Goal *goal, const InstrumentReading *reading) {

  const float deltaVelocity = goal->velocity.target - reading->speedometer;
  const Dir deltaDir = getDir(deltaVelocity, goal->velocity.tolerance);
  if (deltaDir != STOP) {
    return false;
  }
  const float deltaHeading =
      GetHeadingDelta(goal->heading.target, reading->compass);
  if (fabs(deltaHeading) > goal->heading.tolerance) {
    return false;
  }
  return true;
}

void LongAction(const Target *velocity, float speedometer, Action *action) {

  assert(action->brake == 0);
  assert(action->throttle == 0);

  const float delta = velocity->target - speedometer;

  const Dir targetDir = getDir(velocity->target, velocity->tolerance);
  const Dir motionDir = getDir(speedometer, velocity->tolerance);
  const Dir deltaDir = getDir(delta, velocity->tolerance);

  if (motionDir == STOP) {
    // if we aren't moving, decide which direction based on target
    action->direction = targetDir;
  } else {
    // when moving, action has to be in direction of motion
    action->direction = motionDir;
  }

  if (action->direction == STOP) {
    // if motion is stopped and goal is stopped, then we are at goal, do nothing
    return;
  }

  // human drivers brake when:
  //   - they want to slow down a lot (outside window)
  //   - they want to switch to/from forward/reverse
  //     - (they would need to stop before switching)
  // otherwise throttle.
  const bool isStop =
      motionDir != STOP && (motionDir != targetDir || deltaDir != targetDir);

  if (isStop) {
    // the desired action is to brake, determine intensity
    if (targetDir == STOP || motionDir != targetDir) {
      // when stopping (to 0 speed) or changing between fwd and reverse, brake
      // fully
      action->brake = 1;
    } else if (fabs(delta) > velocity->pControl) {
      // when adjusting speed, light brake
      action->brake = 0.1;
    }
  } else {
    action->throttle = 1;
  }
}

void LatAction(const Goal *goal, const InstrumentReading *reading,
               Action *action) {

  assert(action->turn == 0);

  const float delta = GetHeadingDelta(goal->heading.target, reading->compass);
  const bool isAtGoal = fabs(delta) < goal->heading.tolerance;
  if (isAtGoal) {
    return;
  }
  // cannot possibly reach LatGoal if action is to be stopped
  // please re-formulate the scenario so that desired turn can be completed
  // before vehicle is stopped
  assert(action->direction != STOP);

  const bool needLeft = (action->direction == FWD) == delta < 0;
  action->turn = (needLeft) ? 1 : -1;
}

float pControl(float delta, float regime) {
  return fmax(0.0, fmin(1.0, delta / regime));
}

Action CalculateAction(const Goal *goal, const InstrumentReading *reading) {

  Action action = INIT_ACTION;

  LongAction(&goal->velocity, reading->speedometer, &action);
  LatAction(goal, reading, &action);

  // applying p-control when braking close to stopped, allows us to "settle" at
  // 0.
  if (getDir(goal->velocity.target, goal->velocity.tolerance) == STOP) {
    const float control =
        pControl(fabs(reading->speedometer), goal->velocity.pControl);
    action.brake *= control;
  }

  // applying p-control to top-speed, so that we ease off throttle as we get
  // close.
  if (getDir(goal->velocity.target, goal->velocity.tolerance) != STOP) {
    if (getDir(goal->velocity.target, goal->velocity.tolerance) == FWD) {
      const float control =
          pControl(goal->velocity.target - reading->speedometer,
                   goal->velocity.pControl);
      action.throttle *= control;
    } else {
      const float control =
          pControl(goal->velocity.target - reading->speedometer,
                   -goal->velocity.pControl);
      action.throttle *= control;
    }
  }

  // appling p-control to steering, so that steering_angle reaches 0 when we
  // reach target heading.
  {
    const float control =
        pControl(fabs(GetHeadingDelta(goal->heading.target, reading->compass)),
                 goal->heading.pControl);
    action.turn *= control;
  }

  assert(action.throttle >= 0);
  assert(action.throttle <= 1);
  assert(action.brake >= 0);
  assert(action.brake <= 1);
  assert(action.turn >= -1);
  assert(action.turn <= 1);

  if (action.direction == FWD) {
    assert(reading->speedometer > -goal->velocity.tolerance);
  } else if (action.direction == REV) {
    assert(reading->speedometer < goal->velocity.tolerance);
  }

  return action;
}

void helmReset(Helm *helm) {
  memset(&helm->holdDuration, 0, sizeof(TargetHoldDuration));
}

bool IsGoalReached(const Goal *goal, const InstrumentReading *reading,
                   TargetHoldDuration *holdDuration) {

  if (holdDuration->hasReachedTarget) {
    assert(holdDuration->expiryTime > 0);
    if (reading->clock >= holdDuration->expiryTime) {
      // printf("hold done: %f\n", holdDuration->expiryTime);
      // fflush(stdout);
      holdDuration->hasReachedTarget = false;
      holdDuration->expiryTime = 0;
      return true;
    }
    return false;
  }

  assert(!holdDuration->hasReachedTarget);
  assert(holdDuration->expiryTime == 0);

  if (AreTargetsReached(goal, reading)) {
    if (goal->duration == 0) {
      return true;
    }
    holdDuration->hasReachedTarget = true;
    holdDuration->expiryTime = reading->clock + goal->duration;
    // printf("hold until: %f\n", holdDuration->expiryTime);
    // fflush(stdout);
  }
  return false;
}

bool helmUpdate(Helm *helm, const InstrumentReading *reading, const Goal *goal,
                Action *action) {

  if (IsGoalReached(goal, reading, &helm->holdDuration)) {
    // if goal is reached, no action to take
    return true;
  }

  // otherwise compute action normally
  *action = CalculateAction(goal, reading);
  return false;
}
