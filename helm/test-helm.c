#if (!defined(M_PI))
#define M_PI 3.141592653589793238462643383279502884
#endif

#include "helm.h"

#include "testutil-fake-vehicle-update.h"

#include <assert.h>
#include <math.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>

bool ApproxEqual(float a, float b, float epsilon) {
  return fabs(a - b) < epsilon;
}

#define EPSILON 0.0001

// common values
#define STOPPED 0
#define SLOW_FWD 15
#define CRUISING 55
#define SPEEDING 65
#define REVERSE -15
#define V_TOLERANCE 1
#define V_TOLERANCE_STOPPED 0.1
#define V_PCONTROL 2
#define V_PCONTROL_STOPPED 5
#define EAST 0
#define NORTH M_PI / 2.0
#define WEST M_PI
#define SOUTH -M_PI / 2.0
#define H_TOLERANCE M_PI / 16.0
#define H_PCONTROL 2 * H_TOLERANCE

#define TARGET_CRUISING {CRUISING, V_TOLERANCE, V_PCONTROL}
#define TARGET_SLOW_FWD {SLOW_FWD, V_TOLERANCE, V_PCONTROL}
#define TARGET_STOPPED {STOPPED, V_TOLERANCE_STOPPED, V_PCONTROL}
#define TARGET_SPEEDING {SPEEDING, V_TOLERANCE, V_PCONTROL}
#define TARGET_REVERSE {REVERSE, V_TOLERANCE, V_PCONTROL}
#define TARGET_NORTH {NORTH, H_TOLERANCE, H_PCONTROL}
#define TARGET_EAST {EAST, H_TOLERANCE, H_PCONTROL}
#define TARGET_SOUTH {SOUTH, H_TOLERANCE, H_PCONTROL}
#define TARGET_WEST {WEST, H_TOLERANCE, H_PCONTROL}

#define T_0 0
#define NO_HOLD 0
#define SHORT_HOLD 0.1
#define SOME_HOLD 3
#define LONG_HOLD 10

void test_heading() {
  // equal
  assert(ApproxEqual(GetHeadingDelta(M_PI, M_PI), 0.0, EPSILON));

  // equal, but sign flipped
  assert(ApproxEqual(GetHeadingDelta(M_PI, -M_PI), 0.0, EPSILON));

  // quarter circle delta
  assert(ApproxEqual(GetHeadingDelta(0.0, M_PI / 2), M_PI / 2, EPSILON));
  assert(ApproxEqual(GetHeadingDelta(M_PI / 2, 0.0), -M_PI / 2, EPSILON));
  assert(ApproxEqual(GetHeadingDelta(0.0, -M_PI / 2), -M_PI / 2, EPSILON));

  // clockwise half-circle from non-zero location
  assert(ApproxEqual(GetHeadingDelta(-3 * M_PI / 4, M_PI / 4), -M_PI, EPSILON));

  // overflow past M_PI | -M_PI
  assert(ApproxEqual(GetHeadingDelta(3 * -M_PI, -M_PI), 0.0, EPSILON));
  assert(ApproxEqual(GetHeadingDelta(0.0, 5 * M_PI / 2), M_PI / 2, EPSILON));
};

// sample readings we will use them a lot
const InstrumentReading STOPPED_EAST = {STOPPED, EAST, T_0};
const InstrumentReading CRUISING_EAST = {CRUISING, EAST, T_0};
const InstrumentReading REVERSE_EAST = {REVERSE, EAST, T_0};

const InstrumentReading STOPPED_NORTH = {STOPPED, NORTH, T_0};
const InstrumentReading CRUISING_NORTH = {CRUISING, NORTH, T_0};
const InstrumentReading SPEEDING_NORTH = {SPEEDING, NORTH, T_0};

const InstrumentReading STOPPED_SOUTH = {STOPPED, SOUTH, T_0};
const InstrumentReading CRUISING_SOUTH = {CRUISING, SOUTH, T_0};

const Goal GOAL_STOPPED_EAST = {
    TARGET_STOPPED,
    TARGET_EAST,
    NO_HOLD,
};
const Goal GOAL_STOPPED_NORTH = {
    TARGET_STOPPED,
    TARGET_NORTH,
    NO_HOLD,
};
const Goal GOAL_CRUISING_EAST = {
    TARGET_CRUISING,
    TARGET_EAST,
    NO_HOLD,
};
const Goal GOAL_CRUISING_NORTH = {
    TARGET_CRUISING,
    TARGET_NORTH,
    NO_HOLD,
};
const Goal GOAL_CRUISING_WEST = {
    TARGET_CRUISING,
    TARGET_WEST,
    NO_HOLD,
};
const Goal GOAL_CRUISING_SOUTH = {
    TARGET_CRUISING,
    TARGET_SOUTH,
    NO_HOLD,
};
const Goal GOAL_REVERSING_EAST = {
    TARGET_REVERSE,
    TARGET_EAST,
    NO_HOLD,
};
const Goal GOAL_REVERSING_WEST = {
    TARGET_REVERSE,
    TARGET_WEST,
    NO_HOLD,
};
const Goal GOAL_SPEEDING_NORTH = {
    TARGET_SPEEDING,
    TARGET_NORTH,
    NO_HOLD,
};

void test_is_goal_reached() {
  assert(true);
  const Goal goal = {
      TARGET_SPEEDING,
      TARGET_NORTH,
      SOME_HOLD,
  };
  TargetHoldDuration holdDuration = {0, 0};

  InstrumentReading reading = {STOPPED, EAST, T_0};
  bool goalReached = IsGoalReached(&goal, &reading, &holdDuration);
  assert(!goalReached);

  reading.speedometer = SPEEDING;
  reading.compass = NORTH;
  reading.clock = T_0;
  goalReached = IsGoalReached(&goal, &reading, &holdDuration);
  assert(!goalReached);

  reading.clock = T_0 + SOME_HOLD - 1;
  goalReached = IsGoalReached(&goal, &reading, &holdDuration);
  assert(!goalReached);

  reading.clock = T_0 + SOME_HOLD + 1;
  goalReached = IsGoalReached(&goal, &reading, &holdDuration);
  assert(goalReached);
}

void test_series_of_goals() {
  // construct helm with series of goals, check that goals are
  // reached

  const float DT = 0.001;
  const float TIMEOUT = 100;
  float elapsedTime = 0;

  // setup an arbitrary series of goals
  const Goal GOALS[] = {
      GOAL_CRUISING_EAST, GOAL_REVERSING_EAST, GOAL_CRUISING_NORTH,
      GOAL_STOPPED_NORTH, GOAL_REVERSING_WEST, GOAL_CRUISING_SOUTH,
      GOAL_CRUISING_EAST, GOAL_STOPPED_EAST,
  };
  const uint8_t NUM_GOALS = sizeof(GOALS) / sizeof(Goal);

  InstrumentReading reading = STOPPED_EAST;

  Helm helm;
  helmReset(&helm);

  // while there are goals to reach and time left on the clock
  uint8_t goalIdx = 0;
  while (elapsedTime <= TIMEOUT) {
    reading.clock = elapsedTime;
    Action action;
    bool isGoalReached = helmUpdate(&helm, &reading, &GOALS[goalIdx], &action);

    if (isGoalReached) {
      if (goalIdx < NUM_GOALS) {
        // assert that all the goals in the sequence are reached
        assert(IsGoalReached(&GOALS[goalIdx], &reading, &helm.holdDuration));
      }
      goalIdx++;
      helmReset(&helm);
    }

    FakeVehicleUpdate(&action, DT, &reading);
    elapsedTime += DT;

    if (goalIdx == NUM_GOALS) {
      break;
    }
  }

  // printf("FINAL clock %f\n", reading.clock);
  // printf("  speedometer %f\n", reading.speedometer);
  // printf("  compass %f\n", reading.compass);
  // fflush(stdout);
  assert(elapsedTime <= TIMEOUT);
}

int main() {
  test_heading();
  test_is_goal_reached();
  test_series_of_goals();
  return 0;
}
