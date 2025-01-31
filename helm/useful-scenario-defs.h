#ifndef _USEFUL_SCENARIO_DEFS_H_
#define _USEFUL_SCENARIO_DEFS_H_

// C89 compatible static assert as pre-processor macro:
// if condition is false, generates uncompilabe -1 index
#define STATIC_ASSERT(condition)                                               \
  typedef char static_assertion[(condition) ? 1 : -1]

#include <stdbool.h>
#include <stdint.h>

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

#define NO_FAULTS {false, false, 0}
#define IS_ESTOP {true, false, 0}
#define IS_ESTOP_REMOTE {false, true, 0}
#define MCU_FAULT_THREE {false, false, 3}

#define T_0 0
#define NO_HOLD 0
#define SHORT_HOLD 0.1
#define SOME_HOLD 3
#define LONG_HOLD 10

#define NO_RC_CONTROL                                                          \
  {                                                                            \
    0, {}                                                                      \
  }

/// When enabling RC Mode, it is the user's responsibility to
/// - enable High Voltage
/// - disable EPB (it is automatically enabled when switching to Remote)
///   - that's why we need to wait a bit after switching to remote, because
///     if we try to disable the EPB as soon as we switch, we will be overridden
/// - put revoy in Drive
/// After this, the revoy will be controllable via throttle sticks / steer
/// sticks
#define RC_ENABLE                                                              \
  {                                                                            \
    6, {                                                                       \
      RC_TOGGLE_REMOTE, RC_NO_ACTION, RC_HV_TOGGLE, RC_EPB_TOGGLE,             \
          RC_BUTTON_KICKSTAND_DEPLOY, RC_DRIVE_MODE_D                          \
    }                                                                          \
  }

/// When turning off remote, all control goes back to revoy, so there is no need
/// to set HV, put in drive, etc.
#define RC_DISABLE                                                             \
  {                                                                            \
    4, {                                                                       \
      RC_BUTTON_KICKSTAND_STOW, RC_EPB_TOGGLE, RC_HV_TOGGLE, RC_TOGGLE_REMOTE  \
    }                                                                          \
  }

/// When turning off remote, all control goes back to revoy, so there is no need
/// to set HV, put in drive, etc.
#define RC_DISABLE_LEAVE_KICKSTAND_DEPLOYED                                    \
  {                                                                            \
    3, { RC_EPB_TOGGLE, RC_HV_TOGGLE, RC_TOGGLE_REMOTE }                       \
  }

#define RC_CONTROL_BASIC_SCRIPT                                                \
  {                                                                            \
    16, {                                                                      \
      RC_STICK_UP, RC_STICK_DOWN, RC_STICK_LEFT, RC_STICK_RIGHT, RC_STICK_UP,  \
          RC_STICK_DOWN, RC_STICK_LEFT, RC_STICK_RIGHT, RC_STICK_UP,           \
          RC_STICK_DOWN, RC_STICK_LEFT, RC_STICK_RIGHT, RC_STICK_UP,           \
          RC_STICK_DOWN, RC_STICK_LEFT, RC_STICK_RIGHT,                        \
    }                                                                          \
  }

#define RC_DO_DEPLOY_KICKSTAND                                                 \
  {                                                                            \
    1, { RC_BUTTON_KICKSTAND_DEPLOY }                                          \
  }

#define RC_DO_STOW_KICKSTAND                                                   \
  {                                                                            \
    1, { RC_BUTTON_KICKSTAND_STOW }                                            \
  }

#define RC_CONTROL_HOLD_FORWARD                                                \
  {                                                                            \
    16, {                                                                      \
      RC_STICK_UP, RC_STICK_UP, RC_STICK_UP, RC_STICK_UP, RC_STICK_UP,         \
          RC_STICK_UP, RC_STICK_UP, RC_STICK_UP, RC_STICK_UP, RC_STICK_UP,     \
          RC_STICK_UP, RC_STICK_UP, RC_STICK_UP, RC_STICK_UP, RC_STICK_UP,     \
          RC_STICK_UP                                                          \
    }                                                                          \
  }

#define PLANNING_NO_OBSTACLE                                                   \
  {}

#endif // _USEFUL_SCENARIO_DEFS_H_
