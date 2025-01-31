#ifndef _HELM_REMOTE_SCRIPT_H_
#define _HELM_REMOTE_SCRIPT_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

/**
 * @brief represents an action that a human operator can make w/ the Remote
 * , will hold this action for 1 second
 */
typedef enum _RemoteAction {
  RC_NO_ACTION,
  RC_TOGGLE_REMOTE,
  RC_STICK_UP,
  RC_STICK_DOWN,
  RC_STICK_LEFT,
  RC_STICK_RIGHT,
  RC_BUTTON_KICKSTAND_STOW,
  RC_BUTTON_KICKSTAND_DEPLOY,
  RC_EPB_TOGGLE,
  RC_HV_TOGGLE,
  RC_DRIVE_MODE_D,

  /// TODO: implement this
  RC_SUSPENSION_UP,
  RC_SUSPENSION_DOWN,
} RemoteAction;

#define RC_ACTIONS_MAX 100

/**
 * @brief represents a series of actions that a human operator can make w/ the
 * Remote
 */
typedef struct _RemoteScript {
  uint8_t size;
  RemoteAction rcActions[RC_ACTIONS_MAX];
} RemoteScript;

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
