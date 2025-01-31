#include "remote-player.h"
#include "remote-script.h"

static const float BUTTON_PRESS_DURATION = 0.2;
static const float STICK_PRESS_DURATION = 1;

void remotePlayerReset(RemotePlayer *player, float clock) {
  player->idx = 0;
  player->actionStartTime = clock;
}

bool remotePlayerUpdate(RemotePlayer *player, const RemoteScript *script,
                        float clock, RemoteAction *action) {

  *action = RC_NO_ACTION;

  if (script->size == 0 || player->idx >= script->size) {
    return true;
  }

  *action = script->rcActions[player->idx];

  /// generally the stick is held in place for longer, buttons are tapped
  /// quickly
  const bool isStick = (*action == RC_STICK_DOWN || *action == RC_STICK_UP ||
                        *action == RC_STICK_LEFT || *action == RC_STICK_RIGHT);

  const float duration =
      (isStick) ? STICK_PRESS_DURATION : BUTTON_PRESS_DURATION;

  if (clock - player->actionStartTime >= duration) {
    player->idx++;
    player->actionStartTime = clock;
  }

  if (player->idx >= script->size) {
    return true;
  }

  return false;
}
