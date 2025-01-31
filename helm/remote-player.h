#ifndef _HELM_REMOTE_PLAYER_H_
#define _HELM_REMOTE_PLAYER_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "remote-script.h"

#include <stdbool.h>
#include <stdint.h>

/// @brief Maintains the state of playing a script of RemoteActions
typedef struct _RemotePlayer {
  /// @brief index in the Script being played
  uint8_t idx;
  float actionStartTime;
} RemotePlayer;

void remotePlayerReset(RemotePlayer *player, float clock);

/// @brief indexes into the script, updates position in script
/// return true when script is complete and nothing remains to play
bool remotePlayerUpdate(RemotePlayer *player, const RemoteScript *script,
                        float clock, RemoteAction *action);

#ifdef __cplusplus
} /* extern "C" */
#endif

#endif
