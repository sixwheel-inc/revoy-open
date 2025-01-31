#include "external/catch2/extras/catch_amalgamated.hpp"

#include "c_caller_path/remote-player.h"

#include <iostream>
#include <stdint.h>
#include <string>

TEST_CASE("test playing the whole script of button presses") {

  RemoteScript script = {5,
                         {RC_TOGGLE_REMOTE, RC_STICK_UP, RC_STICK_DOWN,
                          RC_STICK_LEFT, RC_STICK_RIGHT}};

  RemotePlayer player;
  float clock = 0;
  remotePlayerReset(&player, clock);

  bool done = false;
  while (clock < 10) {
    RemoteAction action = RC_NO_ACTION;
    done = remotePlayerUpdate(&player, &script, clock, &action);

    if (!done) {
      CHECK(action != RC_NO_ACTION);
    }
    clock += 0.004;
  }

  CHECK(done);
  CHECK(player.idx == script.size);
}
