#include "debounce_toggle.h"
#include "external/catch2/extras/catch_amalgamated.hpp"
#include "remote_control.h"
#include <iostream>

TEST_CASE("test translation logic") {
  ControllerState state;
  state.throttleStick = 48;
  state.steeringStick = 66;
  state.kickstandButton = true;
  state.driveModeNButton = false;
  state.driveModeDButton = false;
  state.sbL1 = true;
  state.sbL2 = false;
  state.sbR1 = false;
  state.sbR2 = true;
  state.driveModePButton = true; // controls extLockDifferential
  state.suspensionButton = true; // controls HV
  state.lockButton = true;       // controls remote

  ControllerParameters params;
  params.maxVelocity = 99.0f;
  params.maxSteeringWheelAngle = 400.0f;

  // run once, debounce prevents transition
  ExternalCommands commands =
      create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

  CHECK(commands.extVelocity == state.throttleStick /
                                    ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                    params.maxVelocity);
  CHECK(commands.extSteeringWheelAngle ==
        -state.steeringStick / (float)INT8_MAX * params.maxSteeringWheelAngle);

  CHECK(!commands.extRemoteControlEnable);
  CHECK(commands.extEPBEnable);
  CHECK(commands.extEPBCommand == 0);
  CHECK(!commands.extHVEnable);
  CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

  CHECK(commands.extSuspensionMotion == 0);
  CHECK(commands.extKickstandMotion == -1);

  CHECK(commands.extLockDifferential);

  // run up to debounce limit, still no transition
  for (int i = 0; i < DEBOUNCE_THRESHOLD - 1; i++) {
    commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

    CHECK(commands.extVelocity == state.throttleStick /
                                      ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                      params.maxVelocity);
    CHECK(commands.extSteeringWheelAngle == -state.steeringStick /
                                                (float)INT8_MAX *
                                                params.maxSteeringWheelAngle);

    CHECK(!commands.extRemoteControlEnable);
    CHECK(commands.extEPBEnable);
    CHECK(commands.extEPBCommand == 0);
    CHECK(!commands.extHVEnable);
    CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

    CHECK(commands.extSuspensionMotion == 0);
    CHECK(commands.extKickstandMotion == -1);

    CHECK(commands.extLockDifferential);
  }

  // run once more, debounce allows transition
  commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

  CHECK(commands.extVelocity == state.throttleStick /
                                    ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                    params.maxVelocity);
  CHECK(commands.extSteeringWheelAngle ==
        -state.steeringStick / (float)INT8_MAX * params.maxSteeringWheelAngle);

  CHECK(commands.extRemoteControlEnable);
  CHECK(!commands.extEPBEnable);
  CHECK(commands.extEPBCommand == 2);
  CHECK(commands.extHVEnable);
  CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

  CHECK(commands.extSuspensionMotion == 0);
  CHECK(commands.extKickstandMotion == -1);

  CHECK(commands.extLockDifferential);

  // run up to and through debounce limit with input flipped, no transition
  state.kickstandButton = false;
  state.driveModeNButton = false;
  state.driveModeDButton = false;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  for (int i = 0; i < 2 * DEBOUNCE_THRESHOLD; i++) {
    commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

    CHECK(commands.extVelocity == state.throttleStick /
                                      ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                      params.maxVelocity);
    CHECK(commands.extSteeringWheelAngle == -state.steeringStick /
                                                (float)INT8_MAX *
                                                params.maxSteeringWheelAngle);

    CHECK(commands.extRemoteControlEnable);
    CHECK(!commands.extEPBEnable);
    CHECK(commands.extEPBCommand == 0);
    CHECK(commands.extHVEnable);
    CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

    CHECK(commands.extSuspensionMotion == 0);
    CHECK(commands.extKickstandMotion == -1);

    CHECK(!commands.extLockDifferential);
  }
  // change gear to drive
  state.kickstandButton = false;
  state.driveModeNButton = false;
  state.driveModeDButton = true;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

  CHECK(commands.extVelocity == state.throttleStick /
                                    ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                    params.maxVelocity);
  CHECK(commands.extSteeringWheelAngle ==
        -state.steeringStick / (float)INT8_MAX * params.maxSteeringWheelAngle);

  CHECK(commands.extRemoteControlEnable);
  CHECK(!commands.extEPBEnable);
  CHECK(commands.extEPBCommand == 0);
  CHECK(commands.extHVEnable);
  CHECK(commands.extGearState == GEAR_STATE_DRIVE);

  CHECK(commands.extSuspensionMotion == 0);
  CHECK(commands.extKickstandMotion == -1);

  CHECK(!commands.extLockDifferential);

  // make sure gear stays in drive when button is released
  state.kickstandButton = false;
  state.driveModeNButton = false;
  state.driveModeDButton = false;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

  CHECK(commands.extRemoteControlEnable);
  CHECK(!commands.extEPBEnable);
  CHECK(commands.extEPBCommand == 0);
  CHECK(commands.extHVEnable);
  CHECK(commands.extGearState == GEAR_STATE_DRIVE);

  CHECK(commands.extSuspensionMotion == 0);
  CHECK(commands.extKickstandMotion == -1);

  CHECK(!commands.extLockDifferential);

  // change gear to neutral with neutral button
  state.kickstandButton = false;
  state.driveModeNButton = true;
  state.driveModeDButton = false;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

  CHECK(commands.extRemoteControlEnable);
  CHECK(!commands.extEPBEnable);
  CHECK(commands.extEPBCommand == 0);
  CHECK(commands.extHVEnable);
  CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

  CHECK(commands.extSuspensionMotion == 0);
  CHECK(commands.extKickstandMotion == -1);

  CHECK(!commands.extLockDifferential);

  // change gear back to drive to test going to neutral with P
  state.kickstandButton = false;
  state.driveModeNButton = false;
  state.driveModeDButton = true;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

  CHECK(commands.extRemoteControlEnable);
  CHECK(!commands.extEPBEnable);
  CHECK(commands.extEPBCommand == 0);
  CHECK(commands.extHVEnable);
  CHECK(commands.extGearState == GEAR_STATE_DRIVE);

  CHECK(commands.extSuspensionMotion == 0);
  CHECK(commands.extKickstandMotion == -1);

  CHECK(!commands.extLockDifferential);

  // with parking brake button on run until the debounce limit
  state.kickstandButton = true;
  state.driveModeNButton = false;
  state.driveModeDButton = false;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  for (int i = 0; i < DEBOUNCE_THRESHOLD; i++) {
    commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

    CHECK(commands.extVelocity == state.throttleStick /
                                      ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                      params.maxVelocity);
    CHECK(commands.extSteeringWheelAngle == -state.steeringStick /
                                                (float)INT8_MAX *
                                                params.maxSteeringWheelAngle);

    CHECK(commands.extRemoteControlEnable);
    CHECK(!commands.extEPBEnable);
    CHECK(commands.extEPBCommand == 0);
    CHECK(commands.extHVEnable);
    CHECK(commands.extGearState == GEAR_STATE_DRIVE);

    CHECK(commands.extSuspensionMotion == 0);
    CHECK(commands.extKickstandMotion == -1);

    CHECK(!commands.extLockDifferential);
  }

  // run one more time and see that parking brake turns on and gear changes
  state.kickstandButton = true;
  state.driveModeNButton = false;
  state.driveModeDButton = false;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);
  CHECK(commands.extVelocity == state.throttleStick /
                                    ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                    params.maxVelocity);
  CHECK(commands.extSteeringWheelAngle ==
        -state.steeringStick / (float)INT8_MAX * params.maxSteeringWheelAngle);

  CHECK(commands.extRemoteControlEnable);
  CHECK(commands.extEPBEnable);
  CHECK(commands.extEPBCommand == 1);
  CHECK(commands.extHVEnable);
  CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

  CHECK(commands.extSuspensionMotion == 0);
  CHECK(commands.extKickstandMotion == -1);

  CHECK(!commands.extLockDifferential);

  // turn parking brake on again
  state.kickstandButton = false;
  state.driveModeNButton = false;
  state.driveModeDButton = false;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  for (int i = 0; i < 2 * DEBOUNCE_THRESHOLD; i++) {
    commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);
  }
  state.kickstandButton = true;
  state.driveModeNButton = false;
  state.driveModeDButton = false;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  for (int i = 0; i <= DEBOUNCE_THRESHOLD; i++) {
    commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);
  }
  CHECK(commands.extRemoteControlEnable);
  CHECK(!commands.extEPBEnable);
  CHECK(commands.extEPBCommand == 2);
  CHECK(commands.extHVEnable);
  CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

  CHECK(commands.extSuspensionMotion == 0);
  CHECK(commands.extKickstandMotion == -1);

  CHECK(!commands.extLockDifferential);

  // run over debounce limit again with input flipped, no transition
  state.kickstandButton = false;
  state.driveModeNButton = false;
  state.driveModeDButton = false;
  state.driveModePButton = false; // controls extLockDifferential
  state.suspensionButton = false; // controls HV
  state.lockButton = false;       // controls remote
  for (int i = 0; i < 2 * DEBOUNCE_THRESHOLD; i++) {
    commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

    CHECK(commands.extVelocity == state.throttleStick /
                                      ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                      params.maxVelocity);
    CHECK(commands.extSteeringWheelAngle == -state.steeringStick /
                                                (float)INT8_MAX *
                                                params.maxSteeringWheelAngle);

    CHECK(commands.extRemoteControlEnable);
    CHECK(!commands.extEPBEnable);
    CHECK(commands.extEPBCommand == 0);
    CHECK(commands.extHVEnable);
    CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

    CHECK(commands.extSuspensionMotion == 0);
    CHECK(commands.extKickstandMotion == -1);

    CHECK(!commands.extLockDifferential);
  }

  // toggle debounce signals back from good measure
  state.kickstandButton = true;
  state.driveModeNButton = false;
  state.driveModeDButton = false;
  state.driveModePButton = true; // controls extLockDifferential
  state.suspensionButton = true; // controls HV
  state.lockButton = true;       // controls remote
  for (int i = 0; i < DEBOUNCE_THRESHOLD; i++) {
    commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

    CHECK(commands.extVelocity == state.throttleStick /
                                      ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                      params.maxVelocity);
    CHECK(commands.extSteeringWheelAngle == -state.steeringStick /
                                                (float)INT8_MAX *
                                                params.maxSteeringWheelAngle);

    CHECK(commands.extRemoteControlEnable);
    CHECK(!commands.extEPBEnable);
    CHECK(commands.extEPBCommand == 0);
    CHECK(commands.extHVEnable);
    CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

    CHECK(commands.extSuspensionMotion == 0);
    CHECK(commands.extKickstandMotion == -1);

    CHECK(commands.extLockDifferential);
  }

  commands = create_controller_commands(EVIONICS_STATE_ASSIST, state, params);

  CHECK(commands.extVelocity == state.throttleStick /
                                    ((float)INT8_MAX * EXT_VELOCITY_SCALE) *
                                    params.maxVelocity);
  CHECK(commands.extSteeringWheelAngle ==
        -state.steeringStick / (float)INT8_MAX * params.maxSteeringWheelAngle);

  CHECK(!commands.extRemoteControlEnable);
  CHECK(commands.extEPBEnable);
  CHECK(commands.extEPBCommand == 1);
  CHECK(!commands.extHVEnable);
  CHECK(commands.extGearState == GEAR_STATE_NEUTRAL);

  CHECK(commands.extSuspensionMotion == 0);
  CHECK(commands.extKickstandMotion == -1);

  CHECK(commands.extLockDifferential);
}
