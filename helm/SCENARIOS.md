# Scenarios

Used to define and run deterministic end-to-end behavior tests in simulation.

## Purpose

### ensure controls software runs the codepaths it should w/o assertion, segfault, inf, nan, etc

  - examples:
    - validate it runs forward/reverse, throttling/braking, steering and in combination
    - error paths, e-stop, fault levels, result in correct degraded behaviors
    - cycle through various operation modes e.g. remote, assist, charging, etc

### validate coarse behavior i.e. on the order of ~1s to ~60s (not 4ms)

  - examples:
    - should push when tractor is accelerating
    - should regenerate when tractor is braking
    - should (or should not?) push tractor during cruising
    - should (or should not?) lock steer during reverse

- should not need to re-tune variables each time ECU.slx changes
  - simple definitions for scenarios, approximately something like
        
    ```toml
    Scenario: Emergency Stop
    - Accel up to 60mph
    - Trigger estop and hold for 5s
    - Clear estop
        
    Scenario: Reverse and Steer
    - Accel in reverse up to 15mph
    - Steer until facing South
    - Stop
    ```

## Components

## Scenario

Represents a single end-to-end behavior test of the Sixwheel.

Scenario is a sequence of Stages (i.e. "desired states").
The simulated Sixwheel will be driven through these Stages in order.

## Stage

The following fields define each Stage, they must be met in order to move onto the next Stage.

### Goal

Used to drive the Helm, kinamatic goals that the Tractor must achieve.

Defined by desired speed, heading, and hold duration.

### RemoteScript

Used to trigger Remote Controller signals.

Defined as a sequence of RemoteActions (stick up, button press, etc).

### Faults

Used to trigger Faults / DTCs, directly passed to the Plant.


## ScenarioRunner

Runs the Scenario, ensuring each Stage is satisfied before moving onto the next.
- Uses the Helm to manuever the simulated Tractor.
- Uses the RemotePlayer to send RC controls to the simulated Sixwheel.
- Passes Faults (e.g. mcu fault, estop) to the Plant.
- (TODO) Passes PlantModes (e.g. charge gun unplugged, keyon off) to the Plant.

## Helm

Represents the "Driver's Seat" of the Tractor, it is responsible for high-level maneuvering and decision making of the Assembled Vehicle.

### Helm Inputs

- Goal (speed, heading, duration)
- InstrumentReading (speedometer, compass, clock)

### Helm Outputs

- Action (throttle, brake, steer, forward/reverse)

## RemotePlayer

Represents the "Attendant", responsible for sending RC Commands to the Sixwheel.

### RemotePlayer Input

- RemoteScript (sequence of RemoteActions)
- clock

### RemotePlayer Output

- RemoteAction (stick up, button press, etc)

