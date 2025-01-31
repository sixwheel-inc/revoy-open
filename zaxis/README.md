# Zaxis

Top-level of the 3D Simulator, combines and coordinates between:

- Assembly (projectchrono physics)
- Helm (shared w/ HIL and SIL)
- Long Control (Simulink ERT codegen)
- Lat Control (Simulink ERT codegen)
- Mcap
- Irrlicht

## [How to Use w/ CI](https://docs.google.com/document/d/1qtSKEpkkB9xzdp8gaG1QNkkXwBfX1RkTjxqT5dwruh0/edit?tab=t.0#heading=h.go681w4epabz)

## General Usage

```bash
bazel run //zaxis:run-zaxis -- <assembly-type> <scenario-type> [scenario-params ...]
```

See [test script](../ci-runner/chrono-test.sh) for examples.

## Note about codegen, logging, and replay

This project relies on two sources of code generation:
- Simulink ERT generates the C code for Controls
  - This is how we run Controls alongside the rest of the simulation, in CI, w/o needing any licenses
  - Otherwise we would have to run everything else inside Simulink, or use TCP, or something
- //dense_log_codegen generates the protobuf for Controls IO
  - This is used to fill an Mcap with all the IO during the scenario
    - the Mcap can be viewed in Foxglove
    - the Mcap can be used to replay what happened, in Simulink (see control-design/IsolatedControlsReplay.slx)

## arch

![zaxis _ assembly - Frame 1](https://github.com/user-attachments/assets/0b7b7b11-d7e9-4877-8036-8141c41d9399)

## major component inputs / outputs

### helm (shared with SIL and HIL)

Tells the tractor model what to do.

#### Input: Instrument Reading
- current tractor speed
- current tractor heading
- current time

#### Input: Goal
- target speed
- target heading
- hold duration

#### Output: Action
- tractor throttle intensity
- tractor steer intensity
- tractor brake intensity
- tractor forward or reverse

### assembly

Wrapper for projectchrono wheeled-vehicle models, maintains and updates the physics / mechanics of the vehicles, suspension, tires-road, etc.

#### Input: Tractor Driver Input
- tractor throttle intensity
- tractor steer intensity
- tractor brake intensity
- tractor forward or reverse

#### Input: Revoy Driver Input
- revoy demand torque
- revoy steer angle command

#### Output: Model State
- current speed, angles, forces, XYZ, for all vehicle components

### Long Controls / Lat Controls

Wrapper around Simulink ERT codegen Isolated Controls, tells the RevoyEV what to do.

#### Input: Revoy Sensors
- revoy-tractor hitch force
- revoy-tractor hitch angle
- revoy wheel speed

#### Output: Revoy Actions
- demand torque
- set steer angle

### Zaxis

Top-level runner, runs a Scenario using the above componets.

#### Input: Scenario
- list of tractor goals (will feed the helm)

#### Input: AssemblyType
- set which assembly to use, from available

#### Output: Result
- is timeout (i.e. no timeout or other issue)
- max rollover angle(s)
- max hitch angle(s)
- max speed
- etc

