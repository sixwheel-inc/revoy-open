# Zaxis

Top-level of the 3D Simulator, combines and coordinates between:

- Assembly
  - wrapper around projectchrono wheeled-vehicles and wheeled-trailers
- Helm
  - drives the simulated tractor, follows a series of goals
- Long Control
  - simulink ERT codegen not included in this repo
- Lat Control
  - simulink ERT codegen not included in this repo
- Mcap (third-party)
  - fast writing of protobuf messages to disk
- Irrlicht (third-party)
  - comes w/ projectchrono, live view of the 3D Simulation, useful for debugging

## General Usage

```bash
bazel run //zaxis:run-zaxis -- <assembly-type> <scenario-type> [scenario-params ...]
```

See [test script](../ci-runner/chrono-test.sh) for examples.

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

### Long Controls / Lat Controls (not included in this repo)

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

