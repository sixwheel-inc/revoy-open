# Revoy Open

This repo contains FOSS ports of internal Revoy projects, mainly related to Autonomy and Safety.

## Planning (using OMPL)

### Current Goal: improve operator safety by using autonomy to stop for hazards

Operators on-site use an RC Controller to move the Revoy EV around the yard. Since the RevoyEV is so large, it is difficult for the operator to see all around the vehicle.

We can stop the Revoy from colliding with a hazarc, even if the operator is not aware of the hazard (due to line of sight etc).

### Future Goal: fully autonomous yard operations (drop-n-hook, etc)

## 3D Simulation (using projectchrono)

### Current Goal: allow Controls engineers to develop advanced high-speed safety behaviors

Using the 3D Simulation, validate the safety behaviors can prevent:

- loss of traction
- jackknife
- rollover
- crack-the-whip
- etc

### Future Goal: aerodynamics, battery / electric-motor dynamics, complex road-surfaces (gravel etc)

## Acknowledgements

The code in this project is largely the work of Shashwat Kandadai, however all of this work is made possible thanks to the team of people designing, building, testing, maintaining, deploying a fleet of robots, and serving paying customers:

- Bayne Smith
- Beverly Morrison
- Catherine Shepherd
- Ian Rust
- Jason Hinesly
- Letian Lee
- Muhammad Ahsan
- Nemo Jin
- Peter Reinhardt
- Roland Blackwell
- Sami Rajala
- Selina Pan
- Shaochun Ye
- Shashwat Kandadai
- Sina Nejati
- Yan Aolan
- Zach Wiley

This code relies on many open-source projects, most significantly

- projectchrono
- ompl
- foxglove and mcap
- bazel
- podman and buildah
- clang
- gcc
- linux: ubuntu and fedora
- python
- git
- TODO: add any more to this list that may have been forgotten by accident.
