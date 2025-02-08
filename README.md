# Revoy Open

This repo contains FOSS ports of internal Revoy projects, mainly related to Autonomy and Safety.

## Planning (using OMPL)

### Current Goal: improve operator safety by using autonomy to stop for hazards

Operators on-site use an RC Controller to move the Revoy EV around the yard. Since the RevoyEV is so large, it is difficult for the operator to see all around the vehicle.

We can stop the Revoy from colliding with a hazard, even if the operator is not aware of the hazard (due to line of sight etc).

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

## Citations

### Motion Planning

Șucan, I. A., Moll, M., & Kavraki, L. E. (2012). The Open Motion Planning Library. IEEE Robotics & Automation Magazine, 19(4), 72-82. https://ompl.kavrakilab.org

Kingston, Z., Moll, M., & Kavraki, L. E. (2019). Exploring implicit spaces for constrained sampling-based planning. International Journal of Robotics Research, 38(10-11), 1151-1178.

Dobson, A., & Bekris, K. E. (2014). Sparse roadmap spanners for asymptotically near-optimal motion planning. The International Journal of Robotics Research, 33(1), 18-47. https://doi.org/10.1177/0278364913498292

Dubins, L.E. (1957). On Curves of Minimal Length with a Constraint on Average Curvature, and with Prescribed Initial and Terminal Positions and Tangents. American Journal of Mathematics, 79, 497.

Reeds, J. A., & Shepp, L. A. (1990). Optimal paths for a car that goes both forwards and backwards. Pacific Journal of Mathematics, 145(2), 367-393.

LaValle, S. M. (2020, August 14). Sampling-based algorithms. Planning Algorithms. https://lavalle.pl/planning/node822.html

Jeon, B. (2023, July 23). Gentle introduction to hybrid A-star. Medium. https://medium.com/@junbs95/gentle-introduction-to-hybrid-a-star-9ce93c0d7869

### Vehicle Dynamics

Tasora, A., Serban, R., Mazhar, H., Pazouki, A., Melanz, D., Fleischmann, J., Taylor, M., Sugiyama, H., & Negrut, D. (2016). Chrono: An open source multi-physics dynamics engine. In T. Kozubek (Ed.), High Performance Computing in Science and Engineering – Lecture Notes in Computer Science (pp. 19-49). Springer.

Project Chrono. (2025). Chrono: An Open Source Framework for the Physics-Based Simulation of Dynamic Systems. http://projectchrono.org [Accessed: 2025-02-08]

Project Chrono Development Team. (2024). Chrono: An Open Source Framework for the Physics-Based Simulation of Dynamic Systems. https://github.com/projectchrono/chrono [Accessed: 2024-10-05]

Ding, Y. (2020, February 15). Simple understanding of kinematic bicycle model. Medium. https://dingyan89.medium.com/simple-understanding-of-kinematic-bicycle-model-81cac6420357

LaValle, S. M. (2012, April 20). Nonholonomic and underactuated systems. Planning Algorithms. http://msl.cs.uiuc.edu/planning/node661.html
