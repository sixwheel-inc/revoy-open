# Planning

## Goals / Roadmap: Yard Autonomy

### v0: Operator Blindspot Safety (Proximity Planner)

Yard Operators move the Revoy EV using a Teleradio that talks directly to the ECU via CAN.

The Revoy EV is large and can block Yard Operators' field-of-view while moving, creating blindspots. Its possible that a hazard could emerge outside the Yard Operator's field-of-view.

We can use sensors on the Revoy EV and in the Yard to observe the Revoy EV's surroundings, notice hazards, and stop the Revoy EV from getting moving into hazards.

### [todo] v1: Automated Deploy / Stow / Charge (Yard Planner)

Yard Operators often move the Revoy EV between the same set positions, back and forth, E.G. from its parking spot to the hook-up location, to the chargers, etc.

Moving several Revoy EVs (with / without Trailers) back and forth can be both tedious and hectic, depending on the situation.

Building on the above, we can use the Revoy EVs sensors and motors to, given a goal pose, automatically and safely move the Revoy EV through the Yard to the goal.

### [todo] v2: Automated Drop-and-Hook (Hook Planner)

To start the trip, the Revoy EV needs to be hooked up to the Trailer, stow the Kickstand, and then it can be hooked up to the Tractor. At the end of the trip, the opposite is done: first the Tractor is un-hooked, Kickstand deployed, then the Trailer.

Separate from the above, we can use additional near-field sensors to carefully reverse the Revoy EV under the Trailer's kingpin, and then carefully drive it forward onto the Tractor's Fifth Wheel.

## Techniques (OMPL based)

### v0: Proximity Planner only

Proximity Planner will always stop the Revoy EV from moving into any obstacle or hazard. It relies on onboard sensors to be aware of what is directly around the EV.

#### inputs

- occupancy grid
- obstactle footprints
- goal

#### outputs

- speed limit (will be used to limit Controller commanded speed)

### [todo] v1: Multi-Level Planning (Yard Planner, Proximity Planner, Assembly Planner)

The Yard Planner will take advantage of offboard sensors, i.e. "I2V" Infrastructure-to-Vehicle communication, allowing the Revoy EV to "see" behind the Trailer.

The Assembly Planner will need seperate onboard sensors to precisely and automatically hook the Trailer and Tractor without damaging any part of the vehicles.

By using a multi-level planner (supported by OMPL), we can use the Yard (I2V) Planner to identify a path through a (potentially obstacle strewn) area, while the Proximity Planner adds extra safety to ensure no collisions with blindspot hazards (I2V can't see through Revoy EV).

#### inputs

- occupancy grid
- obstactle footprints
- goal

#### outputs

- set speed
- set steer

# Future

- interactive (predictive planning)
- probabilistic planning
- Tractor Turn Assist (reduce swept-area of combined assembled vehicle)
