#pragma once

#include <Eigen/Dense>
#include <Eigen/Geometry>

#include <cstdint>
#include <limits>
#include <vector>

/// needed for windows
#if (!defined(M_PI))
#define M_PI 3.141592653589793238462643383279502884
#endif

/// [-pi, pi]
double fixRadian(double value);

namespace planning {

using Point = Eigen::Vector2d;
using Footprint = std::vector<Eigen::Vector2d>;
using Footprints = std::vector<Footprint>;
using Path = std::vector<Eigen::Vector2d>;

struct HookedPose {
  Point position = {0, 0};
  double yaw = 0;
  double trailerYaw = 0;
};

struct Pose {
  Point position = {0, 0};
  double yaw = 0;
};

struct Entity {
  // fixed frame
  Pose pose;

  // TODO: footprint should be about origin, then rotated and
  // translated to get coords in fixed frame
  Footprint footprint;

  double lifetime = std::numeric_limits<double>::max();
};

using Entities = std::vector<Entity>;

struct Bounds {
  // meters
  double upperX = 50;
  double upperY = 50;
  double lowerX = -50;
  double lowerY = -50;
};

struct TimeParams {
  // TODO use units.h
  // microseconds

  // 100,000 us == 100ms == 0.1s
  double dt = 1e5;

  // using 0 sometimes causes problems
  // 1.7e15 is GMT Tuesday, November 14, 2023 10:13:20 PM
  double startTime = 1.7e15;

  // 60,000,000 us == 60,000 ms == 60 s
  double timeout = 6e7;
};

struct BodyParams {
  // meters
  // TODO uses units.h
  double revoyLength = 6;
  double revoyWidth = 2.5;
  double trailerLength = 20;
  double trailerWidth = 2.5;
};

struct Scenario {
  std::string name;
  Footprints walls;
  Entities entities;
  HookedPose start;
  HookedPose goal;
  Bounds bounds;
  TimeParams timeParams;
  BodyParams bodyParams;
};

struct Controls {
  double speed = 0;
  double steer = 0;
  double duration = 0;
};

struct Graph {
  std::vector<Point> nodes;
  std::vector<std::tuple<size_t, size_t>> edges;
};

} // namespace planning
