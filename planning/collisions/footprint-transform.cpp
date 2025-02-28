
#include "planning/footprint-transform.h"
#include "planning/types.h"

#include <cmath>
#include <iostream>
#include <limits>
#include <math.h>
#include <string>

#include <Eigen/Dense>

namespace planning {

static Footprint OriginBox(double length, double width,
                           double lengthOffsetFactor = 0) {

  // start out at origin, long = x lat = y, ccw > 0
  // length offset factor 0 means box rear will be at origin
  // length offset factor 1 means box front will be at origin
  // i.e. revoy length offset factow == 0, trailer length offset factor == 1

  assert(lengthOffsetFactor >= 0);
  assert(lengthOffsetFactor <= 1);

  const double forward = ((1 - lengthOffsetFactor) * length);
  const double rear = -(lengthOffsetFactor * length);

  Footprint box{
      {forward, -width / 2.0},
      {forward, width / 2.0},
      {rear, width / 2.0},
      {rear, -width / 2.0},
  };

  return box;
}

Point RotatePoint(const Point &point, double yaw) {
  Eigen::Matrix2d rotation;
  rotation << cos(yaw), -sin(yaw), sin(yaw), cos(yaw);
  Eigen::Vector2d newPoint = rotation * point;
  return newPoint;
}

Footprint TransformFootprint(const Footprint &box, const Pose &pose) {
  std::vector<Eigen::Vector2d> footprint;
  for (Eigen::Vector2d point : box) {
    point = RotatePoint(point, pose.yaw);
    point = point + pose.position;
    footprint.emplace_back(std::move(point));
  }
  return footprint;
}

Footprint ReverseTransformFootprint(const Footprint &box, const Pose &pose) {
  std::vector<Eigen::Vector2d> footprint;
  for (Eigen::Vector2d point : box) {
    point = point - pose.position;
    point = RotatePoint(point, -pose.yaw);
    footprint.emplace_back(std::move(point));
  }
  return footprint;
}

Point ReverseTransformPoint(const Point &point_, const Pose &pose) {
  Point point = point_ - pose.position;
  point = RotatePoint(point, -pose.yaw);
  return point;
}

Footprints FootprintsFromPose(const HookedPose &pose,
                              const BodyParams &params) {

  Footprints out;
  {
    const Footprint box = OriginBox(params.revoyLength, params.revoyWidth, 0);
    const Footprint revoy = TransformFootprint(box, {pose.position, pose.yaw});
    out.push_back(revoy);
  }
  /// TODO: temp hack, user explicitly sets trailer length to 0 to remove
  /// trailer
  if (params.trailerLength > std::numeric_limits<double>::epsilon()) {

    const Footprint box =
        OriginBox(params.trailerLength, params.trailerWidth, 1);
    const Footprint trailer =
        TransformFootprint(box, {pose.position, pose.trailerYaw});
    out.push_back(trailer);
  }

  return out;
}

} // namespace planning
