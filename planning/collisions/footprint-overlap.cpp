#include "planning/footprint-overlap.h"

#include <clipper2/clipper.h>

namespace planning {

static Clipper2Lib::PathD FootprintToClipper(const Footprint &footprint) {

  using namespace Clipper2Lib;

  PathD clipperPath;
  for (const auto &point : footprint) {
    clipperPath.emplace_back(PointD(point.x(), point.y()));
  }
  return clipperPath;
}

bool AreFootprintsOverlapping(const Footprint &alpha_, const Footprint &beta_) {

  using namespace Clipper2Lib;

  PathD alpha = FootprintToClipper(alpha_);
  PathD beta = FootprintToClipper(beta_);

  PathsD solution = Intersect({alpha}, {beta}, FillRule::NonZero);

  if (solution.empty()) {
    return false;
  }
  return true;
}

bool IsPointInFootprint(const Point &point_, const Footprint &footprint_) {
  using namespace Clipper2Lib;

  PointD point = PointD(point_.x(), point_.y());
  PathD footprint = FootprintToClipper(footprint_);

  if (PointInPolygon(point, footprint) == PointInPolygonResult::IsOutside) {
    return false;
  }
  return true;
}

bool IsBodyCollidingAnyObstacles(const Footprints &body,
                                 const Footprints &obstacles) {

  bool isCollision = false;
  for (const auto &bodyPart : body) {
    for (const auto &obs : obstacles) {
      isCollision |= AreFootprintsOverlapping(bodyPart, obs);
    }
  }
  return isCollision;
};

} // namespace planning
