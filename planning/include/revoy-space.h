#pragma once

#include "planning/types.h"

#include <math.h>
#include <ompl/base/StateSpace.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/base/spaces/SO2StateSpace.h>
#include <ompl/control/SpaceInformation.h>

namespace planning {

class RevoySpace : public ompl::base::CompoundStateSpace {
public:
  static const uint8_t STATE_SPACE_ONE_TRAILER =
      ompl::base::StateSpaceType::STATE_SPACE_TYPE_COUNT + 1;

  class StateType : public ompl::base::CompoundStateSpace::StateType {

  public:
    StateType() = default;
    double getX() const;
    double getY() const;
    double getYaw() const;
    double getTrailerYaw() const;
    double getHitchAngle() const;
    void setX(double x);
    void setY(double y);
    void setXY(double x, double y);
    void setYaw(double yaw);
    void setTrailerYaw(double yaw);
  };

  RevoySpace();
  ~RevoySpace() override = default;

  unsigned int getDimension() const override;
  void setBounds(const ompl::base::RealVectorBounds &bounds);
  const ompl::base::RealVectorBounds &getBounds() const;
  void registerProjections() override;

  static void Propagate(const RevoySpace::StateType *state,
                        const Controls &controls, const double duration,
                        RevoySpace::StateType *result);

  ompl::base::State *allocState() const override;
  void freeState(ompl::base::State *state) const override;
};

} // namespace planning
