#include "planning/revoy-space.h"
#include "planning/mock-revoy-ev.h"

#include <ompl/tools/config/MagicConstants.h>

#include <cstring>

namespace planning {

RevoySpace::RevoySpace() {
  setName("Revoy" + getName());
  type_ = STATE_SPACE_ONE_TRAILER;

  // the main body
  addSubspace(std::make_shared<ompl::base::SE2StateSpace>(), 1);

  // the hitch
  addSubspace(std::make_shared<ompl::base::SO2StateSpace>(), 0.5);
  lock();
}

unsigned int RevoySpace::getDimension() const { return 4; }

void RevoySpace::setBounds(const ompl::base::RealVectorBounds &bounds) {
  as<ompl::base::SE2StateSpace>(0)->setBounds(bounds);
}

const ompl::base::RealVectorBounds &RevoySpace::getBounds() const {
  return as<ompl::base::SE2StateSpace>(0)->getBounds();
}

ompl::base::State *RevoySpace::allocState() const {
  return CompoundStateSpace::allocState();
}

void RevoySpace::freeState(ompl::base::State *state) const {
  CompoundStateSpace::freeState(state);
}

void RevoySpace::registerProjections() {
  // re-use SE2 default projection
  as<ompl::base::SE2StateSpace>(0)->registerProjections();
  registerDefaultProjection(
      as<ompl::base::SE2StateSpace>(0)->getDefaultProjection());
}

void RevoySpace::Propagate(const RevoySpace::StateType *state,
                           const Controls &controls, const double duration,
                           RevoySpace::StateType *result) {

  // propagation startint pose
  const HookedPose start{
      {state->getX(), state->getY()},
      state->getYaw(),
      state->getTrailerYaw(),
  };

  // propagation result
  MockRevoyEv revoyEv(start);
  revoyEv.update(controls, duration);
  const HookedPose end = revoyEv.getHookedPose();

  // set state values
  result->setXY(end.position.x(), end.position.y());
  result->setYaw(end.yaw);
  result->setTrailerYaw(end.trailerYaw);
}

double RevoySpace::StateType::getX() const {
  return as<ompl::base::SE2StateSpace::StateType>(0)->getX();
}
double RevoySpace::StateType::getY() const {
  return as<ompl::base::SE2StateSpace::StateType>(0)->getY();
}
double RevoySpace::StateType::getYaw() const {
  return as<ompl::base::SE2StateSpace::StateType>(0)->getYaw();
}
double RevoySpace::StateType::getTrailerYaw() const {
  return as<ompl::base::SO2StateSpace::StateType>(1)->value;
}
double RevoySpace::StateType::getHitchAngle() const {
  return getYaw() - getTrailerYaw();
}

void RevoySpace::StateType::setX(double x) {
  as<ompl::base::SE2StateSpace::StateType>(0)->setX(x);
}
void RevoySpace::StateType::setY(double y) {
  as<ompl::base::SE2StateSpace::StateType>(0)->setY(y);
}
void RevoySpace::StateType::setXY(double x, double y) {
  setX(x);
  setY(y);
}
void RevoySpace::StateType::setYaw(double yaw) {
  as<ompl::base::SE2StateSpace::StateType>(0)->setYaw(yaw);
}
void RevoySpace::StateType::setTrailerYaw(double yaw) {
  as<ompl::base::SO2StateSpace::StateType>(1)->value = yaw;
}

} // namespace planning
