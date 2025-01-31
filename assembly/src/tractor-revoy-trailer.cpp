#include "assembly/tractor-revoy-trailer.h"

namespace revoy {

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::kraz;

TractorRevoyTrailer::TractorRevoyTrailer(std::shared_ptr<ChTerrain> terrain_)
    : Assembly(terrain_),
      tractor_(std::make_shared<Kraz_tractor>(false, CollisionType::NONE,
                                              ChContactMethod::NSC)),
      revoy_(std::make_shared<Revoy>(CollisionType::NONE)),
      trailer_(std::make_shared<Kraz_trailer>(tractor_->GetSystem(),
                                              CollisionType::NONE)) {

  tractor_->Initialize(initPos, initFwdVel);
  revoy_->Initialize(tractor_->GetChassis(), initPos, initFwdVel);
  trailer_->Initialize(revoy_->GetChassis());

  Assembly::initialize();
}

std::shared_ptr<Kraz_tractor> TractorRevoyTrailer::tractor() const {
  return tractor_;
};

std::shared_ptr<Revoy> TractorRevoyTrailer::revoy() const { return revoy_; };

std::shared_ptr<Kraz_trailer> TractorRevoyTrailer::trailer() const {
  return trailer_;
};

} // namespace revoy
