#include "assembly/tractor-trailer.h"

namespace revoy {

using namespace chrono;
using namespace chrono::vehicle;
using namespace chrono::vehicle::kraz;

TractorTrailer::TractorTrailer(std::shared_ptr<ChTerrain> terrain_)
    : Assembly(terrain_),
      tractor_(std::make_shared<Kraz_tractor>(false, CollisionType::NONE,
                                              ChContactMethod::NSC)),
      trailer_(std::make_shared<Kraz_trailer>(tractor_->GetSystem(),
                                              CollisionType::NONE)) {

  tractor_->Initialize(initPos, initFwdVel);
  trailer_->Initialize(tractor_->GetChassis());
  Assembly::initialize();
}

std::shared_ptr<Kraz_tractor> TractorTrailer::tractor() const {
  return tractor_;
};

std::shared_ptr<Kraz_trailer> TractorTrailer::trailer() const {
  return trailer_;
};

} // namespace revoy
