#pragma once

#include "assembly/assembly.h"

#include <chrono_models/vehicle/kraz/Kraz_tractor.h>
#include <chrono_models/vehicle/kraz/Kraz_trailer.h>

namespace revoy {

class TractorTrailer : public Assembly {

public:
  TractorTrailer(std::shared_ptr<chrono::vehicle::ChTerrain> terrain_);

  virtual ~TractorTrailer() = default;

protected:
  virtual std::shared_ptr<chrono::vehicle::kraz::Kraz_tractor> tractor() const;
  virtual std::shared_ptr<chrono::vehicle::kraz::Kraz_trailer> trailer() const;

private:
  std::shared_ptr<chrono::vehicle::kraz::Kraz_tractor> tractor_;
  std::shared_ptr<chrono::vehicle::kraz::Kraz_trailer> trailer_;
};

} // namespace revoy
