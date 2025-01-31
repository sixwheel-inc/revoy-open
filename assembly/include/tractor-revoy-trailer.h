#pragma once

#include "assembly/assembly.h"

#include <chrono_models/vehicle/kraz/Kraz_tractor.h>
#include <chrono_models/vehicle/kraz/Kraz_trailer.h>
#include <chrono_models/vehicle/kraz/Revoy.h>

namespace revoy {

class TractorRevoyTrailer : public Assembly {

public:
  TractorRevoyTrailer(std::shared_ptr<chrono::vehicle::ChTerrain> terrain_);

  virtual ~TractorRevoyTrailer() = default;

protected:
  virtual std::shared_ptr<chrono::vehicle::kraz::Kraz_tractor> tractor() const;
  virtual std::shared_ptr<chrono::vehicle::kraz::Revoy> revoy() const;
  virtual std::shared_ptr<chrono::vehicle::kraz::Kraz_trailer> trailer() const;

private:
  std::shared_ptr<chrono::vehicle::kraz::Kraz_tractor> tractor_;
  std::shared_ptr<chrono::vehicle::kraz::Revoy> revoy_;
  std::shared_ptr<chrono::vehicle::kraz::Kraz_trailer> trailer_;
};

} // namespace revoy
