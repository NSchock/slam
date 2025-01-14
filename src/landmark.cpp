#include "landmark.h"
#include "feature.h"
#include <memory>

void Landmark::add_observation(std::shared_ptr<Observation> observation) {
  observations_[observation->id_] = observation;
}

void Landmark::remove_observation(std::shared_ptr<Observation> observation) {
  observations_.erase(observation->id_);
}
