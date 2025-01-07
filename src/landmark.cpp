#include "landmark.h"
#include "feature.h"
#include <memory>

void Landmark::add_observation(std::shared_ptr<Feature> feature) {
  observations_[feature->id_] = feature;
}

void Landmark::remove_observation(std::shared_ptr<Feature> feature) {
  observations_.erase(feature->id_);
}
