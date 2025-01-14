#ifndef OBSERVATION_H
#define OBSERVATION_H

#include <memory>

class Feature;
class Frame;
class Landmark;

class Observation {
public:
  std::weak_ptr<Frame> frame_;
  std::shared_ptr<Feature> feature_left_;
  std::shared_ptr<Feature> feature_right_;
  std::weak_ptr<Landmark> landmark_;
  const unsigned long id_;

  Observation(std::shared_ptr<Frame> frame,
              std::shared_ptr<Feature> feature_left,
              std::shared_ptr<Feature> feature_right,
              std::shared_ptr<Landmark> landmark)
      : frame_{std::move(frame)}, feature_left_{std::move(feature_left)},
        feature_right_{std::move(feature_right)},
        landmark_{std::move(landmark)}, id_{observation_id_++} {}

private:
  inline static unsigned long observation_id_ = 0;
};

#endif
