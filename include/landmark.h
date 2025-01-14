#ifndef LANDMARK_H
#define LANDMARK_H

#include "observation.h"
#include <Eigen/Core>
#include <memory>

class Feature;

class Landmark {
public:
  const unsigned long id_;
  Eigen::Vector3d position_ = Eigen::Vector3d::Zero();
  bool is_outlier_ = false;

  std::unordered_map<unsigned long, std::shared_ptr<Observation>> observations_;

  Landmark() : id_{landmark_id_++} {}

  Landmark(Eigen::Vector3d position)
      : id_{landmark_id_++}, position_{std::move(position)} {}

  void add_observation(std::shared_ptr<Observation> feature);

  void remove_observation(std::shared_ptr<Observation> feature);

  int num_observations() const { return observations_.size(); }

private:
  inline static unsigned long landmark_id_ = 0;
};

#endif
