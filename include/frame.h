#ifndef FRAME_H
#define FRAME_H

#include "feature.h"
#include "feature_matcher.h"
#include "image.h"
#include "observation.h"
#include <Eigen/Core>
#include <Eigen/src/Core/util/Memory.h>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <sophus/se3.hpp>

class Frame {
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  Image image_left_, image_right_;
  std::vector<Match> matches_;
  std::vector<std::shared_ptr<Landmark>> landmarks_;
  std::vector<std::shared_ptr<Observation>> observations_;
  Sophus::SE3d real_pose_world_to_camera_; // the actual ground truth pose, only
                                           // for debugging purposes

  bool is_keyframe_ = false;
  unsigned long id_;

  Frame(cv::Mat image_left, cv::Mat image_right)
      : image_left_{image_left}, image_right_{image_right}, id_(frame_id_++) {}

  std::vector<std::shared_ptr<Feature>> left_features() const {
    return image_left_.features();
  }

  std::vector<std::shared_ptr<Feature>> right_features() const {
    return image_right_.features();
  }

  bool has_left_features() const { return image_left_.has_features(); }

  bool has_right_features() const { return image_right_.has_features(); }

  bool has_matched_features() const { return !matches_.empty(); }

  void set_pose_world_to_camera(Sophus::SE3d pose) {
    pose_world_to_camera_ = pose;
    pose_camera_to_world_ = pose_world_to_camera_.inverse();
  }

  void set_pose_camera_to_world(Sophus::SE3d pose) {
    pose_camera_to_world_ = pose;
    pose_world_to_camera_ = pose_camera_to_world_.inverse();
  }

  /**
   * The frame's current position in the world.
   * This is represented by an SE3 matrix giving the transformation from world
   * to camera coordinates.
   */
  Sophus::SE3d pose_world_to_camera() const { return pose_world_to_camera_; }

  /**
   * The inverse of the pose. This gives the transformation from camera to world
   * coordinates.
   */
  Sophus::SE3d pose_camera_to_world() const { return pose_camera_to_world_; }

  void add_observation(std::shared_ptr<Observation> observation) {
    observations_.push_back(observation);
  }

private:
  Sophus::SE3d pose_world_to_camera_;
  Sophus::SE3d pose_camera_to_world_;

  inline static unsigned long frame_id_ = 0;
};

#endif
