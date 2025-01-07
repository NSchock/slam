#ifndef FRAME_H
#define FRAME_H

#include "camera.h"
#include "feature.h"
#include "image.h"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <sophus/se3.hpp>

class Frame {
public:
  Image image_left_, image_right_;
  std::shared_ptr<Camera> camera_left_, camera_right_;
  std::vector<cv::DMatch> matches_;
  std::vector<std::shared_ptr<Landmark>> landmarks_;

  Sophus::SE3d pose_;
  bool is_keyframe_ = false;
  unsigned long id_;

  Frame(cv::Mat image_left, cv::Mat image_right,
        std::shared_ptr<Camera> camera_left,
        std::shared_ptr<Camera> camera_right)
      : image_left_{image_left}, image_right_{image_right},
        camera_left_{std::move(camera_left)},
        camera_right_{std::move(camera_right)}, id_(frame_id_++) {}

  /**
   * Triangulates the points from the left and right images.
   * Saves the corresponding 3d points as landmarks in the frame, and updates
   * each corresponding left/right feature to indicate that it observes the
   * given landmark.
   *
   * Implicitly requires that features have already been extracted and matched
   * between the two images.
   */
  void triangulate();

  /**
   * Estimates the pose of the current frame based on the left image's features
   * and corresponding landmarks.
   *
   * Modifies the pose_ member variable
   *
   * Implicitly requires that these left image features have been extracted and
   * assigned corresponding landmarks (typically either by matching with
   * previous frame's left image features/landmarks, or by matching with right
   * image features and triangulating).
   */
  int estimate_motion();

  std::vector<std::shared_ptr<Feature>> left_features() const {
    return image_left_.features();
  }

  std::vector<std::shared_ptr<Feature>> right_features() const {
    return image_right_.features();
  }

  bool has_left_features() const { return image_left_.has_features(); }

  bool has_right_features() const { return image_right_.has_features(); }

  bool has_matched_features() const { return !matches_.empty(); }

private:
  inline static unsigned long frame_id_ = 0;
};

#endif
