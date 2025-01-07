#include "frontend.h"
#include <Eigen/Core>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

// currently this just sets the current frame to frame and matches
// features across the two images in the frame
bool Frontend::add_frame(std::shared_ptr<Frame> frame) {
  current_frame_ = frame;

  switch (status_) {
  case FrontendStatus::Init:
    initialize();
    break;
  case FrontendStatus::Tracking:
    track();
    break;
  case FrontendStatus::Lost:
    return false;
  }

  prev_frame_ = current_frame_;
  return true;
}

void Frontend::initialize() {
  insert_keyframe();
  status_ = FrontendStatus::Tracking;
}

void Frontend::track() {
  extractor_.extract_features(current_frame_->image_left_);

  std::vector<cv::DMatch> matches = matcher_.match_features(
      current_frame_->image_left_, prev_frame_->image_left_);

  // matched features should see the same landmarks
  for (const auto &match : matches) {
    current_frame_->left_features()[match.queryIdx]->landmark_ =
        prev_frame_->left_features()[match.trainIdx]->landmark_;
  }

  int num_inliers = current_frame_->estimate_motion();
  if (num_inliers < num_features_for_keyframe_) {
    insert_keyframe();
  }
}

void Frontend::insert_keyframe() {
  if (!current_frame_->has_left_features()) {
    extractor_.extract_features(current_frame_->image_left_);
  }
  if (!current_frame_->has_right_features()) {
    extractor_.extract_features(current_frame_->image_right_);
  }
  if (!current_frame_->has_matched_features()) {
    current_frame_->matches_ = matcher_.match_features(
        current_frame_->image_left_, current_frame_->image_right_);
  }

  current_frame_->triangulate();

  current_frame_->is_keyframe_ = true;
  map_->insert_keyframe(current_frame_);
  std::cout << "Keyframe inserted successfully\n";
}
