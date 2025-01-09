#include "frontend.h"
#include <Eigen/Core>
#include <memory>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <sophus/se3.hpp>

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

  viewer_->add_current_frame(current_frame_);
  viewer_->draw();

  return true;
}

void Frontend::initialize() {
  insert_keyframe();
  prev_frame_ = current_frame_;
  status_ = FrontendStatus::Tracking;
}

void Frontend::track() {
  if (prev_frame_) {
    current_frame_->pose_ = relative_motion_ * prev_frame_->pose();
  }
  extractor_.extract_features(current_frame_->image_left_);

  std::vector<cv::DMatch> matches = matcher_.match_features(
      current_frame_->image_left_, prev_frame_->image_left_);

  // matched features should see the same landmarks
  for (const auto &match : matches) {
    current_frame_->left_features()[match.queryIdx]->landmark_ =
        prev_frame_->left_features()[match.trainIdx]->landmark_;
  }

  int num_inliers = current_frame_->estimate_motion(relative_motion_);
  auto tmp_relative_motion_ =
      current_frame_->pose() * prev_frame_->pose_inverse();
  if (!is_valid_frame(num_inliers, tmp_relative_motion_)) {
    num_frames_lost_++;
    if (num_frames_lost_ > num_frames_lost_allowed_) {
      status_ = FrontendStatus::Lost;
    }
    return;
  }
  relative_motion_ = tmp_relative_motion_;
  if (num_inliers < num_features_for_keyframe_) {
    insert_keyframe();
  }
  prev_frame_ = current_frame_;
  std::cout << "Relative motion: " << relative_motion_.matrix3x4() << "\n";
}

bool Frontend::is_valid_frame(int num_inliers, Sophus::SE3d rel_motion) {
  return num_inliers >= num_features_tracking_bad_ &&
         rel_motion.log().norm() <=
             5.0 * (current_frame_->id_ - prev_frame_->id_);
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

  // viewer_->update_map();
  std::cout << "Keyframe inserted successfully\n";
}
