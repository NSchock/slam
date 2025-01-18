#include "frontend.h"
#include "backend.h"
#include "landmark.h"
#include "triangulate.h"
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
  if (!prev_frame_) {
    return;
  }
  current_frame_->set_pose_world_to_camera(relative_motion_ *
                                           prev_frame_->pose_world_to_camera());

  auto left_features = extractor_.extract_features(current_frame_->image_left_);
  left_features =
      extractor_.adaptive_nonmaximal_suppression(left_features, 500);
  current_frame_->image_left_.set_features(left_features);

  auto matches = matcher_.match_features(prev_frame_->image_left_,
                                         current_frame_->image_left_);

  // A left feature point of the current frame is added to the feature_points
  // vector if and only if it has been matched to a left feature point of the
  // previous frame, and that previous left feature point has a valid landmark
  // associated to it, in which case that landmark is also added to the
  // landmark_points vector.
  std::vector<cv::Point3f> landmark_points;
  std::vector<std::shared_ptr<Landmark>> valid_landmarks;
  std::vector<cv::Point2f> feature_points;
  std::vector<std::shared_ptr<Feature>> valid_features;
  for (const auto &match : matches) {
    if (auto landmark = match.feature_1->landmark_.lock()) {
      cv::Point3f landmark_point;
      landmark_point.x = landmark->position_.x();
      landmark_point.y = landmark->position_.y();
      landmark_point.z = landmark->position_.z();

      landmark_points.push_back(landmark_point);
      valid_landmarks.push_back(landmark);

      feature_points.push_back(match.feature_2->point());
      valid_features.push_back(match.feature_2);
    }
  }
  auto [pose_estimate, inlier_indices] =
      estimate_motion(landmark_points, feature_points);
  current_frame_->set_pose_world_to_camera(pose_estimate);
  for (auto idx : inlier_indices) {
    valid_features[idx]->landmark_ = valid_landmarks[idx];
  }

  auto tmp_relative_motion =
      pose_estimate * prev_frame_->pose_camera_to_world();
  if (!is_valid_frame(inlier_indices.size(), tmp_relative_motion)) {
    num_frames_lost_++;
    if (num_frames_lost_ > num_frames_lost_allowed_) {
      status_ = FrontendStatus::Lost;
    }
    return;
  }
  num_frames_lost_ = 0;
  relative_motion_ = tmp_relative_motion;
  if (inlier_indices.size() < num_features_for_keyframe_) {
    insert_keyframe();
  }
  prev_frame_ = current_frame_;
}

bool Frontend::is_valid_frame(int num_inliers, Sophus::SE3d rel_motion) {
  return num_inliers >= num_features_tracking_bad_ &&
         rel_motion.log().norm() <=
             5.0 * (current_frame_->id_ - prev_frame_->id_);
}

void Frontend::insert_keyframe() {
  if (!current_frame_->has_left_features()) {
    auto left_features =
        extractor_.extract_features(current_frame_->image_left_);
    left_features =
        extractor_.adaptive_nonmaximal_suppression(left_features, 1000);
    current_frame_->image_left_.set_features(left_features);
  }
  if (!current_frame_->has_right_features()) {
    auto right_features =
        extractor_.extract_features(current_frame_->image_right_);
    right_features =
        extractor_.adaptive_nonmaximal_suppression(right_features, 1000);
    current_frame_->image_right_.set_features(right_features);
  }
  if (!current_frame_->has_matched_features()) {
    current_frame_->matches_ = matcher_.match_features(
        current_frame_->image_left_, current_frame_->image_right_);
  }

  triangulate();

  current_frame_->is_keyframe_ = true;
  map_->insert_keyframe(current_frame_);
  backend_->optimize();

  // viewer_->update_map();
  std::cout << "Keyframe inserted successfully\n";
}

std::pair<Sophus::SE3d, std::vector<int>>
Frontend::estimate_motion(const std::vector<cv::Point3f> &landmarks,
                          const std::vector<cv::Point2f> &features) {
  cv::Mat rotation_vector, translation_vector;

  // use initial estimates for rotation and translation
  cv::eigen2cv(current_frame_->pose_world_to_camera().rotationMatrix(),
               rotation_vector);
  cv::Rodrigues(rotation_vector, rotation_vector);
  cv::eigen2cv(current_frame_->pose_world_to_camera().translation(),
               translation_vector);

  cv::Mat intrinsic_matrix_cv;
  cv::eigen2cv(camera_left_->intrinsic_matrix(), intrinsic_matrix_cv);

  std::vector<int> inliers;
  cv::solvePnPRansac(landmarks, features, intrinsic_matrix_cv, cv::Mat(),
                     rotation_vector, translation_vector, true, 100, 8.0, 0.99,
                     inliers, cv::SOLVEPNP_ITERATIVE);

  cv::Mat rotation_matrix_cv;
  Eigen::Matrix3d rotation_matrix;
  cv::Rodrigues(rotation_vector, rotation_matrix_cv);
  cv::cv2eigen(rotation_matrix_cv, rotation_matrix);

  Eigen::Vector3d translation;
  cv::cv2eigen(translation_vector, translation);

  // pose = world to camera
  auto estimated_pose = Sophus::SE3d(rotation_matrix, translation);

  // For debugging purposes:
  // std::cout << "Estimated rotation: " << estimated_pose.rotationMatrix()
  //          << "\n";

  // std::cout << "Actual rotation: "
  //           << current_frame_->real_pose_world_to_camera_.rotationMatrix()
  //           << "\n";
  // std::cout
  //     << "Actual translation: "
  //     <<
  //     current_frame_->real_pose_world_to_camera_.rotationMatrix().inverse() *
  //            current_frame_->real_pose_world_to_camera_.translation()
  //     << "\n";
  // std::cout << "Estimated translation: "
  //           << estimated_pose.rotationMatrix().inverse() *
  //                  estimated_pose.translation()
  //           << "\n";
  // std::cout << "Num inliers: " << inliers.size() << "\n";
  return {estimated_pose, inliers};
}

void Frontend::triangulate() {
  for (const auto &match : current_frame_->matches_) {
    auto feature_left = match.feature_1;
    auto feature_right = match.feature_2;

    // feature_left has an associated landmark if and only if it was matched to
    // a left feature with a landmark from the previous frame, and was an inlier
    // in the pose estimation for the current frame. In this case, triangulation
    // is not necessary since we already have the 3d point (landmark).
    auto landmark = feature_left->landmark_.lock();
    if (landmark) {
      feature_right->landmark_ = landmark;
      auto observation = std::make_shared<Observation>(
          current_frame_, feature_left, feature_right, landmark);
      landmark->add_observation(observation);
      current_frame_->add_observation(observation);
    } else {
      auto point_left = feature_left->keypoint_.pt;
      auto point_right = feature_right->keypoint_.pt;
      auto [triangulated_point, valid_triangulation] = triangulate_points(
          camera_left_->projection_matrix() *
              current_frame_->pose_world_to_camera().matrix(),
          camera_right_->projection_matrix() *
              current_frame_->pose_world_to_camera().matrix(),
          point_left, point_right);
      if (valid_triangulation) {
        landmark = std::make_shared<Landmark>(triangulated_point);

        feature_left->landmark_ = landmark;
        feature_right->landmark_ = landmark;

        auto observation = std::make_shared<Observation>(
            current_frame_, feature_left, feature_right, landmark);
        landmark->add_observation(observation);
        current_frame_->add_observation(observation);

        map_->insert_landmark(landmark);
      }
    }
  }
}
