#include "frame.h"
#include "landmark.h"
#include "triangulate.h"
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <sophus/se3.hpp>

void Frame::triangulate() {
  Eigen::Matrix<double, 3, 4> proj_left = camera_left_->projection_matrix();
  Eigen::Matrix<double, 3, 4> proj_right = camera_right_->projection_matrix();

  for (const auto &match : matches_) {
    Eigen::Vector3d triangulated_point = triangulate_points(
        proj_left, proj_right, image_left_.keypoints()[match.queryIdx].pt,
        image_right_.keypoints()[match.trainIdx].pt);

    // TODO: verify triangulated point is valid before constructing landmark
    std::shared_ptr<Landmark> landmark =
        std::make_shared<Landmark>(triangulated_point);
    landmarks_.push_back(landmark);
    left_features()[match.queryIdx]->landmark_ = landmark;
    right_features()[match.trainIdx]->landmark_ = landmark;
  }
}

int Frame::estimate_motion() {
  std::vector<cv::Point3f> points_3d;
  std::vector<cv::Point2f> points_2d;

  for (const auto &feature : left_features()) {
    if (auto landmark = feature->landmark_.lock()) {
      cv::Mat landmark_position;
      cv::eigen2cv(landmark->position_, landmark_position);
      points_3d.push_back(cv::Point3f(landmark_position));
      points_2d.push_back(feature->keypoint_.pt);
    }
  }

  /**
   * TODO: slambook keeps track of the relative motion between current frame
   * and previous frame, and uses it as initial estimate for estimating the
   * motion of the current frame. We could do the same if desired, by passing in
   * rotation_vector and translation_vector computed in track() function by
   * comparing current and previous frame, and setting useExtrinsicGuess
   * parameter of solvePnPRansac to true instead of false. This may improve
   * accuracy or speed.
   */
  cv::Mat rotation_vector, translation_vector, inliers;
  cv::Mat intrinsic_matrix;
  cv::eigen2cv(camera_left_->intrinsic_matrix_, intrinsic_matrix);

  cv::solvePnPRansac(points_3d, points_2d, intrinsic_matrix, cv::Mat(),
                     rotation_vector, translation_vector, false, 100, 8.0, 0.99,
                     inliers);

  cv::Mat rotation_matrix_cv;
  Eigen::Matrix3d rotation_matrix;
  cv::Rodrigues(rotation_vector, rotation_matrix_cv);
  cv::cv2eigen(rotation_matrix_cv, rotation_matrix);

  Eigen::Vector3d translation;
  cv::cv2eigen(translation_vector, translation);

  pose_ = Sophus::SE3d(rotation_matrix, translation);
  return inliers.rows;
}
