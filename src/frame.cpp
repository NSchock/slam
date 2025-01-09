#include "frame.h"
#include "landmark.h"
#include "triangulate.h"
#include <Eigen/Core>
#include <Eigen/src/Core/Matrix.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/eigen.hpp>
#include <opencv2/sfm/triangulation.hpp>
#include <sophus/se3.hpp>

void Frame::triangulate() {
  auto proj_left = camera_left_->projection_matrix();
  auto proj_right = camera_right_->projection_matrix();

  for (const auto &match : matches_) {
    auto point_left = image_left_.keypoints()[match.queryIdx].pt;
    auto point_right = image_right_.keypoints()[match.trainIdx].pt;

    auto [triangulated_point, valid_triangulation] =
        triangulate_points(proj_left, proj_right, point_left, point_right);

    if (valid_triangulation) {
      // I returned 4d homogeneous vector from triangulate_points solely to
      // compare against opencv ideally change back to returning 3d vector
      Eigen::Vector3d dehom_triangulated_point =
          (triangulated_point / triangulated_point(3)).head<3>();
      // multiplying by pose_inverse is what makes things blowup. but it's also
      // necessary: triangulation will recover the 3d point in camera
      // coordinates, need to multiply by pose_inverse to convert from camera to
      // world coordinates.
      dehom_triangulated_point = pose_inverse() * dehom_triangulated_point;

      // std::cout << "left: " << point_left << "\n";
      // std::cout << "right: " << point_right << "\n";
      // std::cout << "triangulated: " << triangulated_point << "\n";
      // std::cout << "last: " << triangulated_point(3) << "\n";
      //  std::cout << "dehom triangulated: "
      //            << (triangulated_point / triangulated_point(3)).head<3>()
      //            << "\n";
      //  std::cout << "dehom triangulated * inverse pose: "
      //            << dehom_triangulated_point << "\n";

      std::shared_ptr<Landmark> landmark =
          std::make_shared<Landmark>(dehom_triangulated_point);
      landmark->add_observation(left_features()[match.queryIdx]);
      landmark->add_observation(right_features()[match.trainIdx]);
      landmarks_.push_back(landmark);
      left_features()[match.queryIdx]->landmark_ = landmark;
      right_features()[match.trainIdx]->landmark_ = landmark;
    }
  }
}

int Frame::estimate_motion(Sophus::SE3d relative_motion) {
  std::vector<cv::Point3f> points_3d;
  std::vector<cv::Point2f> points_2d;

  for (const auto &feature : left_features()) {
    if (auto landmark = feature->landmark_.lock()) {
      cv::Point3f landmark_position;
      landmark_position.x = landmark->position_.x();
      landmark_position.y = landmark->position_.y();
      landmark_position.z = landmark->position_.z();
      points_3d.push_back(cv::Point3f(landmark_position));
      points_2d.push_back(feature->keypoint_.pt);
    }
  }

  cv::Mat rotation_vector, translation_vector;
  // use initial estimates for rotation and translation
  cv::eigen2cv(relative_motion.rotationMatrix(), rotation_vector);
  cv::Rodrigues(rotation_vector, rotation_vector);
  cv::eigen2cv(relative_motion.translation(), translation_vector);

  cv::Mat intrinsic_matrix;
  cv::eigen2cv(camera_left_->intrinsic_matrix(), intrinsic_matrix);

  std::vector<int> inliers;

  cv::solvePnPRansac(points_3d, points_2d, intrinsic_matrix, cv::Mat(),
                     rotation_vector, translation_vector, true, 100, 8.0, 0.99,
                     inliers);

  cv::Mat rotation_matrix_cv;
  Eigen::Matrix3d rotation_matrix;
  cv::Rodrigues(rotation_vector, rotation_matrix_cv);
  cv::cv2eigen(rotation_matrix_cv, rotation_matrix);

  Eigen::Vector3d translation;
  cv::cv2eigen(translation_vector, translation);

  // pose = world to camera
  auto estimated_pose = Sophus::SE3d(rotation_matrix, translation);
  std::cout << "Estimated pose: " << estimated_pose.matrix3x4() << "\n";
  pose_ = estimated_pose;

  // for (int i = 0; i < inliers.size(); ++i) {
  //   std::cout << "3d point: " << points_3d[inliers[i]] << "\n";
  //   std::cout << "2d point: " << points_2d[inliers[i]] << "\n";
  //   Eigen::Vector3d point3d;
  //   point3d << points_3d[inliers[i]].x, points_3d[inliers[i]].y,
  //       points_3d[inliers[i]].z;
  //   Eigen::Vector3d proj_point = estimated_pose * point3d;
  //   proj_point = camera_left_->intrinsic_matrix_ * proj_point;
  //   std::cout << "projected 2d point: " << proj_point / proj_point.z() <<
  //   "\n";
  // }
  return inliers.size();
}
