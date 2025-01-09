#include "slam.h"
#include "frame.h"
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <sophus/se3.hpp>
#include <sophus/so3.hpp>

bool Slam::read_calib_data() {
  std::ifstream fin(dataset_path_ + "/calib.txt");
  std::string line;
  while (std::getline(fin, line)) {
    std::istringstream iss(line);
    std::string camera_name;
    if (!(iss >> camera_name)) {
      std::cout << "Invalid calib.txt format.\n";
      return false;
    }
    camera_name.pop_back();

    std::array<double, 12> raw_camera{};
    for (int i = 0; i < 12; ++i) {
      iss >> raw_camera[i];
    }

    cameras_.insert({camera_name, std::make_shared<Camera>(raw_camera)});
  }
  return !cameras_.empty();
}

void Slam::initialize() {
  if (initialized_ || !read_calib_data()) {
    return;
  }

  frontend_ = std::make_shared<Frontend>(cameras_["P0"], cameras_["P1"]);
  // backend_ = std::make_shared<Backend>();
  map_ = std::make_shared<Map>();
  viewer_ = std::make_shared<Viewer>();

  frontend_->set_map(map_);
  viewer_->set_map(map_);
  frontend_->set_viewer(viewer_);
  initialized_ = true;

  std::ifstream fin("./dataset/poses/00.txt");
  std::string line;
  while (std::getline(fin, line)) {
    std::istringstream iss(line);

    std::array<double, 12> raw_pose{};
    for (int i = 0; i < 12; ++i) {
      iss >> raw_pose[i];
    }

    Eigen::Matrix3d rotation;
    rotation << raw_pose[0], raw_pose[1], raw_pose[2], raw_pose[4], raw_pose[5],
        raw_pose[6], raw_pose[8], raw_pose[9], raw_pose[10];
    Eigen::Vector3d translation;
    translation << raw_pose[3], raw_pose[7], raw_pose[11];
    Sophus::SE3d pose(Sophus::SO3d::fitToSO3(rotation), translation);
    // data is given as camera to world transformation
    // take inverse to go from world to camera
    poses_.push_back(pose.inverse());
  }
}

bool Slam::step() {
  std::string left_image_name =
      std::format("{}/image_0/{:06}.png", dataset_path_, current_frame_index_);
  cv::Mat left_image = cv::imread(left_image_name, cv::IMREAD_GRAYSCALE);
  if (left_image.empty()) {
    std::cout << "Cannot find left image at index " << current_frame_index_
              << "\n";
    return false;
  }

  std::string right_image_name =
      std::format("{}/image_1/{:06}.png", dataset_path_, current_frame_index_);
  cv::Mat right_image = cv::imread(right_image_name, cv::IMREAD_GRAYSCALE);
  if (right_image.empty()) {
    std::cout << "Cannot find right image at index " << current_frame_index_
              << "\n";
    return false;
  }

  std::shared_ptr<Frame> frame = std::make_shared<Frame>(
      left_image, right_image, cameras_["P0"], cameras_["P1"]);

  frame->pose_ = poses_[current_frame_index_];

  std::cout << "Current frame: " << current_frame_index_ << "\n";
  std::cout << "Pose: " << poses_[current_frame_index_].matrix3x4() << "\n";
  // std::cout << "Pose inverse: "
  //           << poses_[current_frame_index_].inverse().matrix3x4() << "\n";

  current_frame_index_++;
  return frontend_->add_frame(frame);
}

void Slam::run() {
  initialize();
  while (step()) {
  }
};
