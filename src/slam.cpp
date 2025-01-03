#include "slam.h"
#include "frame.h"
#include <format>
#include <fstream>
#include <iostream>
#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/imgcodecs.hpp>

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
  if (initialized || !read_calib_data()) {
    return;
  }

  frontend_ = std::make_shared<Frontend>();
  backend_ = std::make_shared<Backend>();
  map_ = std::make_shared<Map>();
  viewer_ = std::make_shared<Viewer>();

  initialized = true;
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

  std::shared_ptr<Frame> frame =
      std::make_shared<Frame>(left_image, right_image);

  current_frame_index_++;
  return frontend_->add_frame(frame);
}
