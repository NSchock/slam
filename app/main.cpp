#include "slam.h"
#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
  Slam slam("./dataset/sequences/00");
  slam.read_calib_data();
  Camera left_cam = *slam.get_camera("P0");
  Camera right_cam = *slam.get_camera("P1");
  std::cout << "Left camera data\n";
  std::cout << "Intrinsic matrix:\n" << left_cam.intrinsic_matrix_ << "\n";
  std::cout << "Extrinsic matrix:\n"
            << left_cam.extrinsic_matrix_.matrix() << "\n";
  std::cout << "Baseline:\n" << left_cam.baseline_ << "\n";
  std::cout << "Right camera data\n";
  std::cout << "Intrinsic matrix:\n" << right_cam.intrinsic_matrix_ << "\n";
  std::cout << "Extrinsic matrix:\n"
            << right_cam.extrinsic_matrix_.matrix() << "\n";
  std::cout << "Baseline:\n" << right_cam.baseline_ << "\n";
  return 0;
}
