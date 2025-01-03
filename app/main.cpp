#include "slam.h"
#include <Eigen/Core>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

int main(int argc, char **argv) {
  Slam slam("./dataset/sequences/00");
  slam.run();
  return 0;
}
