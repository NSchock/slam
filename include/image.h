#ifndef IMAGE_H
#define IMAGE_H

#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

struct Image {
  cv::Mat mat_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;

  Image(cv::Mat mat) : mat_(mat) {}
};

#endif
