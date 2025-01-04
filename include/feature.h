#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

struct Image;

struct Feature {
  cv::KeyPoint keypoint_;
  cv::Mat descriptor_;
  std::weak_ptr<Image> image_;

  Feature(cv::KeyPoint keypoint, cv::Mat descriptor,
          std::shared_ptr<Image> image)
      : keypoint_(keypoint), descriptor_(descriptor), image_(image) {}
};

#endif
