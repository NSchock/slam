#ifndef FRAME_H
#define FRAME_H

#include "feature.h"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <sophus/se3.hpp>

class Frame {
public:
  cv::Mat image_left_, image_right_;

  Frame(cv::Mat image_left, cv::Mat image_right)
      : image_left_(std::move(image_left)),
        image_right_(std::move(image_right)) {}

private:
  std::vector<std::shared_ptr<Feature>> features_left_, features_right_;
  Sophus::SE3d pose_;
};

#endif
