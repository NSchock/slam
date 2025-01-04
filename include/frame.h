#ifndef FRAME_H
#define FRAME_H

#include "feature.h"
#include <memory>
#include <opencv2/core/mat.hpp>
#include <sophus/se3.hpp>

class Frame {
public:
  std::shared_ptr<Image> image_left_, image_right_;
  Sophus::SE3d pose_;

  Frame(cv::Mat image_left, cv::Mat image_right)
      : image_left_{std::make_shared<Image>(image_left)},
        image_right_{std::make_shared<Image>(image_right)} {}
};

#endif
