#include "image.h"

void Image::set_features(std::vector<cv::KeyPoint> keypoints,
                         cv::Mat descriptors) {
  keypoints_ = std::move(keypoints);
  descriptors_ = std::move(descriptors);

  features_.reserve(keypoints_.size());
  for (int i = 0; i < keypoints_.size(); ++i) {
    features_.push_back(
        std::make_shared<Feature>(keypoints_[i], descriptors_.row(i)));
  }
}
