#include "feature_extractor.h"

void FeatureExtractor::extract_features(Image &image) const {
  std::vector<cv::KeyPoint> keypoints;
  detector_->detect(image.image(), keypoints);
  cv::Mat descriptors;
  descriptor_extractor_->compute(image.image(), keypoints, descriptors);
  image.set_features(std::move(keypoints), std::move(descriptors));
}
