#include "feature_extractor.h"
#include <memory>

std::vector<std::shared_ptr<Feature>>
FeatureExtractor::extract_features(const Image &image) const {
  std::vector<cv::KeyPoint> keypoints;
  detector_->detect(image.image(), keypoints);
  cv::Mat descriptors;
  descriptor_extractor_->compute(image.image(), keypoints, descriptors);

  std::vector<std::shared_ptr<Feature>> features;
  features.reserve(keypoints.size());
  for (size_t i = 0; i < keypoints.size(); ++i) {
    features.push_back(
        std::make_shared<Feature>(keypoints[i], descriptors.row(i)));
  }
  return features;
  // image.set_features(std::move(keypoints), std::move(descriptors));
}
