#include "feature_extractor.h"
#include <functional>
#include <limits>
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

std::vector<std::shared_ptr<Feature>>
FeatureExtractor::adaptive_nonmaximal_suppression(
    std::vector<std::shared_ptr<Feature>> &features, int num_to_keep) const {
  if (features.size() < num_to_keep) {
    return features;
  }

  std::sort(features.begin(), features.end(),
            [&](std::shared_ptr<Feature> left, std::shared_ptr<Feature> right) {
              return left->keypoint_.response > right->keypoint_.response;
            });

  std::vector<double> radii(features.size());
  std::vector<double> radii_sorted(features.size());
  const float robust_coeff = 1.11;

  for (size_t i = 0; i < features.size(); ++i) {
    const float response = features[i]->keypoint_.response * robust_coeff;
    double radius = std::numeric_limits<double>::max();
    for (size_t j = 0; j < i && features[j]->keypoint_.response > response;
         ++j) {
      radius = std::min(radius,
                        cv::norm(features[i]->point() - features[j]->point()));
    }
    radii[i] = radius;
    radii_sorted[i] = radius;
  }

  std::sort(radii_sorted.begin(), radii_sorted.end(), std::greater<>());
  const double decision_radius = radii_sorted[num_to_keep];

  std::vector<std::shared_ptr<Feature>> anms_features;
  for (size_t i = 0; i < radii.size(); ++i) {
    if (radii[i] >= decision_radius) {
      anms_features.push_back(features[i]);
    }
  }
  return anms_features;
}
