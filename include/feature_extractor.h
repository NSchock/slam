#ifndef FEATURE_EXTRACTOR_H
#define FEATURE_EXTRACTOR_H

#include "image.h"
#include <opencv2/features2d.hpp>

/**
 * A wrapper class for extracting features from an image.
 * Internally uses OpenCV to detect features and get their descriptors.
 * By default we use ORB, can be customized.
 */
class FeatureExtractor {
public:
  FeatureExtractor() {}

  FeatureExtractor(cv::Ptr<cv::FeatureDetector> detector,
                   cv::Ptr<cv::DescriptorExtractor> descriptor_extractor)
      : detector_(std::move(detector)),
        descriptor_extractor_(std::move(descriptor_extractor)) {}

  /**
   * Detects and returns the features in the given image.
   */
  std::vector<std::shared_ptr<Feature>>
  extract_features(const Image &image) const;

  /**
   * Adaptive nonmaximal suppressional: adjust feature points to achieve a more
   * even distribution.
   */
  std::vector<std::shared_ptr<Feature>> adaptive_nonmaximal_suppression(
      std::vector<std::shared_ptr<Feature>> &features, int num_to_keep) const;

private:
  cv::Ptr<cv::FeatureDetector> detector_{cv::ORB::create(2000)};
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_{cv::ORB::create(1000)};
};

#endif
