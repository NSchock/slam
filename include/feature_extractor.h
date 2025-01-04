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
   * Detects the features in the given image.
   * Features are saved in the Image object.
   */
  void extract_features(std::shared_ptr<Image> image) const;

private:
  cv::Ptr<cv::FeatureDetector> detector_{cv::ORB::create()};
  cv::Ptr<cv::DescriptorExtractor> descriptor_extractor_{cv::ORB::create()};
};

#endif
