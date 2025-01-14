#ifndef FEATURE_MATCHER_H
#define FEATURE_MATCHER_H

#include "image.h"
#include <opencv2/core/base.hpp>
#include <opencv2/features2d.hpp>

class Feature;

struct Match {
  std::shared_ptr<Feature> feature_1;
  std::shared_ptr<Feature> feature_2;
};

class FeatureMatcher {
public:
  FeatureMatcher() {}

  FeatureMatcher(cv::Ptr<cv::DescriptorMatcher> matcher)
      : matcher_{std::move(matcher)} {}

  /**
   * Matches the features between the two images, and returns the resulting
   * DMatch objects. It is assumed that the features for the two images have
   * already been computed.
   */
  std::vector<Match> match_features(const Image &image_1, const Image &image_2);

private:
  // TODO: using BFMatcher isn't very wise here; we know images are already
  // rectified, so we could compute matches much more efficiently, or, e.g.,
  // compute disparity map
  cv::Ptr<cv::DescriptorMatcher> matcher_{
      cv::BFMatcher::create(cv::NORM_HAMMING, true)};
};

#endif
