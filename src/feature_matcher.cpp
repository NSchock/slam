#include "feature_matcher.h"

std::vector<cv::DMatch> FeatureMatcher::match_features(const Image &image_1,
                                                       const Image &image_2) {
  std::vector<cv::DMatch> matches;
  matcher_->match(image_1.descriptors(), image_2.descriptors(), matches);
  return matches;
}
