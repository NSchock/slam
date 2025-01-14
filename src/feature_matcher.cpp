#include "feature_matcher.h"

std::vector<Match> FeatureMatcher::match_features(const Image &image_1,
                                                  const Image &image_2) {
  std::vector<cv::DMatch> matches_cv;
  matcher_->match(image_1.descriptors(), image_2.descriptors(), matches_cv);
  std::vector<Match> matches;
  for (const auto &match : matches_cv) {
    matches.push_back({image_1.features()[match.queryIdx],
                       image_2.features()[match.trainIdx]});
  }
  return matches;
}
