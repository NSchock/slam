#include "image.h"

// TODO: I'm making lots of unnecessary copies in this
void Image::set_features(
    const std::vector<std::shared_ptr<Feature>> &features) {
  for (const auto &feature : features) {
    features_.push_back(feature);
    keypoints_.push_back(feature->keypoint_);
    descriptors_.push_back(feature->descriptor_);
  }
}
