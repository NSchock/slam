#include "feature_extractor.h"

void FeatureExtractor::extract_features(std::shared_ptr<Image> image) const {
  detector_->detect(image->mat_, image->keypoints_);
  cv::Mat descriptors;
  descriptor_extractor_->compute(image->mat_, image->keypoints_,
                                 image->descriptors_);
}
