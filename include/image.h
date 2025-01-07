#ifndef IMAGE_H
#define IMAGE_H

#include "feature.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

class Image {
public:
  explicit Image(cv::Mat mat) : mat_(mat) {}

  /**
   * Return the image matrix.
   */
  cv::Mat image() const { return mat_; }

  /**
   * Return the feature keypoints.
   */
  std::vector<cv::KeyPoint> keypoints() const { return keypoints_; }

  /**
   * Return the feature descriptors.
   */
  cv::Mat descriptors() const { return descriptors_; }

  /**
   * Return the features.
   */
  std::vector<std::shared_ptr<Feature>> features() const { return features_; }

  /**
   * Set the image features based on the passsed in keypoints and descriptors.
   */
  void set_features(std::vector<cv::KeyPoint> keypoints, cv::Mat descriptors);

  /**
   * Returns whether the image's features have been computed.
   */
  bool has_features() const { return !features_.empty(); }

private:
  cv::Mat mat_;
  std::vector<cv::KeyPoint> keypoints_;
  cv::Mat descriptors_;
  std::vector<std::shared_ptr<Feature>> features_;
};

#endif
