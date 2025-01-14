#ifndef FEATURE_H
#define FEATURE_H

#include <memory>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>

class Landmark;

class Image;

class Feature {
public:
  const unsigned long id_;
  cv::KeyPoint keypoint_;
  cv::Mat descriptor_;
  std::weak_ptr<Landmark> landmark_;
  bool is_outlier_ = false;

  Feature(cv::KeyPoint keypoint, cv::Mat descriptor)
      : id_{feature_id_++}, keypoint_{keypoint}, descriptor_{descriptor} {}

private:
  inline static unsigned long feature_id_ = 0;
};

#endif
