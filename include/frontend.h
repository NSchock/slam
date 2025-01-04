#ifndef FRONTEND_H
#define FRONTEND_H

#include "feature_extractor.h"
#include "frame.h"
#include <memory>
#include <opencv2/core/base.hpp>
#include <opencv2/features2d.hpp>

enum class FrontendStatus { Init, Tracking, Lost };

/**
 * SLAM Frontend: Perform visual odometry to estimate camera motion.
 */
class Frontend {

public:
  Frontend() {}

  /**
   * Processes the given frame as the next frame in the system.
   *
   * TODO: not implemented, currently I've set it to just display the left/right
   * images for demonstration purposes.
   */
  bool add_frame(std::shared_ptr<Frame> frame);

  void initialize();

  void track();

  /**
   * Match features in the current frame.
   */
  void match_features();

private:
  FrontendStatus status_{FrontendStatus::Init};

  FeatureExtractor extractor_;
  cv::Ptr<cv::DescriptorMatcher> matcher_{
      cv::BFMatcher::create(cv::NORM_HAMMING, true)};

  std::shared_ptr<Frame> current_frame_;
  std::shared_ptr<Frame> prev_frame_;
};

#endif
