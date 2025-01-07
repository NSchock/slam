#ifndef FRONTEND_H
#define FRONTEND_H

#include "feature_extractor.h"
#include "feature_matcher.h"
#include "frame.h"
#include "map.h"
#include <memory>
#include <opencv2/core/base.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>

enum class FrontendStatus { Init, Tracking, Lost };

/**
 * SLAM Frontend: Perform visual odometry to estimate camera motion.
 */
class Frontend {

public:
  Frontend(std::shared_ptr<Camera> camera_left,
           std::shared_ptr<Camera> camera_right)
      : camera_left_{std::move(camera_left)},
        camera_right_{std::move(camera_right)} {}

  void set_map(std::shared_ptr<Map> map) { map_ = std::move(map); }

  /**
   * Processes the given frame as the next frame in the system.
   */
  bool add_frame(std::shared_ptr<Frame> frame);

  /**
   * Initializes the map; called when FrontendStatus is Init.
   * This just calls insert_keyframe() for the current frame, and sets the
   * FrontendStatus to Tracking.
   */
  void initialize();

  /**
   * Tracks changes/updates map when processing a new frame.
   *
   * More precisely, this does the following.
   * 1. Extract features from the current frame's left image, and match them
   * with the features from the previous frame's left image.
   * 2. Set the observed landmarks of the current frame's left features to be
   * the same as the landmarks from the previous frame's corresponding left
   * features.
   * 3. Estimate the pose of the current frame from its left features and
   * corresponding landmarks. If the number of inliers from this computation is
   * too small, then call insert_frame() to insert the current frame into the
   * map as a new keyframe.
   */
  void track();

  /**
   * Inserts current frame as new keyframe for the map.
   * This consists of extracting/matching features from the left and right
   * image, triangulating based on these matches, and inserting the frame and
   * resulting landmarks into the map.
   */
  void insert_keyframe();

private:
  std::shared_ptr<Camera> camera_left_, camera_right_;

  FrontendStatus status_{FrontendStatus::Init};

  FeatureExtractor extractor_;
  FeatureMatcher matcher_;

  std::shared_ptr<Frame> current_frame_, prev_frame_;

  std::shared_ptr<Map> map_;

  int num_features_for_keyframe_ = 50;
  int num_features_tracking_bad_ = 20;
};

#endif
