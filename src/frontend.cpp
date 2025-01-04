#include "frontend.h"
#include <memory>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>

// currently this just sets the current frame to frame and matches
// features across the two images in the frame
bool Frontend::add_frame(std::shared_ptr<Frame> frame) {
  current_frame_ = frame;
  match_features();

  // TODO: Implement the initialize() and track() functions below
  switch (status_) {
  case FrontendStatus::Init:
    initialize();
    break;
  case FrontendStatus::Tracking:
    track();
    break;
  case FrontendStatus::Lost:
    return false;
  }
  return true;
}

void Frontend::initialize() {}

void Frontend::track() {}

// As currently setup, this would make more sense inside of the Frame class.
void Frontend::match_features() {
  extractor_.extract_features(current_frame_->image_left_);
  extractor_.extract_features(current_frame_->image_right_);
  std::vector<cv::DMatch> matches;
  matcher_->match(current_frame_->image_left_->descriptors_,
                  current_frame_->image_right_->descriptors_, matches);

  cv::Mat out_img;
  cv::drawMatches(current_frame_->image_left_->mat_,
                  current_frame_->image_left_->keypoints_,
                  current_frame_->image_right_->mat_,
                  current_frame_->image_right_->keypoints_, matches, out_img);
  cv::imshow("Matches", out_img);
  cv::waitKey(1);
}
