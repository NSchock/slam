#include "frontend.h"
#include <memory>
#include <opencv2/highgui.hpp>

bool Frontend::add_frame(std::shared_ptr<Frame> frame) {
  cv::imshow("Left image", frame->image_left_);
  cv::imshow("Right image", frame->image_right_);
  cv::waitKey(1);
  return true;
}
