#include "viewer.h"
#include "landmark.h"
#include <chrono>
#include <memory>
#include <mutex>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <pangolin/display/display.h>
#include <pangolin/gl/glplatform.h>
#include <pangolin/pangolin.h>
#include <sophus/se3.hpp>
#include <thread>

Viewer::Viewer() {
  // viewer_thread_ = std::thread(std::bind(&Viewer::thread_loop, this));
  //
  pangolin::CreateWindowAndBind("SLAM", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  vis_camera_ = pangolin::OpenGlRenderState(
      pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

  vis_display_ = pangolin::CreateDisplay()
                     .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
                     .SetHandler(new pangolin::Handler3D(vis_camera_));
}

void Viewer::close() {
  viewer_running_ = false;
  // viewer_thread_.join();
}

void Viewer::add_current_frame(std::shared_ptr<Frame> frame) {
  // std::unique_lock<std::mutex> lock(viewer_data_mutex_);
  current_frame_ = frame;
}

void Viewer::thread_loop() {
  pangolin::CreateWindowAndBind("SLAM", 1024, 768);
  glEnable(GL_DEPTH_TEST);
  glEnable(GL_BLEND);
  glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

  pangolin::OpenGlRenderState vis_camera(
      pangolin::ProjectionMatrix(1024, 768, 400, 400, 512, 384, 0.1, 1000),
      pangolin::ModelViewLookAt(0, -5, -10, 0, 0, 0, 0.0, -1.0, 0.0));

  pangolin::View &vis_display =
      pangolin::CreateDisplay()
          .SetBounds(0.0, 1.0, 0.0, 1.0, -1024.0f / 768.0f)
          .SetHandler(new pangolin::Handler3D(vis_camera));

  while (!pangolin::ShouldQuit() && viewer_running_) {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
    vis_display.Activate(vis_camera);

    // std::unique_lock<std::mutex> lock(viewer_data_mutex_);

    if (current_frame_) {
      draw_frame(current_frame_, green_);
      follow_current_frame(vis_camera);

      cv::Mat img = plot_frame_image();
      cv::imshow("Current frame", img);
      cv::waitKey(1);
    }

    if (map_) {
      draw_map_points();
    }

    pangolin::FinishFrame();
    // std::this_thread::sleep_for(std::chrono::milliseconds(5));
  }
}

void Viewer::draw() {
  glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
  glClearColor(1.0f, 1.0f, 1.0f, 1.0f);
  vis_display_.Activate(vis_camera_);

  // std::unique_lock<std::mutex> lock(viewer_data_mutex_);

  if (current_frame_) {
    draw_frame(current_frame_, green_);
    follow_current_frame(vis_camera_);

    cv::Mat img = plot_frame_image();
    cv::imshow("Current frame", img);
    cv::waitKey(1);
  }

  if (map_) {
    draw_map_points();
  }

  pangolin::FinishFrame();
  std::this_thread::sleep_for(std::chrono::milliseconds(5));
}

void Viewer::draw_frame(std::shared_ptr<Frame> frame,
                        const std::array<float, 3> color) {
  auto pose_inverse = current_frame_->pose_inverse();

  const float sz = 1.0;
  const int line_width = 2.0;
  const float fx = 400;
  const float fy = 400;
  const float cx = 512;
  const float cy = 384;
  const float width = 1080;
  const float height = 768;

  glPushMatrix();
  glMultMatrixd(pose_inverse.matrix().data());
  glColor3f(color[0], color[1], color[2]);

  glLineWidth(line_width);
  glBegin(GL_LINES);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(0, 0, 0);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (width - 1 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (height - 1 - cy) / fy, sz);
  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);

  glVertex3f(sz * (0 - cx) / fx, sz * (0 - cy) / fy, sz);
  glVertex3f(sz * (width - 1 - cx) / fx, sz * (0 - cy) / fy, sz);

  glEnd();
  glPopMatrix();
}

void Viewer::follow_current_frame(pangolin::OpenGlRenderState &vis_camera) {
  // pose_inverse = mapping from camera to world coordinates
  auto pose_inverse = current_frame_->pose_inverse();
  pangolin::OpenGlMatrix m(pose_inverse.matrix());
  vis_camera.Follow(m, true);
}

cv::Mat Viewer::plot_frame_image() {
  cv::Mat out_img;
  cv::drawKeypoints(current_frame_->image_left_.image(),
                    current_frame_->image_left_.keypoints(), out_img);
  return out_img;
}

void Viewer::draw_map_points() {
  for (auto &[keyframe_id, keyframe] : map_->active_keyframes_) {
    draw_frame(keyframe);
  }

  glPointSize(2);
  glBegin(GL_POINTS);

  for (auto &[landmark_id, landmark] : map_->active_landmarks_) {
    auto pos = landmark->position_;
    glColor3f(red_[0], red_[1], red_[2]);
    glVertex3d(pos[0], pos[1], pos[2]);
  }
  glEnd();
}
