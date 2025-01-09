#ifndef VIEWER_H
#define VIEWER_H

#include "frame.h"
#include "map.h"
#include <pangolin/display/view.h>
#include <pangolin/gl/opengl_render_state.h>
#include <thread>

/**
 * SLAM Viewer: Responsible for displaying the results of the SLAM program.
 */
class Viewer {
public:
  Viewer();

  void close();

  void set_map(std::shared_ptr<Map> map) { map_ = std::move(map); }

  void add_current_frame(std::shared_ptr<Frame> frame);

  void thread_loop();

  void draw();

  void draw_frame(std::shared_ptr<Frame> frame,
                  const std::array<float, 3> color = red_);

  void follow_current_frame(pangolin::OpenGlRenderState &vis_camera);

  cv::Mat plot_frame_image();

  void draw_map_points();

private:
  // std::thread viewer_thread_;
  // std::mutex viewer_data_mutex_;

  bool viewer_running_ = true;

  std::shared_ptr<Frame> current_frame_;
  std::shared_ptr<Map> map_;

  inline static const std::array<float, 3> red_{1, 0, 0};
  inline static const std::array<float, 3> green_{0, 1, 0};
  inline static const std::array<float, 3> blue_{0, 0, 1};

  pangolin::OpenGlRenderState vis_camera_;
  pangolin::View vis_display_;
};

#endif
