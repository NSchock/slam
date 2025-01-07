#ifndef SLAM_H
#define SLAM_H

#include "backend.h"
#include "camera.h"
#include "frontend.h"
#include "map.h"
#include "viewer.h"
#include <memory>

class Slam {
public:
  Slam(std::string dataset_path) : dataset_path_(std::move(dataset_path)) {}

  /**
   * Read calibration data from calib.txt and create camera objects.
   */
  bool read_calib_data();

  std::shared_ptr<Camera> get_camera(std::string camera_name) {
    return cameras_.at(camera_name);
  }

  /**
   * Initialize the SLAM system.
   */
  void initialize();

  /**
   * Process the next frame
   */
  bool step();

  /**
   * Run the SLAM system.
   */
  void run();

private:
  std::string dataset_path_;
  int current_frame_index_ = 0;
  std::unordered_map<std::string, std::shared_ptr<Camera>> cameras_;

  std::shared_ptr<Frontend> frontend_;
  std::shared_ptr<Backend> backend_;
  std::shared_ptr<Map> map_;
  std::shared_ptr<Viewer> viewer_;

  bool initialized_ = false;
};

#endif
