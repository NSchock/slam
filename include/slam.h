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

  bool initialize();

  bool read_calib_data();

  Camera get_camera(std::string camera_name) {
    return cameras_.at(camera_name);
  }

private:
  std::string dataset_path_;
  int current_image_index_ = 0;
  std::unordered_map<std::string, Camera> cameras_;

  std::shared_ptr<Frontend> frontend_;
  std::shared_ptr<Backend> backend_;
  std::shared_ptr<Map> map_;
  std::shared_ptr<Viewer> viewer_;
};

#endif
