#ifndef DATASET_H
#define DATASET_H

#include "camera.h"
#include <memory>
#include <string>

class Dataset {
public:
  Dataset(std::string dataset_path) : dataset_path_(std::move(dataset_path)) {}

  bool read_calib_data();

  // std::shared_ptr<Frame> next_frame() const;

  std::shared_ptr<Camera> get_camera(const std::string &camera_id) const {
    return cameras_.at(camera_id);
  }

  std::shared_ptr<Camera> get_camera(std::string &&camera_id) const {
    return cameras_.at(camera_id);
  }

private:
  std::string dataset_path_;
  int current_image_index_ = 0;
  std::unordered_map<std::string, std::shared_ptr<Camera>> cameras_;
};

#endif
