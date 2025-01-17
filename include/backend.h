#ifndef BACKEND_H
#define BACKEND_H

/**
 * SLAM Backend: Optimize motion estimates from frontend to construct graph.
 */
#include "camera.h"
#include "map.h"
#include <algorithm>
#include <memory>

class Backend {
public:
  Backend(std::shared_ptr<Camera> camera_left,
          std::shared_ptr<Camera> camera_right)
      : camera_left_{std::move(camera_left)},
        camera_right_{std::move(camera_right)} {}

  void optimize_ceres();

  void optimize();

  void set_map(std::shared_ptr<Map> map) { map_ = std::move(map); }

public:
  std::shared_ptr<Map> map_;
  std::shared_ptr<Camera> camera_left_, camera_right_;
};

#endif
