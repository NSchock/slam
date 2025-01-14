#ifndef BACKEND_H
#define BACKEND_H

/**
 * SLAM Backend: Optimize motion estimates from frontend to construct graph.
 */
#include "camera.h"
#include "map.h"
#include <memory>

class Backend {
public:
  void optimize();

public:
  std::shared_ptr<Map> map_;
  std::shared_ptr<Camera> left_camera_;
  std::shared_ptr<Camera> right_camera_;
};

#endif
