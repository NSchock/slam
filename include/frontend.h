#ifndef FRONTEND_H
#define FRONTEND_H

#include "frame.h"
#include <memory>

/**
 * SLAM Frontend: Perform visual odometry to estimate camera motion.
 */
class Frontend {

public:
  /**
   * Processes the given frame as the next frame in the system.
   *
   * TODO: not implemented, currently I've set it to just display the left/right
   * images for demonstration purposes.
   */
  bool add_frame(std::shared_ptr<Frame> frame);
};

#endif
