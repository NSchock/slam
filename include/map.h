#ifndef MAP_H
#define MAP_H

#include "frame.h"
#include <memory>
#include <unordered_map>

/**
 * SLAM Map: Track the landmarks and keyframes.
 */
class Map {
public:
  /**
   * Inserts the given frame as a new active keyframe. This also calls
   * insert_landmarks(), inserting all landmarks of the given frame as active
   * landmarks.
   *
   * If insertion causes there to be too many active keyframes, then also calls
   * remove_keyframe() to remove a previous keyframe.
   */
  void insert_keyframe(std::shared_ptr<Frame> frame);

  /**
   * Inserts the given landmark as an active landmark.
   */
  void insert_landmark(std::shared_ptr<Landmark> landmark);

  /**
   * Inserts the given collection of landmarks as active landmarks.
   */
  void insert_landmarks(std::vector<std::shared_ptr<Landmark>> landmarks);

  /**
   * Removes an old keyframe from the map.
   * The keyframe removed is either the farthest from the current keyframe, or
   * the closest if there is one that is very close.
   */
  void remove_keyframe();

  /**
   * Removes from the collection of active landmarks those landmarks that are no
   * longer visible from any features.
   */
  void cleanup_landmarks();

private:
  std::unordered_map<unsigned long, std::shared_ptr<Frame>> keyframes_;
  std::unordered_map<unsigned long, std::shared_ptr<Frame>> active_keyframes_;
  std::unordered_map<unsigned long, std::shared_ptr<Landmark>> landmarks_;
  std::unordered_map<unsigned long, std::shared_ptr<Landmark>>
      active_landmarks_;

  int max_active_keyframes_ = 10;

  unsigned long current_keyframe_id_ = -1;
};

#endif
