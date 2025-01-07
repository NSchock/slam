#include "map.h"
#include "landmark.h"
#include <vector>

void Map::insert_keyframe(std::shared_ptr<Frame> frame) {
  keyframes_[frame->id_] = frame;
  active_keyframes_[frame->id_] = frame;

  insert_landmarks(frame->landmarks_);

  if (active_keyframes_.size() > max_active_keyframes_) {
    remove_keyframe();
  }
}

void Map::insert_landmark(std::shared_ptr<Landmark> landmark) {
  landmarks_[landmark->id_] = landmark;
  active_landmarks_[landmark->id_] = landmark;
}

void Map::insert_landmarks(std::vector<std::shared_ptr<Landmark>> landmarks) {
  for (const auto &landmark : landmarks) {
    insert_landmark(landmark);
  }
}

void Map::remove_keyframe() {
  if (current_keyframe_id_ == -1) {
    return;
  }
  // find the keyframes at the max and min distance from the current frame
  double max_dist = 0, min_dist = 999999;
  unsigned long max_keyframe_id_ = -1, min_keyframe_id_ = -1;
  // inverse_pose = transformation from camera to world coordinates at current
  // keyframe
  auto inverse_pose = active_keyframes_[current_keyframe_id_]->pose_.inverse();
  for (const auto &[id, frame] : active_keyframes_) {
    if (id == current_keyframe_id_) {
      continue;
    }
    auto dist = (frame->pose_ * inverse_pose).log().norm();
    if (dist > max_dist) {
      max_dist = dist;
      max_keyframe_id_ = id;
    }
    if (dist < min_dist) {
      min_dist = dist;
      min_keyframe_id_ = id;
    }

    // remove the closest keyframe if it is very close (distance within
    // min_threshold) to current one, otherwise remove farthest keyframe
    const double min_threshold = 0.2;
    unsigned long frame_to_remove_id =
        min_dist < min_threshold ? min_keyframe_id_ : max_keyframe_id_;
    auto frame_to_remove = active_keyframes_[frame_to_remove_id];
    for (const auto &feature : frame_to_remove->left_features()) {
      if (auto landmark = feature->landmark_.lock()) {
        landmark->remove_observation(feature);
      }
    }
    for (const auto &feature : frame_to_remove->right_features()) {
      if (auto landmark = feature->landmark_.lock()) {
        landmark->remove_observation(feature);
      }
    }

    active_keyframes_.erase(frame_to_remove_id);

    cleanup_landmarks();
  }
}

void Map::cleanup_landmarks() {
  for (auto it = active_landmarks_.begin(); it != active_landmarks_.end();) {
    if (it->second->num_observations() == 0) {
      it = active_landmarks_.erase(it);
    } else {
      ++it;
    }
  }
}
