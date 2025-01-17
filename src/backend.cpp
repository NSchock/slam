#include "backend.h"
#include "g2o_types.h"
#include "landmark.h"
#include <algorithm>
#include <ceres/autodiff_cost_function.h>
#include <ceres/ceres.h>
#include <ceres/problem.h>
#include <ceres/rotation.h>
#include <ceres/solver.h>
#include <ceres/types.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/linear_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/eigen/linear_solver_eigen.h>
#include <g2o/types/slam3d/parameter_se3_offset.h>
#include <map>
#include <memory>

// Ceres optimization crashes and I'm not sure why. Leaving it in for now, will
// fix or delete later.
struct ReprojectionError {
  ReprojectionError(double observed_x, double observed_y)
      : observed_x_{observed_x}, observed_y_{observed_y} {}

  template <typename T>
  bool operator()(const T *const extrinsic, const T *const intrinsic,
                  const T *const point, T *residuals) const {
    T q[4];
    q[0] = extrinsic[3];
    q[1] = extrinsic[0];
    q[2] = extrinsic[1];
    q[3] = extrinsic[2];
    // std::cout << "q: " << q[0] << " " << q[1] << " " << q[2] << " " << q[3]
    // << "\n";

    // std::cout << "point input: " << point[0] << " " << point[1] << " "
    //           << point[2] << "\n";

    T p[3];
    ceres::UnitQuaternionRotatePoint(q, point, p);
    p[0] += extrinsic[4];
    p[1] += extrinsic[5];
    p[2] += extrinsic[6];
    // std::cout << "point : " << p[0] << " " << p[1] << " " << p[2] << "\n";

    const T predicted_x = intrinsic[0] * p[0] / p[2] + intrinsic[2];
    const T predicted_y = intrinsic[1] * p[1] / p[2] + intrinsic[3];

    residuals[0] = predicted_x - T(observed_x_);
    residuals[1] = predicted_y - T(observed_y_);

    // std::cout << "extrinsic: " << extrinsic << "\n";
    // std::cout << "intrinsic: " << intrinsic << "\n";
    // std::cout << "point: " << point << "\n";
    // std::cout << "residuals: " << residuals << "\n";
    return true;
  }

  static ceres::CostFunction *Create(const double observed_x,
                                     const double observed_y) {
    // 2 = projected 2d point
    // 7 = quaternion followed by translation, giving camera pose (extrinsic)
    // 4 = f_x, f_y, c_x, c_y, giving camera intrinsic parameters
    // 3 = 3d world point
    return new ceres::AutoDiffCostFunction<ReprojectionError, 2, 7, 4, 3>(
        new ReprojectionError(observed_x, observed_y));
  }

  double observed_x_, observed_y_;
};

void Backend::optimize_ceres() {
  ceres::Problem problem;

  for (const auto &[landmark_id, landmark] : map_->active_landmarks_) {
    for (const auto &[observation_id, observation] : landmark->observations_) {
      if (auto frame = observation->frame_.lock()) {
        if (map_->active_keyframes_.contains(frame->id_)) {
          ceres::CostFunction *cost_function =
              ReprojectionError::Create(observation->feature_left_->point().x,
                                        observation->feature_left_->point().y);
          // std::cout << "original point input: " << landmark->position_ <<
          // "\n";
          // std::cout << "original point: "
          //<< frame->pose_world_to_camera() * landmark->position_ << "\n";
          // std::cout << "frame: " << frame->pose_world_to_camera().data()
          //          << "\n";
          // std::cout << "camera: " << camera_left_->intrinsic_data().data()
          //          << "\n";
          // std::cout << "landmark: " << landmark->position_.data() << "\n";
          problem.AddResidualBlock(cost_function, nullptr,
                                   frame->pose_world_to_camera().data(),
                                   camera_left_->intrinsic_data().data(),
                                   landmark->position_.data());
        }
      }
    }
  }

  ceres::Solver::Options options;
  options.linear_solver_type = ceres::SPARSE_SCHUR;
  options.minimizer_progress_to_stdout = true;
  ceres::Solver::Summary summary;
  ceres::Solve(options, &problem, &summary);
  std::cout << summary.FullReport() << "\n";
}

void Backend::optimize() {
  using BlockSolverType = g2o::BlockSolver_6_3;
  using LinearSolverType =
      g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>;

  g2o::SparseOptimizer optimizer;

  auto linear_solver = std::make_unique<LinearSolverType>();
  linear_solver->setBlockOrdering(false);
  g2o::OptimizationAlgorithmLevenberg *solver =
      new g2o::OptimizationAlgorithmLevenberg(
          std::make_unique<BlockSolverType>(std::move(linear_solver)));

  optimizer.setAlgorithm(solver);

  std::map<unsigned long, VertexPose *> vertices_poses;
  unsigned long max_keyframe_id = 0;
  for (const auto &[keyframe_id, keyframe] : map_->active_keyframes_) {
    VertexPose *vertex_pose = new VertexPose();
    vertex_pose->setId(keyframe_id);
    vertex_pose->setEstimate(keyframe->pose_world_to_camera());
    max_keyframe_id = std::max(keyframe_id, max_keyframe_id);
    optimizer.addVertex(vertex_pose);
    vertices_poses.insert({keyframe_id, vertex_pose});
  }
  std::cout << "added vertices\n";

  std::map<unsigned long, VertexLandmark *> vertices_landmarks;

  double chi2_th = 5.991;

  int idx = 1;
  // std::map<EdgeProjection *, std::shared_ptr<Observation>>
  // edges_and_observations;

  for (const auto &[landmark_id, landmark] : map_->active_landmarks_) {
    VertexLandmark *vertex_landmark = new VertexLandmark();
    vertex_landmark->setId(landmark_id + max_keyframe_id + 1);
    vertex_landmark->setEstimate(landmark->position_);
    vertex_landmark->setMarginalized(true);
    optimizer.addVertex(vertex_landmark);
    vertices_landmarks.insert({landmark_id, vertex_landmark});
    for (const auto &[observation_id, observation] : landmark->observations_) {
      if (auto frame = observation->frame_.lock()) {
        EdgeProjection *edge = new EdgeProjection(camera_left_, camera_right_);
        edge->setId(idx++);
        edge->setVertex(0, vertices_poses.at(frame->id_));
        // std::cout << frame->id_ << "\n";
        // std::cout << vertices_poses.at(frame->id_) << "\n";
        edge->setVertex(1, vertex_landmark);
        Eigen::Vector4d measurement;
        measurement << observation->feature_left_->point().x,
            observation->feature_left_->point().y,
            observation->feature_right_->point().x,
            observation->feature_right_->point().y;
        edge->setMeasurement(measurement);
        edge->setInformation(Eigen::Matrix4d::Identity());
        g2o::RobustKernelHuber *rk = new g2o::RobustKernelHuber();
        rk->setDelta(chi2_th);
        edge->setRobustKernel(rk);
        optimizer.addEdge(edge);
        // edges_and_observations.insert({edge, observation});
      }
    }
  }
  std::cout << "added edges\n";

  optimizer.setVerbose(true);
  optimizer.initializeOptimization();
  std::cout << "initialized optimization\n";
  optimizer.optimize(10);
  std::cout << "optimized\n";

  for (auto &[id, vertex_pose] : vertices_poses) {
    map_->active_keyframes_.at(id)->set_pose_world_to_camera(
        vertex_pose->estimate());
  }
  for (auto &[id, vertex_landmark] : vertices_landmarks) {
    map_->active_landmarks_.at(id)->position_ = vertex_landmark->estimate();
  }
}
