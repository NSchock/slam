#ifndef G2O_TYPES_H
#define G2O_TYPES_H

#include "camera.h"
#include <Eigen/Core>
#include <g2o/core/base_binary_edge.h>
#include <g2o/core/base_vertex.h>
#include <sophus/se3.hpp>

/**
 * Class for pose vertex.
 * The vertex estimate is the world-to-camera pose of the corresponding frame.
 */
class VertexPose : public g2o::BaseVertex<6, Sophus::SE3d> {
public:
  virtual void setToOriginImpl() override { _estimate = Sophus::SE3d(); }

  /// left multiplication on SE3
  virtual void oplusImpl(const double *update) override {
    Eigen::Vector<double, 6> update_eigen;
    update_eigen << update[0], update[1], update[2], update[3], update[4],
        update[5];
    _estimate = Sophus::SE3d::exp(update_eigen) * _estimate;
  }

  virtual bool read(std::istream &in) override { return true; }

  virtual bool write(std::ostream &out) const override { return true; }
};

/**
 * Class for landmark vertex.
 * The vertex estimate is the 3d position in world coordinates of the
 * corresponding landmark.
 */
class VertexLandmark : public g2o::BaseVertex<3, Eigen::Vector3d> {
public:
  virtual void setToOriginImpl() override {
    _estimate = Eigen::Vector3d::Zero();
  }

  virtual void oplusImpl(const double *update) override {
    _estimate[0] += update[0];
    _estimate[1] += update[1];
    _estimate[2] += update[2];
  }

  virtual bool read(std::istream &in) override { return true; }

  virtual bool write(std::ostream &out) const override { return true; }
};

/**
 * Class for edge between a pose vertex and landmark vertex.
 * The edge measurement is the 4d vector obtained by concatenating the left and
 * right features of the corresponding observation.
 * The error measurement is calculated by comparing this with the 4d vector
 * formed by concatenating the projections of the 3d landmark point by the left
 * and right cameras.
 */
class EdgeProjection : public g2o::BaseBinaryEdge<4, Eigen::Vector4d,
                                                  VertexPose, VertexLandmark> {
public:
  EdgeProjection(std::shared_ptr<Camera> camera_left,
                 std::shared_ptr<Camera> camera_right)
      : camera_left_{std::move(camera_left)},
        camera_right_{std::move(camera_right)} {}

  virtual void computeError() override {
    const auto v0 = static_cast<VertexPose *>(_vertices[0]);
    const auto v1 = static_cast<VertexLandmark *>(_vertices[1]);
    Eigen::Vector4d world_point;
    world_point << v1->estimate().x(), v1->estimate().y(), v1->estimate().z(),
        1;
    auto pose_matrix = v0->estimate().matrix();
    Eigen::Vector3d point_left =
        camera_left_->projection_matrix() * pose_matrix * world_point;
    Eigen::Vector3d point_right =
        camera_right_->projection_matrix() * pose_matrix * world_point;
    point_left /= point_left.z();
    point_right /= point_right.z();
    Eigen::Vector4d projected_point;
    projected_point << point_left.x(), point_left.y(), point_right.x(),
        point_right.y();
    _error = _measurement - projected_point;
  }

  // virtual void linearizeOplus() override {
  //   const auto v0 = static_cast<VertexPose *>(_vertices[0]);
  //   const auto v1 = static_cast<VertexLandmark *>(_vertices[1]);
  //   Sophus::SE3d T = v0->estimate();
  //   Eigen::Vector3d pw = v1->estimate();

  //  Eigen::Vector3d pos_cam = _cam_ext * T * pw;
  //  double fx = _K(0, 0);
  //  double fy = _K(1, 1);
  //  double X = pos_cam[0];
  //  double Y = pos_cam[1];
  //  double Z = pos_cam[2];
  //  double Zinv = 1.0 / (Z + 1e-18);
  //  double Zinv2 = Zinv * Zinv;
  //  _jacobianOplusXi << -fx * Zinv, 0, fx * X * Zinv2, fx * X * Y * Zinv2,
  //      -fx - fx * X * X * Zinv2, fx * Y * Zinv, 0, -fy * Zinv, fy * Y *
  //      Zinv2, fy + fy * Y * Y * Zinv2, -fy * X * Y * Zinv2, -fy * X * Zinv;

  //  _jacobianOplusXj = _jacobianOplusXi.block<2, 3>(0, 0) *
  //                     _cam_ext.rotationMatrix() * T.rotationMatrix();
  //}

  virtual bool read(std::istream &in) override { return true; }

  virtual bool write(std::ostream &out) const override { return true; }

private:
  std::shared_ptr<Camera> camera_left_, camera_right_;
};

#endif
