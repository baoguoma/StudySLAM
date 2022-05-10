/*
 * <one line to give the program's name and a brief idea of what it does.>
 * Copyright (C) 2016  <copyright holder> <email>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#ifndef STUDYSLAM_G2O_TYPES_H
#define STUDYSLAM_G2O_TYPES_H

#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_vertex.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/solvers/dense/linear_solver_dense.h>
#include <g2o/types/sba/types_six_dof_expmap.h>

#include "camera.h"
#include "studyslam/common_include.h"

namespace studyslam {
class EdgeProjectXYZRGBD
    : public g2o::BaseBinaryEdge<3, Eigen::Vector3d, g2o::VertexPointXYZ,
                                 g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
  virtual void computeError();
  virtual void linearizeOplus();
  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& out) const {}
};

// only to optimize the pose, no point
class EdgeProjectXYZRGBDPoseOnly
    : public g2o::BaseUnaryEdge<3, Eigen::Vector3d, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // Error: measure = R*point+t
  virtual void computeError();
  virtual void linearizeOplus();

  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& out) const {}

  Vector3d point_;
};

class EdgeProjectXYZ2UVPoseOnly
    : public g2o::BaseUnaryEdge<2, Eigen::Vector2d, g2o::VertexSE3Expmap> {
 public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  virtual void computeError();
  virtual void linearizeOplus();

  virtual bool read(std::istream& in) {}
  virtual bool write(std::ostream& os) const {};

  Vector3d point_;
  Camera* camera_;
};

}  // namespace studyslam

#endif  // studySLAM_G2O_TYPES_H
