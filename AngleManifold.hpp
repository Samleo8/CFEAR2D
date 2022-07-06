/**
 * @file AngleManifold.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Angle manifold (single angle variable)
 * @ref
 * https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam/pose_graph_2d/angle_manifold.h
 * @version 0.1
 * @date 2022-07-05
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __ANGLE_MANIFOLD_HPP__
#define __ANGLE_MANIFOLD_HPP__

#include "ceres/autodiff_manifold.h"
#include "OptimisationHandler.hpp"

// Defines a manifold for updating the angle to be constrained in [-pi to pi).
class AngleManifold {
 public:
  template <typename T>
  bool Plus(const T* x_radians,
            const T* delta_radians,
            T* x_plus_delta_radians) const {
    *x_plus_delta_radians = constrainAngle(*x_radians + *delta_radians);
    return true;
  }

  template <typename T>
  bool Minus(const T* y_radians,
             const T* x_radians,
             T* y_minus_x_radians) const {
    *y_minus_x_radians =
        constrainAngle(*y_radians) - constrainAngle(*x_radians);

    return true;
  }

  static ceres::Manifold* Create() {
    return new ceres::AutoDiffManifold<AngleManifold, 1, 1>;
  }
};

#endif  // __ANGLE_MANIFOLD_HPP__