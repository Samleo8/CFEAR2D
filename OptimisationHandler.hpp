/**
 * @file OptimisationHandler.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-06-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __OPTIMISATION_HANDLER_HPP__
#define __OPTIMISATION_HANDLER_HPP__

#include "Keyframe.hpp"
#include "PoseTransformHandler.hpp"
#include "RadarImage.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>

#include <ceres/ceres.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define ANGLE_RAD_TO_DEG (180.0 / M_PI)
#define ANGLE_DEG_TO_RAD (M_PI / 180.0)

// TODO: Integrate with ceres somehow
// TOOD: 2D for now
template <typename T> struct OptimParams {
    // Eigen::Quaterniond q;     ///< rotation
    Eigen::Matrix<T, 2, 1> translation; ///< translation
    T theta;                            ///< rotation
};

/** @brief Keyframe buffer size */
const size_t KF_BUFF_SIZE = 3;

/** @brief Keyframe buffer typedef */
template <typename T>
using KeyframeBuffer = boost::circular_buffer<Keyframe<T>>;

/** @brief Angle tolerance threshold in radians */
const double ANGLE_TOLERANCE_RAD = 30 * ANGLE_DEG_TO_RAD;

/** @brief (Default) Delta threshold for Huber loss */
const double HUBER_DELTA_DEFAULT = 0.1;

// TODO: Create a cost functor for computing cost function. See
// http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres20AutoDiffCostFunctionE

template <typename T> T constrainAngle(const T &aAngleRad);

template <typename T>
const double angleBetweenVectors(const VectorXT<T> &aVec1,
                                 const VectorXT<T> &aVec2);

template <typename T>
const PoseTransform2D<T>
transformFromOptimParams(const struct OptimParams<T> &aParams);

template <typename T> const double HuberLoss(const T &a, const T &delta);

#endif // __OPTIMISATION_HANDLER_HPP__
