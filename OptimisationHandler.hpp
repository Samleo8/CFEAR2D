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
typedef struct {
    // Eigen::Quaterniond q;     ///< rotation
    Eigen::Vector2d translation; ///< translation
    double theta;                ///< rotation
} OptimParams;

const size_t KF_BUFF_SIZE = 3;
typedef boost::circular_buffer<Keyframe> KeyframeBuffer;
// typedef std::vector<Keyframe> KeyframeBuffer;

/** @brief Angle tolerance threshold in radians */
const double ANGLE_TOLERANCE_RAD = 30 * ANGLE_DEG_TO_RAD;

/** @brief (Default) Delta threshold for Huber loss */
const double HUBER_DELTA_DEFAULT = 0.1;

// TODO: Create a cost functor for computing cost function. See
// http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres20AutoDiffCostFunctionE

const double constrainAngle(const double aAngleRad);

const double angleBetweenVectors(const Eigen::VectorXd &aVec1,
                                 const Eigen::VectorXd &aVec2);

const PoseTransform2D transformFromOptimParams(const OptimParams &aParams);

const double HuberLoss(const double a,
                       const double delta = HUBER_DELTA_DEFAULT);

#endif // __OPTIMISATION_HANDLER_HPP__
