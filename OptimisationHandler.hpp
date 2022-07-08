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

#include "AngleManifold.hpp"
#include "Keyframe.hpp"
#include "ORSP.hpp"
#include "PoseTransformHandler.hpp"
#include "RadarImage.hpp"
#include "RegistrationCostFunctor.hpp"
#include "TransformDefines.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <ceres/ceres.h>

#define _USE_MATH_DEFINES
#include <math.h>

#define ANGLE_RAD_TO_DEG (180.0 / M_PI)
#define ANGLE_DEG_TO_RAD (M_PI / 180.0)

/** @brief Keyframe buffer size */
const size_t KF_BUFF_SIZE = 3;

/** @brief Angle tolerance threshold in radians */
const double ANGLE_TOLERANCE_RAD = 30 * ANGLE_DEG_TO_RAD;

/** @brief (Default) Delta threshold for Huber loss */
const double HUBER_DELTA_DEFAULT = 0.1;

// TODO: Create a cost functor for computing cost function. See
// http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres20AutoDiffCostFunctionE

template <typename T> T constrainAngle(const T &aAngleRad);

template <typename T, int Dimension = 2>
const T angleBetweenVectors(const VectorDimT<T, Dimension> &aVec1,
                            const VectorDimT<T, Dimension> &aVec2);

template <typename T> const T HuberLoss(const T &a, const T &delta);

[[nodiscard]] const bool buildPoint2LineProblem(ceres::Problem *aProblem,
                                                const RadarImage &aRImage,
                                                const Keyframe &aKeyframe,
                                                Pose2D<double> &aPose);

// Implementation file for optimisation handler functions
#include "OptimisationHandler.tpp"

#endif // __OPTIMISATION_HANDLER_HPP__
