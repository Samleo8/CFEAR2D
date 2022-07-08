/**
 * @file OptimisationHandler.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for optimisation functions, including the building and solving
 * of registration and motion distortion optimisation problems.
 * Notably, @see buildAndSolveRegistrationProblem()
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
#include "TransformDefines.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <ceres/ceres.h>

/** @brief Keyframe buffer size */
const size_t KF_BUFF_SIZE = 3;

/** @brief Angle tolerance threshold in radians */
const double ANGLE_TOLERANCE_RAD = 30 * ANGLE_DEG_TO_RAD;

/** @brief (Default) Delta threshold for Huber loss */
const double HUBER_DELTA_DEFAULT = 0.1;

// Problem builder functions. Implemented in OptimisationHandler.cpp
void buildPoint2LineProblem(ceres::Problem *aProblem,
                                  ceres::LossFunction *aLossFnPtr,
                                  const RadarImage &aRImage,
                                  const ORSPVec<double> &aKeyframeFeaturePoints,
                                  double *positionArr, double *orientationArr);

[[nodiscard]] const bool
buildAndSolveRegistrationProblem(const RadarImage &aRImage,
                                 const KeyframeBuffer &aKFBuffer,
                                 Pose2D<double> &aPose);

// Templated helper functions. Implemented in OptimisationHandler.tpp
template <typename T> T constrainAngle(const T &aAngleRad);

template <typename T, int Dimension = 2>
const T angleBetweenVectors(const VectorDimT<T, Dimension> &aVec1,
                            const VectorDimT<T, Dimension> &aVec2);

template <typename T> const T HuberLoss(const T &a, const T &delta);

template <typename CastType>
[[nodiscard]] const bool
findClosestORSPInSet(const ORSP<CastType> &aORSPPoint,
                     const ORSPVec<double> &aORSPFeaturePoints,
                     ORSP<CastType> &aClosestORSPPoint);

// Implementation file for optimisation handler functions
#include "OptimisationHandler.tpp"

#endif // __OPTIMISATION_HANDLER_HPP__
