/**
 * @file OptimisationHandler.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Functions related to optimization and cost functions for bundle
 * adjustment
 * @version 0.1
 * @date 2022-06-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "OptimisationHandler.hpp"

const double constrainAngle(const double aAngleRad) {
    double constAngle = std::fmod(aAngleRad + M_PI, M_TAU);
    if (constAngle < 0) constAngle += M_TAU;
    return constAngle - M_PI;
}

/**
 * @brief Obtain the angle between two vectors. Used for comparing angle
 * tolerance between normals of associated points
 *
 * @param[in] aVec1 Vector 1 of arbitrary dimenion k
 * @param[in] aVec2 Vector 2 of arbitrary dimenion k
 * @return Constrained Angle between the two vectors in radians
 */
const double angleBetweenVectors(const Eigen::VectorXd &aVec1,
                                 const Eigen::VectorXd &aVec2) {
    double unnormalizedAngle =
        acos(aVec1.dot(aVec2) / (aVec1.norm() * aVec2.norm()));

    return constrainAngle(unnormalizedAngle);
}

const PoseTransform2D transformFromOptimParams(const OptimParams &aParams) {
    return rotTransToTransform(aParams.theta, aParams.translation);
}