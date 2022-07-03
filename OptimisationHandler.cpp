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

/**
 * @brief Get pose transform from optimisation parameters, currently a pose
 *
 * @param[in] aParams Optimisation parameters, basically a pose. @see
 * OptimParams struct
 * @return Pose transform from optimisation parameters
 */
const PoseTransform2D transformFromOptimParams(const OptimParams &aParams) {
    return rotTransToTransform(aParams.theta, aParams.translation);
}

/**
 * @brief Huber loss according to formula
 * @see https://en.wikipedia.org/wiki/Huber_loss
 *
 * @param[in] a Value
 * @param[in] delta threshold for Huber loss
 * @return Huber loss
 */
const double HuberLoss(const double a, const double delta) {
    if (std::abs(a) < delta) {
        return 0.5 * a * a;
    }
    else {
        return delta * (std::abs(a) - 0.5 * delta);
    }
}