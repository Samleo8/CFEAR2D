/**
 * @file OptimisationHandler.tpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Implementation file for templated functions related to optimization
 * and cost functions
 * @version 0.1
 * @date 2022-06-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __OPTIMISATION_HANDLER_TPP__
#define __OPTIMISATION_HANDLER_TPP__

/**
 * @brief Constrain angle in radians between [-pi and pi)
 * @ref
 * https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam/pose_graph_2d/normalize_angle.h
 *
 * @tparam T Scalar type, used by ceres
 * @param[in] aAngleRad
 * @return T
 */
template <typename T> T constrainAngle(const T &aAngleRad) {
    // Use ceres::floor because it is specialized for double and Jet types.
    T two_pi(2.0 * M_PI);
    return aAngleRad - two_pi * ceres::floor((aAngleRad + T(M_PI)) / two_pi);
}

/**
 * @brief Obtain the angle between two vectors. Used for comparing angle
 * tolerance between normals of associated points
 *
 * @param[in] aVec1 Vector 1 of arbitrary dimenion k
 * @param[in] aVec2 Vector 2 of arbitrary dimenion k
 * @return Constrained Angle between the two vectors in radians
 */
template <typename T, int Dimension>
const T angleBetweenVectors(const VectorDimT<T, Dimension> &aVec1,
                            const VectorDimT<T, Dimension> &aVec2) {
    T unnormalizedAngle =
        ceres::acos(aVec1.dot(aVec2) / (aVec1.norm() * aVec2.norm()));

    return constrainAngle<T>(unnormalizedAngle);
}

/**
 * @brief Huber loss according to formula
 * @see https://en.wikipedia.org/wiki/Huber_loss
 *
 * @param[in] a Value
 * @param[in] delta threshold for Huber loss
 * @return Huber loss
 */
template <typename T> const T HuberLoss(const T &a, const T &delta) {
    if (ceres::abs(a) < delta) {
        return 0.5 * a * a;
    }
    else {
        return delta * (ceres::abs(a) - 0.5 * delta);
    }
}

#endif // __OPTIMISATION_HANDLER_TPP__