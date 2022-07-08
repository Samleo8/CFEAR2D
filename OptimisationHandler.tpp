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

#include "RegistrationCostFunctor.hpp"

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
 * @deprecated Now using Ceres Huber loss function instead
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

/**
 * @brief Find the closest feature point in a list of feature points to a
 * given point
 * @note User needs to check if a feature point was found; otherwise, the
 * aClosestORSPPoint can give garbage values.
 *
 * @pre All feature points must be in the same coordinate system for outputs to
 * make sense
 *
 * @tparam CastType Type to cast to
 * @param[in] aORSPPoint ORSP Point (WTF point in set closest to this)
 * @param[in] aORSPFeaturePoints Set of feature points to search in
 * @param[out] aClosestORSPPoint Closest ORSP Point (from keyframe)
 * @return Whether a closest feature point was found
 */
template <typename CastType>
const bool findClosestORSPInSet(const ORSP<CastType> &aORSPPoint,
                                const ORSPVec<double> &aORSPFeaturePoints,
                                ORSP<CastType> &aClosestORSPPoint) {
    // Init stuff
    bool found = false;
    const CastType ANGLE_TOLERANCE_RAD_CASTED =
        static_cast<CastType>(ANGLE_TOLERANCE_RAD);

    // NOTE: Closest distance set to radius because we only want points at a
    // close enough distance
    CastType closestDistance = static_cast<CastType>(ORSP_RADIUS);

    // center and normal vector of reference ORSP to find closest point against
    const Vector2T<CastType> centerPoint = aORSPPoint.center;
    const Vector2T<CastType> normalVec = aORSPPoint.normal;

    // Find the minimum distance, but we need to loop through to check if the
    // point is even valid because of angle tolerances
    for (size_t i = 0, sz = aORSPFeaturePoints.size(); i < sz; i++) {
        // Get potential closest point
        const ORSP<double> &potentialClosestPoint = aORSPFeaturePoints[i];

        // Obtain casted version of center, normal of ORSP point
        Vector2T<CastType> potentialCenter =
            potentialClosestPoint.center.template cast<CastType>();
        Vector2T<CastType> potentialNormal =
            potentialClosestPoint.normal.template cast<CastType>();

        // Check angle tolerance
        const CastType angle =
            angleBetweenVectors<CastType, Keyframe::DIMENSION>(potentialNormal,
                                                               normalVec);

        if (ceres::abs(angle) > ANGLE_TOLERANCE_RAD_CASTED) continue;

        // Calculate distance between potential closest point, and
        // check if it is indeed the closest point
        CastType dist = getDistance<CastType>(potentialCenter, centerPoint);

        if (dist < closestDistance) {
            closestDistance = dist;
            aClosestORSPPoint.center = potentialCenter;
            aClosestORSPPoint.normal = potentialNormal;

            found = true;
        }
    }

    return found;
}

#endif // __OPTIMISATION_HANDLER_TPP__