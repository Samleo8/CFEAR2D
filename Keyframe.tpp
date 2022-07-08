/**
 * @file Keyframe.tpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Template implementation of certain class functions
 * @version 0.1
 * @date 2022-07-06
 *
 * @copyright Copyright (c) 2022
 *
 */
#ifndef __CFEAR_KEYFRAME_TPP__
#define __CFEAR_KEYFRAME_TPP__

#include <ceres/jet.h> // for functions like abs, max

// Externally declare, otherwise run into circular dependency
template <typename T, int Dimension>
extern const T angleBetweenVectors(const VectorDimT<T, Dimension> &aVec1,
                            const VectorDimT<T, Dimension> &aVec2);

extern const double ANGLE_TOLERANCE_RAD;

/**
 * @brief Find the closest feature point to a given point in world
 * coordinates
 * @note User needs to check if a feature point was found; otherwise, the
 * aClosestORSPPoint can give garbage values.
 *
 * @tparam CastType Type to cast to
 * @param[in] aORSPPoint ORSP Point (WTF Keyframe point closest to this),
 * world coordinates
 * @param[out] aClosestORSPPoint Closest ORSP Point (from keyframe), world
 * coordinates
 * @return Whether a closest feature point was found
 */
template <typename CastType>
const bool Keyframe::findClosestORSP(const ORSP<CastType> &aORSPPoint,
                                     ORSP<CastType> &aClosestORSPPoint) const {
    // Init stuff
    bool found = false;
    CastType ANGLE_TOLERANCE_RAD_CASTED =
        static_cast<CastType>(ANGLE_TOLERANCE_RAD);

    // NOTE: Closest distance set to radius because we only want points at a
    // close enough distance
    CastType closestDistance = static_cast<CastType>(ORSP_RADIUS);

    // center and normal vector of reference ORSP to find closest point against
    const Vector2T<CastType> centerPoint = aORSPPoint.center;
    const Vector2T<CastType> normalVec = aORSPPoint.normal;

    // Find the minimum distance, but we need to loop through to check if the
    // point is even valid because of angle tolerances
    for (size_t i = 0, sz = mORSPFeaturePoints.size(); i < sz; i++) {
        // Get potential closest point
        const ORSP<double> &potentialClosestPoint = mORSPFeaturePoints[i];

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

#endif // __CFEAR_KEYFRAME_TPP__