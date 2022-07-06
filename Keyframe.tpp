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

/**
 * @brief Find the closest feature point to a given point in world
 * coordinates
 * @note User needs to check if a feature point was found; otherwise, the
 * aClosestORSPPoint can give garbage values.
 *
 * @tparam T Type to cast to
 * @param[in] aORSPPoint
 * @param[in] aClosestORSPPoint
 * @return Whether a closest feature point was found
 */
template <typename CastType>
const bool Keyframe::findClosestORSP(const ORSP<CastType> &aORSPPoint,
                                     ORSP<CastType> &aClosestORSPPoint) const {
    // Init stuff
    bool found = false;
    double closestDistance = ORSP_RADIUS;

    // Convert point to appropriate grid coordinate, casting to T type (for Ceres)
    const Vector2T<CastType> centerPoint = aORSPPoint.center.template cast<CastType>();

    // Cast the matrix of ORSP centers to be of type T (for Ceres)
    Eigen::Matrix<CastType, Keyframe::DIMENSION, Eigen::Dynamic> ORSPCenters =
        mORSPCenters.template cast<CastType>();

    // // Loop through all potential closest points
    // for (size_t i = 0; i < potentialClosestPointIndices.size(); i++) {
    //     // Get potential closest point
    //     const ORSP<double> &potentialClosestPoint =
    //         mORSPFeaturePoints[potentialClosestPointIndices[i]];

    //     const PointCart2D potentialClosestPointCart(
    //         potentialClosestPoint.center);

    //     // Check angle tolerance
    //     const double angle = angleBetweenVectors<double>(
    //         potentialClosestPoint.normal, aClosestORSPPoint.normal);

    //     if (ABS(angle) > ANGLE_TOLERANCE_RAD) continue;

    //     // Calculate distance between potential closest point, and
    //     // check if it is indeed the closest point
    //     double dist = centerPoint.distance(potentialClosestPointCart);
    //     if (dist < closestDistance) {
    //         closestDistance = dist;
    //         aClosestORSPPoint =
    //             potentialClosestPoint; // should be ok if reference
    //                                     // since persistent
    //         found = true;
    //     }
    // }

    return found;
}

#endif // __CFEAR_KEYFRAME_TPP__