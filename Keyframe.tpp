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
 * @tparam CastType Type to cast to
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

    // Convert point to appropriate grid coordinate
    const PointCart2D centerPoint(aORSPPoint.center);

    PointCart2D gridCoord;

    pointToGridCoordinate(centerPoint, gridCoord, mGridCenter);

    // Check around the grid square but only up to sampling factor
    const ssize_t gridX = static_cast<ssize_t>(gridCoord[0]);
    const ssize_t gridY = static_cast<ssize_t>(gridCoord[1]);
    const ssize_t f = static_cast<ssize_t>(ORSP_RESAMPLE_FACTOR);
    const ssize_t N = static_cast<ssize_t>(ORSP_KF_GRID_N);

    for (ssize_t dx = -f; dx <= f; dx++) {
        // Bounds check: X
        ssize_t neighGridX = gridX + dx;
        if (neighGridX < 0 || neighGridX >= N) continue;

        for (ssize_t dy = -f; dy <= f; dy++) {
            // Bounds check: Y
            ssize_t neighGridY = gridY + dy;
            if (neighGridY < 0 || neighGridY >= N) continue;

            // Now look through all filtered points in neighbours
            const IndexList &potentialClosestPointIndices =
                mORSPIndexGrid[neighGridX][neighGridY];

            // No potential closest point, continue
            if (potentialClosestPointIndices.size() == 0) continue;

            // Loop through all potential closest points
            for (size_t i = 0; i < potentialClosestPointIndices.size(); i++) {
                // Get potential closest point
                const ORSP<double> &potentialClosestPoint =
                    mORSPFeaturePoints[potentialClosestPointIndices[i]];

                const PointCart2D potentialClosestPointCart(
                    potentialClosestPoint.center);

                // Check angle tolerance
                const double angle = angleBetweenVectors<double>(
                    potentialClosestPoint.normal, aClosestORSPPoint.normal);

                if (ABS(angle) > ANGLE_TOLERANCE_RAD) continue;

                // Calculate distance between potential closest point, and
                // check if it is indeed the closest point
                double dist = centerPoint.distance(potentialClosestPointCart);
                if (dist < closestDistance) {
                    closestDistance = dist;
                    aClosestORSPPoint =
                        potentialClosestPoint; // should be ok if reference
                                               // since persistent
                    found = true;
                }
            }
        }
    }

    return found;
}

#endif // __CFEAR_KEYFRAME_TPP__