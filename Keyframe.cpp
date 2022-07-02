/**
 * @file Keyframe.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for keyframes, containing class and relevant internal data
 * structures, and functions to register keyframes with the incoming scan
 * frames.
 *
 * Each keyframe has a list of feature points associated with this keyframe
 * a grid representation containing these feature points transferred
 * from RadarImage (in local coordinates), and a world pose
 *
 * @version 0.1
 * @date 2022-06-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "Keyframe.hpp"
#include "OptimisationHandler.hpp" // needed for angle handling
#include "PoseTransformHandler.hpp"

/**
 * @brief Constructor for Keyframe class. Handles transferring of relevant
 * data structures (specifically grid representations and points) from
 * RadarImage into the class
 * @todo To save memory, we can remove the need for saving the pose
 * transform since it is only used in this initialization
 *
 * @param[in] aRadarImage Reference to radar image to be used to construct
 * keyframe
 * @param[in] aWorldPose
 */
Keyframe::Keyframe(const RadarImage &aRadarImage, const Pose2D &aWorldPose)
    : mWorldPose(aWorldPose),
      mLocalToWorldTransform(poseToTransform(aWorldPose)),
      mWorldToLocalTransform(mLocalToWorldTransform.inverse()),
      mGridCenter(aWorldPose.position[0] + RADAR_MAX_RANGE_M_SQRT2,
                  aWorldPose.position[1] + RADAR_MAX_RANGE_M_SQRT2) {
    // Initialize feature points vector, which needs to be populated by
    // converted feature points from RadarImage to world coordinates
    const ORSPVec &ORSPFeaturePointsRef = aRadarImage.getORSPFeaturePoints();
    mORSPFeaturePoints.reserve(ORSPFeaturePointsRef.size());

    // TODO: Check if this is correct.
    // Loop through all ORSP feature points, convert to world coordinates and
    // convert to world coordinate
    for (size_t i = 0; i < ORSPFeaturePointsRef.size(); i++) {
        // First convert existing point to world coordinate
        ORSP worldORSPPoint;
        localToWorldORSP(ORSPFeaturePointsRef[i], worldORSPPoint);

        // Populate grid accordingly
        PointCart2D centerPoint(worldORSPPoint.center);
        PointCart2D gridCoord;

        pointToGridCoordinate(centerPoint, gridCoord, mGridCenter);

        // TODO: Check if gridCoord is in bounds

        size_t gridX = static_cast<size_t>(gridCoord.x);
        size_t gridY = static_cast<size_t>(gridCoord.y);
        mORSPIndexGrid[gridX][gridY].push_back(i);

        // Copy over ORSP point
        mORSPFeaturePoints.push_back(worldORSPPoint);
    }

    // TODO: Use Eigen::Map if necessary to (quickly) batch convert a set of
    // coordinates from world to local or vice versa
}

/**
 * @brief Copy Constructor for Keyframe. Needed for circular buffer
 * because of the use of const initializers
 *
 * @param[in] aKeyframe Input keyframe
 */
Keyframe::Keyframe(const Keyframe &aKeyframe)
    : mWorldPose(aKeyframe.mWorldPose),
      mLocalToWorldTransform(aKeyframe.mLocalToWorldTransform),
      mWorldToLocalTransform(aKeyframe.mWorldToLocalTransform),
      mGridCenter(aKeyframe.mWorldPose.position[0] + RADAR_MAX_RANGE_M_SQRT2,
                  aKeyframe.mWorldPose.position[1] + RADAR_MAX_RANGE_M_SQRT2) {
    // Copy over feature points
    mORSPFeaturePoints = aKeyframe.mORSPFeaturePoints;
}

/**
 * @brief Destructor for Keyframe::Keyframe
 * Currently empty.
 */
Keyframe::~Keyframe() {}

/**
 * @brief Get world pose of keyframe
 * @note Alias to get world pose
 *
 * @return const Pose2D& World pose
 */
const Pose2D &Keyframe::getPose() const {
    return getWorldPose();
}

/**
 * @brief Get world pose of keyframe
 *
 * @return const Pose2D& World pose
 */
const Pose2D &Keyframe::getWorldPose() const {
    return mWorldPose;
}

/**
 * @brief Get vector of ORSP feature points
 * @return Vector of ORSP feature points, in LOCAL coordinates
 */
const ORSPVec &Keyframe::getORSPFeaturePoints() const {
    return mORSPFeaturePoints;
}

/**
 * @brief Get local to world transform
 * @return Transform class of local to world coordinate transform
 */
const PoseTransform2D &Keyframe::getLocalToWorldTransform() const {
    return mLocalToWorldTransform;
}

/**
 * @brief Get world to local transform
 * @return Transform class of world to local coordinate transform
 */
const PoseTransform2D &Keyframe::getWorldToLocalTransform() const {
    return mWorldToLocalTransform;
}

/**
 * @brief Convert local ORSP point to world coordinate
 *
 * @param[in] aLocalORSPPoint Local ORSP point to be converted
 * @param[out] aWorldORSPPoint Output world ORSP point
 */
void Keyframe::localToWorldORSP(const ORSP &aLocalORSPPoint,
                                ORSP &aWorldORSPPoint) const {
    // Use pose transform handler library and internal world pose to convert
    // to world coordinate
    convertORSPCoordinates(aLocalORSPPoint, aWorldORSPPoint,
                           mLocalToWorldTransform);
}

/**
 * @brief Convert world ORSP point to local keyframe coordinate
 *
 * @param[in] aWorldORSPPoint World ORSP point to be converted
 * @param[out] aLocalORSPPoint Output local ORSP point
 */
void Keyframe::worldToLocalORSP(const ORSP &aWorldORSPPoint,
                                ORSP &aLocalORSPPoint) const {
    // Use pose transform handler library and internal world pose to convert
    // to world coordinate NOTE: Same function, but different pose transform
    // and parameter order
    convertORSPCoordinates(aWorldORSPPoint, aLocalORSPPoint,
                           mWorldToLocalTransform);
}

/**
 * @brief Find the closest feature point to a given point in world
 * coordinates
 * @note User needs to check if a feature point was found; otherwise, the
 * aClosestORSPPoint can give garbage values.
 * @return Whether a closest feature point was found
 */
const bool Keyframe::findClosestORSP(const ORSP &aORSPPoint,
                                     ORSP &aClosestORSPPoint) const {
    // Init stuff
    bool found = false;
    double closestDistance = ORSP_RADIUS;

    // Convert point to appropriate grid coordinate
    const PointCart2D centerPoint(aORSPPoint.center);

    PointCart2D gridCoord;

    pointToGridCoordinate(centerPoint, gridCoord, mGridCenter);

    // Check around the grid square but only up to sampling factor
    const ssize_t gridX = static_cast<ssize_t>(gridCoord.x);
    const ssize_t gridY = static_cast<ssize_t>(gridCoord.y);
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
                const ORSP &potentialClosestPoint =
                    mORSPFeaturePoints[potentialClosestPointIndices[i]];

                const PointCart2D potentialClosestPointCart(
                    potentialClosestPoint.center);

                // Check angle tolerance
                const double angle = angleBetweenVectors(
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