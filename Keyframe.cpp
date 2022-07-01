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
#include "PoseTransformHandler.hpp"

/**
 * @brief Constructor for Keyframe class. Handles transferring of relevant data
 * structures (specifically grid representations and points) from RadarImage
 * into the class
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

        size_t gridX = static_cast<size_t>(gridCoord.x);
        size_t gridY = static_cast<size_t>(gridCoord.y);
        mORSPIndexGrid[gridX][gridY].push_back(i);

        // Copy over ORSP point
        mORSPFeaturePoints.push_back(worldORSPPoint);
    }

    // TODO: Use Eigen::Map if necessary to convert a set of coordinates from
    // world to local or vice versa
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
    // Use pose transform handler library and internal world pose to convert to
    // world coordinate
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
    // Use pose transform handler library and internal world pose to convert to
    // world coordinate
    // NOTE: Same function, but different pose transform and parameter order
    convertORSPCoordinates(aWorldORSPPoint, aLocalORSPPoint,
                           mWorldToLocalTransform);
}