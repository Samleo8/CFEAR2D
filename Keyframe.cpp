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

/**
 * @brief Constructor for Keyframe class. Handles transferring of relevant data
 * structures (specifically grid representations and points) from RadarImage
 * into the class
 * TODO: Use transformation matrix instead
 *
 * @param[in] aRadarImage Reference to radar image to be used to construct
 * keyframe
 * @param[in] aWorldPose
 */
Keyframe::Keyframe(RadarImage &aRadarImage, const Pose2D &aWorldPose)
    : mWorldPoseTransform(aWorldPose) {
    // Copy over ORSP points and populate grid
    const ORSPVec &ORSPFeaturePointsRef = aRadarImage.getORSPFeaturePoints();
    mORSPFeaturePoints.reserve(ORSPFeaturePointsRef.size());

    // Populate grid with -1 to indicate no ORSP point in this grid cell
    for (size_t i = 0; i < ORSP_GRID_N; i++) {
        for (size_t j = 0; j < ORSP_GRID_N; j++) {
            mORSPIndexGrid[i][j] = -1;
        }
    }

    // TODO: Need to ensure deep copy?
    for (const ORSP &ORSPFeaturePoint : ORSPFeaturePointsRef) {
        // Populate grid accordingly, note we are still in local coordinates
        PointCart2D centerPoint(ORSPFeaturePoint.center);
        PointCart2D gridCoord;   
        pointToGridCoordinate(centerPoint, gridCoord);

        // Copy over ORSP point
        mORSPFeaturePoints.push_back(ORSPFeaturePoint);
    }
}

const Eigen::Matrix3d Keyframe::poseToTransform(const ) {
    return mWorldPoseTransform;
}

/**
 * @brief Convert a local to a world coordinate, using locally stored world pose
 * @note (0,0) in local coordinates is center of keyframe
 * 
 * @param[in] aLocalPoint Local point to be converted to world coordinate
 * @param[out] aWorldPoint World coordinate of local point
 */
void Keyframe::localToWorldCoordinate(const ORSP &aLocalORSPPoint,
                                      ORSP &aWorlORSPPoint) {
    
}