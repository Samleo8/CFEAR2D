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
 *
 * @param[in] aRadarImage Reference to radar image to be used to construct
 * keyframe
 * @param[in] aWorldPose
 */
Keyframe::Keyframe(const RadarImage &aRadarImage, const PoseTransform2D &aWorldPose)
    : mLocalToWorldTransform(aWorldPose) {
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

/**
 * @brief Convert local ORSP point to world coordinate
 * 
 * @param[in] aLocalORSPPoint Local ORSP point to be converted 
 * @param[out] aWorldORSPPoint Output world ORSP point
 */
void Keyframe::localToWorldORSP(const ORSP &aLocalORSPPoint,
                                      ORSP &aWorldORSPPoint) const {
    // Use pose transform handler library and internal world pose to convert to world coordinate
    convertORSPCoordinates(aLocalORSPPoint, aWorldORSPPoint, mLocalToWorldTransform);
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
    convertORSPCoordinates(aWorldORSPPoint, aLocalORSPPoint, mWorldToLocalTransform);
}