/**
 * @file Keyframe.hpp
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

#ifndef __KEYFRAME_H__
#define __KEYFRAME_H__

#include "Eigen/Core"
#include "OrientedSurfacePointsHandler.hpp"
#include "RadarImage.hpp"

typedef Eigen::Matrix3d Pose2D;

class Keyframe {
  private:
    /** @brief World pose as transformation matrix */
    const Pose2D mWorldPoseTransform;

    /** @brief ORSP feature points in LOCAL coordinates @todo maybe make this global? */
    ORSPVec mORSPFeaturePoints; 

    /**
     * @brief Caching of grid representation of ORSP feature points, with r/f as
     * the length of the grid. Allows very quick checking of surrounding ORSP
     * points. Value of the grid contains the index to the exact ORSP point,
     * indexed to mORSPFeaturePoints. -1 implies no coordinate there.
     * @todo Make this a sparse representation instead
     * @todo Make it store index+1, so 0s are default as no coordinate
     * @see pointToGridCoordinate()
     */
    ssize_t mORSPIndexGrid[ORSP_GRID_N][ORSP_GRID_N];

  public:
    Keyframe(RadarImage &aRadarImage, const Pose2D &aWorldPose);
    ~Keyframe();

    void localToWorldCoordinate(const ORSP &aLocalORSPPoint,
                                     ORSP &aWorlORSPPoint);
};

const Eigen::Matrix3d poseToTransform(const Eigen::Matrix2d &aRotMat,
                                      const Eigen::Vector2d &aTrans);

#endif // __KEYFRAME_H__