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

#include <Eigen/Core>

#include "PoseTransformHandler.hpp"
#include "RadarImage.hpp"
#include "OrientedSurfacePointsHandler.hpp"

class Keyframe {
  private:
    /** @brief World pose as homogeneous transformation matrix that converts
     * from local to world coordinates */
    const PoseTransform2D mLocalToWorldTransform;

    /** @brief Homogeneous transform matrix that converts world to local
     * coordinates */
    const PoseTransform2D mWorldToLocalTransform;

    /** @brief ORSP feature points in LOCAL coordinates @todo maybe make this
     * global? */
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
    Keyframe(const RadarImage &aRadarImage, const PoseTransform2D &aWorldPose);
    ~Keyframe();

    void localToWorldORSP(const ORSP &aLocalORSPPoint,
                          ORSP &aWorldORSPPoint) const;
    void worldToLocalORSP(const ORSP &aWorldORSPPoint,
                          ORSP &aLocalORSPPoint) const;
};

#endif // __KEYFRAME_H__