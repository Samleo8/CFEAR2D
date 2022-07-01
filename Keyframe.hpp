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

#include "OrientedSurfacePointsHandler.hpp"
#include "PoseTransformHandler.hpp"
#include "RadarImage.hpp"

/**
 * @brief Maximum number of grid squares, used in ORSP grid formation.
 * Needs to double because max range is radius of image.
 * @note Needs to be larger than the original ORSP grid size because in global
 * coordinates, so frame is allowed to turn.
 */
const size_t ORSP_KF_GRID_N = static_cast<size_t>(
    floor(2 * RADAR_MAX_RANGE_M_SQRT2 / ORSP_GRID_SQUARE_WIDTH));

class Keyframe {
  private:
    /** @brief World pose, probably mainly for plotting */
    const Pose2D mWorldPose;

    /** @brief World pose as homogeneous transformation matrix that converts
     * from local to world coordinates */
    const PoseTransform2D mLocalToWorldTransform;

    /** @brief Homogeneous transform matrix that converts world to local
     * coordinates */
    PoseTransform2D mWorldToLocalTransform;

    /** @brief Grid center used for getting new grid coordinate in WORLD
     * coordinates
     * @see pointToGridCoordinate()
     */
    const PointCart2D mGridCenter;

    /** @brief ORSP feature points in WORLD coordinates */
    ORSPVec mORSPFeaturePoints;

    /**
     * @brief Caching of grid representation of ORSP feature points, with r/f as
     * the length of the grid. Allows very quick checking of surrounding ORSP
     * points. Value of the grid contains the indices to the exact ORSP point,
     * indexed to mORSPFeaturePoints.
     * @note Empty vector implies no coordinate there.
     * @note Because of the grid conversion that could potentially be rotated,
     * we might not be able to assume (TODO: check math?) that a single ORSP
     * point will be alone inside a grid square. Thus a vector of indices is
     * required.
     * @todo Make this a sparse representation instead?
     * @see pointToGridCoordinate()
     */
    std::vector<ssize_t> mORSPIndexGrid[ORSP_KF_GRID_N][ORSP_KF_GRID_N];

  public:
    Keyframe(const RadarImage &aRadarImage, const Pose2D &aWorldPose);
    ~Keyframe();

    // Getters
    const Pose2D &getPose() const;
    const Pose2D &getWorldPose() const;
    const ORSPVec &getORSPFeaturePoints() const;
    const PoseTransform2D &getLocalToWorldTransform() const;
    const PoseTransform2D &getWorldToLocalTransform() const;

    // Conversion functions for ORSP
    void localToWorldORSP(const ORSP &aLocalORSPPoint,
                          ORSP &aWorldORSPPoint) const;
    void worldToLocalORSP(const ORSP &aWorldORSPPoint,
                          ORSP &aLocalORSPPoint) const;
};

#endif // __KEYFRAME_H__