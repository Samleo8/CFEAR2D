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

#ifndef __KEYFRAME_HPP__
#define __KEYFRAME_HPP__

#include <Eigen/Geometry>
#include <vector>

#include <boost/circular_buffer.hpp>

#include "ORSP.hpp"
#include "OrientedSurfacePointsHandler.hpp"
#include "PoseTransformHandler.hpp"
#include "RadarImage.hpp"
#include "TransformDefines.hpp"

/**
 * @brief Maximum number of grid squares, used in ORSP grid formation.
 * Needs to double because max range is radius of image.
 * @note Needs to be larger than the original ORSP grid size because in global
 * coordinates, so frame is allowed to turn.
 */
const size_t ORSP_KF_GRID_N = static_cast<size_t>(
    floor(2 * RADAR_MAX_RANGE_M_SQRT2 / ORSP_GRID_SQUARE_WIDTH));

typedef std::vector<size_t> IndexList;

class Keyframe {
  private:
    static const size_t DIMENSION = 2;

    /** @brief World pose, probably mainly for plotting */
    Pose2D<double> mWorldPose;

    /**
     * @brief World pose as homogeneous transformation matrix that converts
     * from local to world coordinates. Effectively const: only initialized on
     constructor but no `const` because circular_buffer needs the copy
     assignment
     */
    PoseTransform2D<double> mLocalToWorldTransform;

    /**
     * @brief Homogeneous transform matrix that converts world to local
     * coordinates. Effectively const: only initialized on constructor but no
     * `const` because circular_buffer needs the copy assignment.
     */
    PoseTransform2D<double> mWorldToLocalTransform;

    /** @brief List/Vector of ORSP feature points in WORLD coordinates */
    ORSPVec<double> mORSPFeaturePoints;

  public:
    Keyframe(const RadarImage &aRadarImage, const Pose2D<double> &aWorldPose);
    ~Keyframe();

    // Getters
    const Pose2D<double> &getPose() const;
    const Pose2D<double> &getWorldPose() const;
    const ORSPVec<double> &getORSPFeaturePoints() const;

    const PoseTransform2D<double> &getLocalToWorldTransform() const;
    const PoseTransform2D<double> &getWorldToLocalTransform() const;

    // Conversion functions for ORSP
    void localToWorldORSP(const ORSP<double> &aLocalORSPPoint,
                          ORSP<double> &aWorldORSPPoint) const;
    void worldToLocalORSP(const ORSP<double> &aWorldORSPPoint,
                          ORSP<double> &aLocalORSPPoint) const;

    // Find the closest ORSP point to a given point
    template <typename CastType>
    [[nodiscard]] const bool
    findClosestORSP(const ORSP<CastType> &aORSPPoint,
                    ORSP<CastType> &aClosestORSPPoint) const;
};

/** @brief Keyframe buffer typedef */
typedef boost::circular_buffer<Keyframe> KeyframeBuffer;

#include "Keyframe.tpp"

#endif // __KEYFRAME_HPP__