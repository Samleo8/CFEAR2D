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

class Keyframe {
  private:
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
    /** @brief Dimension of internal poses, transforms etc */
    static constexpr size_t DIMENSION = 2;

    /**
     * @brief How much movement (distance in m) from previous keyframe to count
     * as new keyframe
     * @todo Move to the radar feed file?
     */
    static constexpr double KF_DIST_THRESH = 1.5;

    /** @brief Square of @see Keyframe::KF_DIST_THRESH */
    static constexpr double KF_DIST_THRESH_SQ = KF_DIST_THRESH * KF_DIST_THRESH;

    /**
     * @brief How much movement (distance in m) from previous keyframe to count
     * as new keyframe
     * @todo Move to the radar feed file?
     */
    static constexpr double KF_ROT_THRESH = 5 * ANGLE_DEG_TO_RAD;

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
};

/** @brief Keyframe buffer typedef */
typedef boost::circular_buffer<Keyframe> KeyframeBuffer;

#endif // __KEYFRAME_HPP__