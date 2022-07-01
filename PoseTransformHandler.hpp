#ifndef __POSE_TRANSFORM_H__
#define __POSE_TRANSFORM_H__

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "OrientedSurfacePointsHandler.hpp"

/** @brief Typedef for 3D pose transform as Eigen isometric transform. Slightly
 * more memory but more natural. */
typedef Eigen::Isometry3d PoseTransform3D;

/** @brief Typedef for 2D pose transform as Eigen isometric transform. Slightly
 * more memory but more natural. */
typedef Eigen::Isometry2d PoseTransform2D;

/**
 * @brief Struct for storing 2D pose information as position(x,y),
 * orientation(theta)
 * @note 3D pose will be similar but orientation will probably be a quaternion
 * or set of euler angles
 */
struct Pose2D {
    Eigen::Vector2d position = Eigen::Vector2d::Zero(); ///< position as x, y
    double orientation = 0; ///< orientation as theta

    /**
     * @brief Constructor for Pose2D using position and orientation
     * vectors/scalars
     * @param[in] aPosition Position (x, y)
     * @param[in] aOrientation Orientation (theta)
     */
    Pose2D(const Eigen::Vector2d &aPosition, const double aOrientation)
        : position(aPosition), orientation(aOrientation) {}

    /**
     * @brief Constructor for Pose2D using (x,y,theta)
     * @param[in] aPosition Position (x, y)
     * @param[in] aOrientation Orientation (theta)
     */
    Pose2D(const double ax, const double ay, const double aOrientation)
        : orientation(aOrientation) {
        position << ax, ay;
    }
};

typedef struct Pose2D Pose2D; ///< typedef for struct Pose2D

const PoseTransform2D rotTransToTransform(const Eigen::Rotation2Dd &aRotMat,
                                          const Eigen::Vector2d &aTrans);

const PoseTransform2D rotTransToTransform(const double aAngleRad,
                                          const Eigen::Vector2d &aTrans);

const PoseTransform2D poseToTransform(const Pose2D &aPose);

const Eigen::Vector2d
convertCoordinate(const Eigen::Vector2d &aLocalCoordinate,
                  const PoseTransform2D &aWorldPoseTransform,
                  bool isVector = false);

void convertORSPCoordinates(const ORSP &aLocalORSPPoint, ORSP &aWorldORSPPoint,
                            const PoseTransform2D &aWorldPoseTransform);

#endif // __POSE_TRANSFORM_H__