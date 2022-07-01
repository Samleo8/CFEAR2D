#ifndef __POSE_TRANSFORM_H__
#define __POSE_TRANSFORM_H__

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "OrientedSurfacePointsHandler.hpp"

/** @brief Typedef for 3D pose transform as Eigen isometric transform. Slightly more memory but more natural. */
typedef Eigen::Isometry3d PoseTransform3D;

/** @brief Typedef for 2D pose transform as Eigen isometric transform. Slightly more memory but more natural. */
typedef Eigen::Isometry2d PoseTransform2D;

const PoseTransform2D rotTransToTransform(const Eigen::Rotation2Dd &aRotMat,
                                          const Eigen::Vector2d &aTrans);

const PoseTransform2D rotTransToTransform(const double aAngleRad,
                                          const Eigen::Vector2d &aTrans);

const Eigen::Vector2d
convertCoordinate(const Eigen::Vector2d &aLocalCoordinate,
                  const PoseTransform2D &aWorldPoseTransform,
                  bool isVector = false);

void convertORSPCoordinates(const ORSP &aLocalORSPPoint, ORSP &aWorldORSPPoint,
                            const PoseTransform2D &aWorldPoseTransform);

#endif // __POSE_TRANSFORM_H__