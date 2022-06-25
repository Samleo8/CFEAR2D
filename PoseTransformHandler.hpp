#ifndef __POSE_TRANSFORM_H__
#define __POSE_TRANSFORM_H__

#include "Eigen/Core"
#include "OrientedSurfacePointsHandler.hpp"

typedef Eigen::Matrix3d PoseTransform2D;

const Eigen::Matrix3d poseToTransform(const Eigen::Matrix2d &aRotMat,
                                      const Eigen::Vector2d &aTrans);

void localToWorldCoordinate(const Eigen::Vector2d &aLocalCoordinate,
                            Eigen::Vector2d &aWorldCoordinate,
                            const PoseTransform2D &aWorldPoseTransform,
                            bool isVector = false);

void localToWorldORSP(const ORSP &aLocalORSPPoint, ORSP &aWorlORSPPoint,
                      const PoseTransform2D &aWorldPoseTransform);

#endif // __POSE_TRANSFORM_H__