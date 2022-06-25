#ifndef __POSE_TRANSFORM_H__
#define __POSE_TRANSFORM_H__

#include "Eigen/Core"
#include "OrientedSurfacePointsHandler.hpp"

typedef Eigen::MatrixXd PoseTransformXD;
typedef Eigen::Matrix3d PoseTransform2D;
typedef Eigen::Matrix4d PoseTransform3D;

const Eigen::MatrixXd poseToTransform(const Eigen::MatrixXd &aRotMat,
                                      const Eigen::VectorXd &aTrans);

const Eigen::MatrixXd poseToTransformInverted(const Eigen::MatrixXd &aRotMat,
                                              const Eigen::VectorXd &aTrans);

const Eigen::VectorXd localToWorldCoordinate(
    const Eigen::VectorXd &aLocalCoordinate,
    const PoseTransformXD &aWorldPoseTransform, bool isVector = false);

void localToWorldORSP(const ORSP &aLocalORSPPoint, ORSP &aWorldORSPPoint,
                      const PoseTransform2D &aWorldPoseTransform);

#endif // __POSE_TRANSFORM_H__