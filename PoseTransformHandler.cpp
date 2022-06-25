/**
 * @file PoseTransformHandler.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handlers for poses and transformation matrices
 * @version 0.1
 * @date 2022-06-25
 *
 * @copyright Copyright (c) 2022
 */

#include "PoseTransformHandler.hpp"

/**
 * @brief Convert rotation and translation to transformation matrix
 *
 * @param[in] aRotMat Rotation matrix
 * @param[in] aTrans Translation vector
 * @return const Eigen::Matrix3d Homogeneous transformation matrix
 */
const Eigen::Matrix3d poseToTransform(const Eigen::Matrix2d &aRotMat,
                                      const Eigen::Vector2d &aTrans) {
    Eigen::Matrix3d transform;
    // transform << aRotMat(0, 0), aRotMat(0, 1), aTrans(0), aRotMat(1, 0),
    //     aRotMat(1, 1), aTrans(1), 0, 0, 1;
    transform << aRotMat, aTrans, 0, 0, 1;
    return transform;
}

void localToWorldCoordinate(const Eigen::Vector2d &aLocalCoordinate,
                            Eigen::Vector2d &aWorldCoordinate,
                            const PoseTransform2D &aWorldPoseTransform,
                            bool isVector) {
    Eigen::Vector3d coordHomo;
    double homoVal = (isVector) ? 0.0 : 1.0;

    coordHomo << aLocalCoordinate, homoVal;

    aWorldCoordinate = aWorldPoseTransform * coordHomo;
}

/**
 * @brief Convert a local to a world coordinate, using locally stored world pose
 * @note (0,0) in local coordinates is center of keyframe
 *
 * @param[in] aLocalPoint Local point to be converted to world coordinate
 * @param[out] aWorldPoint World coordinate of local point
 * @param[in] aWorldPoseTransform Transformation matrix of world pose
 */
void localToWorldORSP(const ORSP &aLocalORSPPoint, ORSP &aWorlORSPPoint,
                      const PoseTransform2D &aWorldPoseTransform) {
    // Eigen::Vector2d center(aLocalORSPPoint.center);
    // Eigen::Vector2d normal(aLocalORSPPoint.normal);

    Eigen::Vector3d centerHomo;
    centerHomo << aLocalORSPPoint.center, 1;

    Eigen::Vector3d normalHomo;
    normalHomo << aLocalORSPPoint.normal, 0; // note: 0 cos vector

    Eigen::Vector3d centerWorldHomo = aWorldPoseTransform * centerHomo;
    Eigen::Vector3d normalWorldHomo = aWorldPoseTransform * normalHomo;

    aWorlORSPPoint.center = centerWorldHomo.head(2);
    aWorlORSPPoint.normal = normalWorldHomo.head(2);
}
