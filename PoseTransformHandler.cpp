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
 * @param[in] aRotMat Rotation matrix (N x N)
 * @param[in] aTrans Translation vector (N x 1)
 * @return Homogeneous transformation matrix (N+1 x N+1)
 */
const PoseTransform2D rotTransToTransform(const Eigen::Rotation2Dd &aRotMat,
                                          const Eigen::Vector2d &aTrans) {
    const int dimension = 2;

    PoseTransform2D transform = PoseTransform2D::Identity();

    // TODO: Check if its supposed to actually be rot, trans
    transform.translate(aTrans);
    transform.rotate(aRotMat);

    return transform;
}

const PoseTransform2D rotTransToTransform(const double aAngleRad,
                                          const Eigen::Vector2d &aTrans) {
    const Eigen::Rotation2Dd rotMat(aAngleRad);
    return rotTransToTransform(rotMat, aTrans);
}

/**
 * @brief Convert one coordinate to another coordinate frame using homogeneous
 * transforms. 2D interface.
 *
 * @param[in] aCoordinate Coordinate (N x 1)
 * @param[in] aConversionTransform Homogeneous pose transform matrix from local
 * to world coordinates (N+1 x N+1)
 * @param[in] isVector Whether or not coordinate is a vector (affects
 * homogeneous value)
 *
 * @return World coordinate (N x 1)
 */
const Eigen::Vector2d
convertCoordinate(const Eigen::Vector2d &aCoordinate,
                  const PoseTransform2D &aConversionTransform, bool isVector) {
    if (isVector) {
        return aConversionTransform.rotation() * aCoordinate.homogeneous();
    }

    return aConversionTransform * aCoordinate.homogeneous();
}

/**
 * @brief Convert a local to a world coordinate, using locally stored world pose
 * @note (0,0) in local coordinates is center of keyframe
 *
 * @param[in] aInputORSPPoint ORSP point to be converted to different coordinate
 * @param[out] aOutputORSPPoint ORSP point in new coordinate
 * @param[in] aConversionTransform Transformation matrix used in coordinate
 * conversion
 */
void convertORSPCoordinates(const ORSP &aInputORSPPoint, ORSP &aOutputORSPPoint,
                            const PoseTransform2D &aConversionTransform) {
    aOutputORSPPoint.center =
        convertCoordinate(aInputORSPPoint.center, aConversionTransform, false);
    aOutputORSPPoint.normal =
        convertCoordinate(aInputORSPPoint.normal, aConversionTransform, true);
}
