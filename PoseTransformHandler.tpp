/**
 * @file PoseTransformHandler.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handlers for poses and transformation matrices
 * @version 0.1
 * @date 2022-06-25
 *
 * @copyright Copyright (c) 2022
 */

#ifndef __POSE_TRANSFORM_HANDLER_CPP__
#define __POSE_TRANSFORM_HANDLER_CPP__

/**
 * @brief Convert rotation and translation to transformation matrix
 *
 * @tparam[in] T Scalar type, used for Ceres
 * @param[in] aRotMat Rotation matrix (N x N)
 * @param[in] aTrans Translation vector (N x 1)
 * @return Homogeneous transformation matrix (N+1 x N+1)
 */
template <typename T>
const PoseTransform2D<T>
rotTransToTransform(const Eigen::Rotation2D<T> &aRotMat,
                    const Vector2T<T> &aTrans) {
    const int dimension = 2;

    PoseTransform2D<T> transf = PoseTransform2D<T>::Identity();

    // TODO: Check if its supposed to actually be rot, trans
    transf.translate(aTrans);
    transf.rotate(aRotMat);

    return transf;
}

/**
 * @brief Convert rotation and translation to transformation matrix
 *
 * @tparam T Scalar type, used for Ceres
 * @param[in] aAngleRad Angle of rotation in radians
 * @param[in] aTrans Translation vector (N x 1)
 * @return Homogeneous transformation matrix (N+1 x N+1)
 */
template <typename T>
const PoseTransform2D<T> rotTransToTransform(const T &aAngleRad,
                                             const Vector2T<T> &aTrans) {
    const Eigen::Rotation2D<T> rotMat(aAngleRad);
    return rotTransToTransform<T>(rotMat, aTrans);
}

template <typename T>
const PoseTransform2D<T> poseToTransform(const Pose2D &aPose) {
    return rotTransToTransform<T>(aPose.orientation, aPose.position);
}

/**
 * @brief Convert one coordinate to another coordinate frame using homogeneous
 * transforms. 2D interface.
 *
 * @tparam T Scalar type, used for Ceres
 * @param[in] aCoordinate Coordinate (N x 1)
 * @param[in] aConversionTransform Homogeneous pose transform matrix from local
 * to world coordinates (N+1 x N+1)
 * @param[in] isVector Whether or not coordinate is a vector (affects
 * homogeneous value)
 *
 * @return World coordinate (N x 1)
 */
template <typename T>
const Vector2T<T>
convertCoordinate(const Vector2T<T> &aCoordinate,
                  const PoseTransform2D<T> &aConversionTransform,
                  bool isVector) {
    if (isVector) {
        return aConversionTransform.rotation() * aCoordinate;
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
template <typename T>
void convertORSPCoordinates(const ORSP<T> &aInputORSPPoint, ORSP<T> &aOutputORSPPoint,
                            const PoseTransform2D<T> &aConversionTransform) {
    aOutputORSPPoint.center = convertCoordinate<T>(aInputORSPPoint.center,
                                                   aConversionTransform, false);
    aOutputORSPPoint.normal = convertCoordinate<T>(aInputORSPPoint.normal,
                                                   aConversionTransform, true);
}

#endif // __POSE_TRANSFORM_HANDLER_HPP__