#ifndef __POSE_TRANSFORM_H__
#define __POSE_TRANSFORM_H__

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "ORSP.hpp"
#include "Pose2D.hpp"
#include "TransformDefines.hpp"

template <typename T>
const PoseTransform2D<T>
rotTransToTransform(const Eigen::Rotation2D<T> &aRotMat,
                    const Vector2T<T> &aTrans);

template <typename T>
const PoseTransform2D<T> rotTransToTransform(const T &aAngleRad,
                                             const Vector2T<T> &aTrans);

template <typename T>
const PoseTransform2D<T> poseToTransform(const Pose2D<T> &aPose);

template <typename T>
const Pose2D<T> transformToPose(const PoseTransform2D<T> &aPoseTransform);

template <typename T>
const PoseTransform2D<T> paramsToTransform(const T *const aPositionArray,
                  const T *const aOrientationArray);

template <typename T>
const Vector2T<T>
convertCoordinate(const Vector2T<T> &aCoordinate,
                  const PoseTransform2D<T> &aConversionTransform,
                  bool isVector = false);

template <typename T>
void convertORSPCoordinates(const ORSP<T> &aLocalORSPPoint,
                            ORSP<T> &aWorldORSPPoint,
                            const PoseTransform2D<T> &aWorldPoseTransform);

// Include implementation file
#include "PoseTransformHandler.tpp"

#endif // __POSE_TRANSFORM_H__