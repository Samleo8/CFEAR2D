/**
 * @file MotionUndistort.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Motion undistortion for feature points.
 * @version 0.1
 * @date 2022-07-18
 *
 * @copyright Copyright (c) 2022
 */

#include "MotionUndistort.hpp"
#include "Pose2D.hpp"
#include "PoseTransformHandler.hpp"
#include "TransformDefines.hpp"

/**
 * @brief Get transform matrix for undistorting motion given time and velocity
 *
 * @param[in] aVelocity
 * @param[in] aTime
 *
 * @return PoseTransform2D<double>
 */
PoseTransform2D<double> getVelocityTransform(const Pose2D<double> &aVelocity,
                                             const double aTime) {
    // Basically get the translation and rotation by multiplying the velocity by
    // the time
    Pose2D<double> movement(aVelocity);
    movement *= aTime;

    // The transform is a conversion of this transl + rotation (represented as a
    // pose) to a transform
    // TODO: Check correctness of conversion of transform
    return poseToTransform(movement);
}

/**
 * @brief Generates a time vector from [-T/2, T/2) with number of azimuths
 *
 * @param[in] aScanPeriod Period of a radar scan in seconds
 * @param[in] aNAzimuths  Number of azimuths in a radar scan
 *
 * @return Time vector of size number of azimuths from [-T/2, T/2)
 */
const VectorXT<double> generateTimeVector(const double aScanPeriod,
                                          const double aNAzimuths) {
    const double interval = aScanPeriod / aNAzimuths;
    const double halfT = aScanPeriod / 2;

    return VectorXT<double>::LinSpaced(-halfT, halfT - interval, aNAzimuths);
}