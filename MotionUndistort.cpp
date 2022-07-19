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

template <typename T> extern T constrainAngle(const T &aAngleRad);

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

/**
 * @brief Convert azimuth value to index
 * @note Assumes the azimuth is from [-fovAngle/2, fovAngle/2]
 *
 * @pre Angle is from [-fovAngle/2, fovAngle/2]
 * @param[in] aAzimuthRad Azimuth angle in radians
 * @param[in] aNAzimuths Number of azimuths in a radar scan
 * @param[in] aFovAngleRad FOV angle in radians. Assumes azimuth is from
 * [-fovAngle/2, fovAngle/2]
 *
 * @return const size_t
 */
const size_t azimuthToIndex(const double aAzimuthRad, const double aNAzimuths,
                            const double fovAngleRad) {
    const double azimRadConstrained = constrainAngle<double>(aAzimuthRad);
    const double halfFov = fovAngleRad / 2;

    return static_cast<size_t>((azimRadConstrained + halfFov) / fovAngleRad *
                               aNAzimuths);
}

/**
 * @brief Undistorts feature points given a velocity and time vector
 *
 * @param[in] aORSPLocal Local ORSP feature point to undistort
 * @param[in] aVelocity Velocity of the vehicle
 * @param[in] aTimeVector Time vector
 * @param[in] aUndistortedORSPLocal Undistorted ORSP feature point, in local
 * coordinates
 */
void undistortORSP(const ORSP<double> &aORSPLocal,
                   const Pose2D<double> aVelocity,
                   const VectorXT<double> &aTimeVector,
                   ORSP<double> &aUndistortedORSPLocal) {
    // First get the associated azimuth of the ORSP point
    const Vector2T<double> &center = aORSPLocal.center;
    const double azimuth = std::atan2(center[1], center[0]);

    // Get the associated time and thus vel transformation matrix
    const size_t velInd = azimuthToIndex(azimuth, aTimeVector.size());
    const double time = aTimeVector[velInd];

    const PoseTransform2D<double> velTrans =
        getVelocityTransform(aVelocity, time);

    convertORSPCoordinates(aORSPLocal, aUndistortedORSPLocal, velTrans);
}