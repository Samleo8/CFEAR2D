/**
 * @file MotionUndistort.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Motion undistortion for feature points.
 * @version 0.1
 * @date 2022-07-18
 *
 * @copyright Copyright (c) 2022
 */

#include "ORSP.hpp"
#include "Pose2D.hpp"
#include "PoseTransformHandler.hpp"
#include "RadarImage.hpp"
#include "TransformDefines.hpp"

PoseTransform2D<double> getVelocityTransform(const Pose2D<double> &aVelocity,
                                             const double aTime);

void undistortPoint(const ORSP<double> &aORSPLocal,
                   const Pose2D<double> &aVelocity,
                   const VectorXT<double> &aTimeVector,
                   ORSP<double> &aUndistortedORSPLocal);

const size_t azimuthToIndex(const double aAzimuth, const size_t aNAzimuths,
                            const double fovAngleRad = RADAR_ANGLE_FOV_RAD);

/**
 * @brief Generates a time vector from [-T/2, T/2) with number of azimuths
 *
 * @tparam NAzimuths Number of azimuths in a radar scan
 * @param[in] aScanPeriod Period of a radar scan in seconds
 *
 * @return Time vector of size number of azimuths from [-T/2, T/2)
 */
template <int NAzimuths>
const VectorDimd<NAzimuths> generateTimeVector(const double aScanPeriod) {
    const double interval = aScanPeriod / NAzimuths;
    const double halfT = aScanPeriod / 2;

    return VectorDimd<NAzimuths>::LinSpaced(-halfT, halfT - interval);
}