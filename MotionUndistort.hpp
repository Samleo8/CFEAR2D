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

const VectorXT<double> generateTimeVector(const double aScanPeriod,
                                          const double aNAzimuths);

void undistortORSP(const ORSP<double> &aORSPLocal,
                   const Pose2D<double> &aVelocity,
                   const VectorXT<double> &aTimeVector,
                   ORSP<double> &aUndistortedORSPLocal);

const size_t azimuthToIndex(const double aAzimuth, const double aNAzimuths,
                            const double fovAngleRad = RADAR_ANGLE_FOV_RAD);