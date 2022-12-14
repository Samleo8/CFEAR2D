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
#include "ORSP.hpp"
#include "Pose2D.hpp"

#include "OptimisationHandler.hpp"
#include "PoseTransformHandler.hpp"
#include "RadarImageHandler.hpp"

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
const size_t azimuthToIndex(const double aAzimuthRad, const size_t aNAzimuths,
                            const double fovAngleRad) {
    const double azimRadConstrained = constrainAngle<double>(aAzimuthRad);
    const double halfFov = fovAngleRad / 2;

    return static_cast<size_t>((azimRadConstrained + halfFov) / fovAngleRad *
                               aNAzimuths);
}

/**
 * @brief Undistorts feature points given a velocity and time vector
 *
 * @param[in] aFilteredPtLocal Local ORSP feature point to undistort
 * @param[in] aVelocity Velocity of the vehicle
 * @param[in] aTimeVector Time vector
 * @param[in] aUndistortedFilteredPtLocal Undistorted ORSP feature point, in
 * local coordinates
 */
void undistortPoint(const FilteredPoint &aFilteredPtLocal,
                    const Pose2D<double> &aVelocity,
                    const VectorXT<double> &aTimeVector,
                    FilteredPoint &aUndistortedFilteredPtLocal) {
    // First get the associated azimuth of the filtered point
    const double azimuth = std::atan2(aFilteredPtLocal[1], aFilteredPtLocal[0]);

    // Get the associated time and thus vel transformation matrix
    const size_t velInd = azimuthToIndex(azimuth, aTimeVector.size());
    const double time = aTimeVector[velInd];

    const PoseTransform2D<double> velTrans =
        getVelocityTransform(aVelocity, time).inverse();

    aUndistortedFilteredPtLocal =
        convertCoordinate<double>(aFilteredPtLocal, velTrans, false);
}

/**
 * @brief Perform motion undistortion on internal vector of filtered points in
 * local frame
 *
 * @param[in] aVelocity Velocity vector in the form of a 2D pose (vx, vy, omega)
 * @param[in] aTimeVector Cached vector of time values for computing
 * intermediate azimuths
 *
 */
void RadarImage::performMotionUndistortion(
    const Pose2D<double> &aVelocity, const VectorXT<double> &aTimeVector) {

    // Replace current ORSP feature points with undistorted ones
    for (FilteredPoint &filteredPt : mFilteredPoints) {
        undistortPoint(filteredPt, aVelocity, aTimeVector, filteredPt);
    }
}