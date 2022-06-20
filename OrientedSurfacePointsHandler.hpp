/**
 * @file OrientedSurfacePointsHandler.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief 
 * @version 0.1
 * @date 2022-06-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __ORSP_H__
#define __ORSP_H__

#include <Eigen/Geometry>

/** @brief Oriented surface point radius, used for both point cloud downsampling
 * and finding point distribution */
const double ORSP_RADIUS = 3.5; // in meters

/** @brief Oriented surface point downsampling factor, used for point cloud
 * downsampling */
const unsigned int ORSP_RESAMPLE_FACTOR = 1;

/**
 * @brief Final Oriented Surface Point representation, holds mean and normal vector obtained from covariance
 */
struct OrientedSurfacePoint {
    // Mean / center of point
    Eigen::Vector2d center;

    // Normal vector
    Eigen::Vector2d normal;
};

/** @brief Typedef for OrientedSurfacePoint struct */
typedef struct OrientedSurfacePoints ORSP;


#endif