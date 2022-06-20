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

// Typedefs for points
typedef Eigen::Vector2d Point2D;
typedef Eigen::Vector3d Point3D;

typedef std::vector<Point2D> Point2DList;
typedef std::vector<Point3D> Point3DList;

/**
 * @brief Final Oriented Surface Point representation, holds mean and normal vector obtained from covariance
 */
struct OrientedSurfacePoint {
    // Mean / center of point
    Point2D center;

    // Normal vector
    Point2D normal;
};

/** @brief Typedef for OrientedSurfacePoint struct */
typedef struct OrientedSurfacePoints ORSP;

const Point2D getCentroid(const Point2DList &aPoints);
const Point3D getCentroid(const Point3DList &aPoints);

/// NOTE: All class related functions are declared in @see RadarImage.hpp

#endif