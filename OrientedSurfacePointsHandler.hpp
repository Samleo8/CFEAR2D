/**
 * @file OrientedSurfacePointsHandler.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for the oriented surface points associated with the @see RadarImage class:
 * 
 * Key sections:
 * 1) Downsampling and grid process
 * 
 * Grid is represented as a square, with grid square width = r/f where r is the radius of the grid and f is the downsampled factor
 * and maximum grid length of maximum radar range. 
 * Filtered points are associated to the grid (2D array of vector of filtered points)
 * 
 * 2) Formation of downsampled point cloud
 * 3) Formation of oriented surface points via distribution forming and association
 * 
 * @version 0.1
 * @date 2022-06-20
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __ORSP_H__
#define __ORSP_H__

#include <math.h>
#include <Eigen/Core>
#include "PointCart2D.hpp"
#include "RadarImageHandler.hpp"

/** @brief Oriented surface point radius, used for both point cloud downsampling
 * and finding point distribution */
const double ORSP_RADIUS = 3.5; // in meters

/** @brief Oriented surface point downsampling factor, used for point cloud
 * downsampling */
const unsigned int ORSP_RESAMPLE_FACTOR = 1;

/** @brief Width of the grid square used in the ORSP grid formation */
const double ORSP_GRID_SQUARE_WIDTH = ORSP_RADIUS / ORSP_RESAMPLE_FACTOR;

/** @brief Maximum number of grid squares, used in ORSP grid formation. Needs to double because max range is radius of image. */
const size_t ORSP_GRID_N = static_cast<size_t>(floor(2 * RADAR_MAX_RANGE_M / ORSP_GRID_SQUARE_WIDTH));

/**
 * @brief Final Oriented Surface Point representation, holds mean and normal vector obtained from covariance
 */
typedef struct {
    // Mean / center of point
    Eigen::Vector2d center;

    // Normal vector
    Eigen::Vector2d normal;
} ORSP;

/** @brief Typedef for OrientedSurfacePoint struct */
typedef std::vector<ORSP> ORSPVec;

/// NOTE: All class related functions are declared in @see RadarImage.hpp. Helper functions are here.
const PointCart2D getCentroid(const Point2DList &aPoints);
const PointCart3D getCentroid(const Point3DList &aPoints);

void pointToGridCoordinate(const PointCart2D &aPoint, PointCart2D &aGridCoordinate);

#endif