/**
 * @file OrientedSurfacePointsHandler.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for the oriented surface points associated with the @see
 * RadarImage class:
 *
 * Key sections:
 * 1) Downsampling and grid process
 *
 * Grid is represented as a square, with grid square width = r/f where r is the
 * radius of the grid and f is the downsampled factor and maximum grid length of
 * maximum radar range. Filtered points are associated to the grid (2D array of
 * vector of filtered points)
 *
 * 2) Formation of downsampled point cloud
 * 3) Formation of oriented surface points via distribution forming and
 * association
 *
 * @version 0.1
 * @date 2022-06-20
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __CFEAR_ORSP_HANDLER_HPP__
#define __CFEAR_ORSP_HANDLER_HPP__

#include "RadarImageHandler.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <math.h>
#include <vector>

/** @brief Oriented surface point radius, used for both point cloud downsampling
 * and finding point distribution */
const double ORSP_RADIUS = 3.5; // in meters

/** @brief Oriented surface point downsampling factor, used for point cloud
 * downsampling */
const unsigned int ORSP_RESAMPLE_FACTOR = 1;

/** @brief Width of the grid square used in the ORSP grid formation */
const double ORSP_GRID_SQUARE_WIDTH = ORSP_RADIUS / ORSP_RESAMPLE_FACTOR;

/** @brief Maximum number of grid squares, used in ORSP grid formation. Needs to
 * double because max range is radius of image. */
const size_t ORSP_GRID_N =
    static_cast<size_t>(floor(2 * RADAR_MAX_RANGE_M / ORSP_GRID_SQUARE_WIDTH));

/**
 * @brief Minimum number of valid neighbours of a filtered centroid point
 * required for the point distribution to be valid
 */
const size_t ORSP_VALID_NEIGHBOUR_MIN = 6;

/** @brief Eigenvalue ratio threshold above which distribution is considered ill
 * defined */
const double ORSP_EIGENVAL_THRESHOLD = 10e5;

/// NOTE: All class related functions are declared in @see RadarImage.hpp.
/// Helper functions are here.
template <typename T>
const T getDistance(const VectorXT<T> &aVec1, const VectorXT<T> &aVec2);

template <size_t Dimension>
const VectorDimd<Dimension>
getCentroid(const VectorDimdList<Dimension> &aPoints);

template <size_t Dimension>
void getMeanCovariance(const VectorDimdList<Dimension> &aPoints,
                       VectorDimd<Dimension> &aMean,
                       MatrixDimd<Dimension> &aCovMatrix);

void pointToGridCoordinate(const Eigen::Vector2d &aPoint,
                           Eigen::Vector2d &aGridCoordinate,
                           const Eigen::Vector2d &aGridCenter = Eigen::Vector2d(
                               RADAR_MAX_RANGE_M, RADAR_MAX_RANGE_M));

#include "OrientedSurfacePointsHandler.tpp"

#endif // __CFEAR_ORSP_HANDLER_HPP__