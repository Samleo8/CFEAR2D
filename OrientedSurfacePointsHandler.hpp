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

#ifndef __ORSP_H__
#define __ORSP_H__

#include "PointCart2D.hpp"
#include "RadarImageHandler.hpp"

#include <Eigen/Core>
#include <Eigen/Eigenvalues>
#include <math.h>

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

/**
 * @brief Oriented Surface Point representation, holds mean and normal
 * vector obtained from covariance
 */
struct ORSP {
    /// @brief Mean / center of point
    Eigen::Vector2d center;

    /// @brief Normal vector
    Eigen::Vector2d normal;

    /**
     * @brief Empty constructor for OrientedSurfacePoint
     */
    ORSP() : center(Eigen::Vector2d::Zero()), normal(Eigen::Vector2d::Zero()) {}

    /**
     * @brief Constructor for OrientedSurfacePoint
     * @param[in] aCenter Center of point
     * @param[in] aNormal Normal vector
     */
    ORSP(const Eigen::Vector2d &aCenter, const Eigen::Vector2d &aNormal)
        : center(aCenter), normal(aNormal) {}

    /**
     * @brief Copy Constructor for OrientedSurfacePoint
     * @param[in] aORSP ORSP to copy
     */
    ORSP(const ORSP &aORSP) : center(aORSP.center), normal(aORSP.normal) {}
};

/** @brief Oriented surface point struct typedef */
typedef struct ORSP ORSP;

/** @brief Typedef for OrientedSurfacePoint struct */
typedef std::vector<ORSP> ORSPVec;

/// NOTE: All class related functions are declared in @see RadarImage.hpp.
/// Helper functions are here.
const PointCart2D getCentroid(const Point2DList &aPoints);
const PointCart3D getCentroid(const Point3DList &aPoints);

void getMeanCovariance(const Point2DList &aPoints, Eigen::Vector2d &aMean,
                       Eigen::Matrix2d &aCovMatrix);
void pointToGridCoordinate(const PointCart2D &aPoint,
                           PointCart2D &aGridCoordinate,
                           const PointCart2D &aGridCenter = PointCart2D(
                               RADAR_MAX_RANGE_M, RADAR_MAX_RANGE_M));

#endif