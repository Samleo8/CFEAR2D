/**
 * @file OrientedSurfacePointsHandler.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for oriented surface points (ORSP) representation in @see RadarImage class
 * @version 0.1
 * @date 2022-06-20
 * 
 * @copyright Copyright (c) 2022
 */

#include "OrientedSurfacePointsHandler.hpp"
#include "RadarImage.hpp"

/**
 * @brief Get Centroid from list of 2D points
 * 
 * @param[in] aPoints List of points
 * @return const Point2D 
 */
const PointCart2D getCentroid(const Point2DList &aPoints) {
    PointCart2D centroid(0, 0);
    for (const PointCart2D &point : aPoints) {
        centroid += point;
    }

    size_t sz = aPoints.size();
    centroid /= sz;

    return centroid;
}


/**
 * @brief Get Centroid from list of 3D points
 * 
 * @param[in] aPoints List of points
 * @return const Point3D 
 */
// const PointCart3D getCentroid(const Point3DList &aPoints) {
//     Point3D centroid(0,0);
//     for (const PointCart3D &point : aPoints) {
//         centroid += point;
//     }
//     centroid /= aPoints.size();
//     return centroid;
// }

/**
 * @brief Associate filtered point coordinate to downsampled grid coordinate
 * @param[in] aPoint Filtered point to associate
 * @param[out] aGridCoordinate Output grid coordinate
 */
void pointToGridCoordinate(const PointCart2D &aPoint, PointCart2D &aGridCoordinate) {
    // TODO: Check correctness
    // NOTE: Need to add max range to convert to correct coordinate system (where (0,0) is at top left)
    aGridCoordinate.x = floor((aPoint.x + RADAR_MAX_RANGE_M) / ORSP_GRID_SQUARE_WIDTH);
    aGridCoordinate.y = floor((aPoint.y + RADAR_MAX_RANGE_M) / ORSP_GRID_SQUARE_WIDTH);

    // TODO: Bound aGridCoordinate to be within grid
}

/**
 * @brief Downsample point cloud in radar image
 * 
 * 1) Associate the points in the 2D point cloud to their respective grids @see associateWithDownsampledGrid
 * 2) Find the centroids of the grids @see findCentroids
 */
void RadarImage::downsamplePointCloud(){
    // Place filtered points in grid
    FilteredPointsVec filteredPoints;

    for (size_t i = 0, sz = filteredPoints.size(); i < sz; i++) {
        PointCart2D gridCoordinate;
        pointToGridCoordinate(filteredPoints[i], gridCoordinate);

        mORSPGrid[gridCoordinate.x][gridCoordinate.y].push_back(filteredPoints[i]);
    }

    // Now find the centroids
    for (size_t i = 0; i < ORSP_GRID_N; i++) {
        for (size_t j = 0; j < ORSP_GRID_N; j++) {
            mORSPCentroidGrid[i][j] = getCentroid(mORSPGrid[i][j]);
        }
    }
}