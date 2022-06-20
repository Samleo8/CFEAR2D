/**
 * @file OrientedSurfacePointsHandler.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for oriented surface points (ORSF) representation in @see RadarImage class
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
const Point2D getCentroid(const Point2DList &aPoints) {
    Point2D centroid(0,0);
    for (const Point2D &point : aPoints) {
        centroid += point;
    }
    centroid /= aPoints.size();
    return centroid;
}


/**
 * @brief Get Centroid from list of 3D points
 * 
 * @param[in] aPoints List of points
 * @return const Point3D 
 */
const Point3D getCentroid(const Point3DList &aPoints) {
    Point3D centroid(0,0);
    for (const Point3D &point : aPoints) {
        centroid += point;
    }
    centroid /= aPoints.size();
    return centroid;
}

void associateWithDownsampledGrid(){

}

/**
 * @brief Downsample point cloud in radar image
 * 
 * 1) Associate the points in the 2D point cloud to their respective grids @see associateWithDownsampledGrid
 * 
 */
void RadarImage::downsamplePointCloud(){

}