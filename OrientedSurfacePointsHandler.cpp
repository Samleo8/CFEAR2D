/**
 * @file OrientedSurfacePointsHandler.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for oriented surface points (ORSP) representation in @see
 * RadarImage class
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

    // Empty list: Return (0,0) for centroid
    if (aPoints.size() == 0) return centroid;

    // Otherwise, calculate centroid by summing up all coordinates
    for (const PointCart2D &point : aPoints) {
        centroid += point;
    }

    // And dividing by size
    size_t sz = aPoints.size();
    centroid /= sz;

    return centroid;
}

/**
 * @brief Get mean and covariance matrix from list of 2D points
 *
 * @param[in] aPoints
 * @param[in] aMean
 * @param[in] aCovMatrix
 */
void getMeanCovariance(const Point2DList &aPoints, Eigen::Vector2d &aMean,
                       Eigen::Matrix2d &aCovMatrix) {
    // Convert list of points into Eigen matrix, then use vectorization
    const size_t sz = aPoints.size();
    Eigen::Matrix2d pointsListMat(
        sz, 2); // TODO: Eigen is col-major, so colwise is faster?

    for (size_t i = 0; i < sz; i++) {
        pointsListMat(i, 0) = aPoints[i].x;
        pointsListMat(i, 1) = aPoints[i].y;
    }

    // Use vectorisation to get mean and covariance
    aMean = pointsListMat.colwise().mean();

    // Xvec - E[Xvec]
    Eigen::Matrix2d deltaExpected = pointsListMat.colwise() - aMean;

    aCovMatrix = deltaExpected.transpose() * deltaExpected / sz;
}

/**
 * @brief Associate filtered point coordinate to downsampled grid coordinate
 * @param[in] aPoint Filtered point to associate
 * @param[out] aGridCoordinate Output grid coordinate
 */
void pointToGridCoordinate(const PointCart2D &aPoint,
                           PointCart2D &aGridCoordinate) {
    // TODO: Check correctness
    // NOTE: Need to add max range to convert to correct coordinate system
    // (where (0,0) is at top left)
    aGridCoordinate.x =
        floor((aPoint.x + RADAR_MAX_RANGE_M) / ORSP_GRID_SQUARE_WIDTH);
    aGridCoordinate.y =
        floor((aPoint.y + RADAR_MAX_RANGE_M) / ORSP_GRID_SQUARE_WIDTH);

    // TODO: Bound aGridCoordinate to be within grid
}

/**
 * @brief Downsample point cloud in radar image
 *
 * 1) Associate the points in the 2D point cloud to their respective grids @see
 * associateWithDownsampledGrid 2) Find the centroids of the grids @see
 * findCentroids
 *
 * @note Upgrades the member variables ORSPGrid and ORSPCentroid grid
 */
void RadarImage::downsamplePointCloud() {
    // Place filtered points in grid
    const FilteredPointsVec &filteredPoints = getFilteredPoints();

    const size_t sz = filteredPoints.size();
    std::cout << "Downsampling point cloud of size " << sz << " into " << ORSP_GRID_N << " x " << ORSP_GRID_N << " grid..." << std::endl;

    for (size_t i = 0; i < sz; i++) {
        const FilteredPoint &filtPt = filteredPoints[i];
        PointCart2D gridCoordinate;
        pointToGridCoordinate(filtPt, gridCoordinate);

        size_t gridX = static_cast<size_t>(gridCoordinate.x);
        size_t gridY = static_cast<size_t>(gridCoordinate.y);

        // std::cout << "Raw XY: (" << filtPt.x << ", " << filtPt.y << ") | ";
        // std::cout << "Grid coordinate: (" << gridX << ", " << gridY << ")" << std::endl;

        mORSPGrid[gridX][gridY].push_back(filteredPoints[i]);
    }

    // Now find the centroids
    for (size_t i = 0; i < ORSP_GRID_N; i++) {
        for (size_t j = 0; j < ORSP_GRID_N; j++) {
            mORSPCentroidGrid[i][j] = getCentroid(mORSPGrid[i][j]);
        }
    }
}

/**
 * @brief Find valid neighbours of a grid square
 * @note Because the search radius is r, and grid square size is r/f, we only
 *       need to search around the `f` grids around the center
 *
 * @param[out] aValidNeighbours List of valid neighbours to output into
 * @param[in] aGridX Grid x coordinate
 * @param[in] aGridY Grid y coordinate
 */
void RadarImage::findValidNeighbours(Point2DList aValidNeighbours,
                                     const size_t aGridX, const size_t aGridY) {
    // Prep valid neighbours
    aValidNeighbours.clear();

    const PointCart2D centroid = mORSPCentroidGrid[aGridX][aGridY];

    // Check around the grid square
    for (ssize_t dx = -ORSP_RESAMPLE_FACTOR; dx <= ORSP_RESAMPLE_FACTOR; dx++) {
        // Bounds check: X
        ssize_t neighGridX = static_cast<ssize_t>(aGridX) + dx;
        if (neighGridX < 0 || neighGridX >= ORSP_GRID_N) continue;

        for (ssize_t dy = -ORSP_RESAMPLE_FACTOR; dy <= ORSP_RESAMPLE_FACTOR;
             dy++) {
            // Bounds check: Y
            ssize_t neighGridY = static_cast<ssize_t>(aGridY) + dy;
            if (neighGridY < 0 || neighGridY >= ORSP_GRID_N) continue;

            // Now look through all filtered points in neighbours
            const Point2DList &potentialNeighbourPoints =
                mORSPGrid[neighGridX][neighGridY];
            for (const PointCart2D &point : potentialNeighbourPoints) {
                // Check if point is within search radius
                const size_t dist = point.distance(centroid);
                if (dist <= ORSP_RADIUS) {
                    aValidNeighbours.push_back(point);
                }
            }
        }
    }

    return;
}

/**
 * @brief Estimate point distribution
 * @pre @see downsamplePointCloud() to be called first
 */
void RadarImage::estimatePointDistribution() {
    // For each centroid, search around to find neighbours within radius
    for (size_t i = 0; i < ORSP_GRID_N; i++) {
        for (size_t j = 0; j < ORSP_GRID_N; j++) {
            // First we need to check if this grid space has a valid centroid
            const Point2DList &currPoints = mORSPGrid[i][j];

            if (currPoints.size() == 0) continue;

            // Search around the centroids to find valid neighbours
            // NOTE: Smart efficiency consideration: search only within the
            // neighbouring grids
            Point2DList validNeighbours;
            findValidNeighbours(validNeighbours, i, j);

            // TODO: Check for invalid points
            // Ensure enough neighbours to consider as valid point distribution
            if (validNeighbours.size() <= ORSP_VALID_NEIGHBOUR_MIN) continue;

            // After finding valid neighbours
            // Compute mean and covariance
            Eigen::Vector2d mean;
            Eigen::Matrix2d covMatrix;
            getMeanCovariance(validNeighbours, mean, covMatrix);

            // From covariance matrix, use SVD to get eigenvectors
            Eigen::EigenSolver<Eigen::Matrix2d> eigenSolver(covMatrix);

            // Smallest eigenvector is normal vector
            Eigen::Vector2cd eigVal = eigenSolver.eigenvalues();
            Eigen::Matrix2cd eigVec = eigenSolver.eigenvectors();

            std::cout << eigVal << std::endl; 
        }
    }
}

/**
 * @brief Full pipeline for computing surface points
 * @see estimatePointDistribution()
 * @see downsamplePointCloud()
 */
void RadarImage::computeOrientedSurfacePoints(){
    downsamplePointCloud();
    estimatePointDistribution();
}