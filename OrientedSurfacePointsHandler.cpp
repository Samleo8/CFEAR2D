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
#include <Eigen/Core>

/**
 * @brief Associate filtered point coordinate to downsampled grid coordinate
 * @param[in] aPoint Filtered point to associate
 * @param[out] aGridCoordinate Output grid coordinate
 */
void pointToGridCoordinate(const Eigen::Vector2d &aPoint,
                           Eigen::Vector2d &aGridCoordinate,
                           const Eigen::Vector2d &aGridCenter) {
    // TODO: Check correctness
    // NOTE: Need to add grid center to convert to correct coordinate system
    // where (0,0) is at top left
    // In the default, non keyframe case, this is (MAX_RANGE_M, MAX_RANGE_M)
    // aGridCoordinate[0] =
    //     floor((aPoint[0] + aGridCenter[0]) / ORSP_GRID_SQUARE_WIDTH);
    // aGridCoordinate[1] =
    //     floor((aPoint[1] + aGridCenter[1]) / ORSP_GRID_SQUARE_WIDTH);

    // Vectorized operations, TODO: also allowing for multidimensional cases?
    aGridCoordinate = (aPoint + aGridCenter) / ORSP_GRID_SQUARE_WIDTH;

    // TODO: Floor the thing but Eigen 3.37 doesnt have floor() yet, now rely on
    // int casting

    // TODO: Bound aGridCoordinate to be within grid
}

/**
 * @brief Getter for vector of oriented surface feature points
 *
 * @return const ORSPVec<double>& Vector of internal ORSP points
 */
const ORSPVec<double> &RadarImage::getORSPFeaturePoints() const {
    return mORSPFeaturePoints;
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
    std::cout << "Downsampling point cloud of size " << sz << " into "
              << ORSP_GRID_N << " x " << ORSP_GRID_N << " grid... "
              << std::flush;

    for (size_t i = 0; i < sz; i++) {
        const FilteredPoint &filtPt = filteredPoints[i];
        Eigen::Vector2d gridCoordinate;
        pointToGridCoordinate(filtPt, gridCoordinate);

        size_t gridX = static_cast<size_t>(gridCoordinate[0]);
        size_t gridY = static_cast<size_t>(gridCoordinate[1]);

        // std::cout << "Raw XY: (" << filtPt.x << ", " << filtPt.y << ") | ";
        // std::cout << "Grid coordinate: (" << gridX << ", " << gridY << ")" <<
        // std::endl;

        mORSPGrid[gridX][gridY].push_back(filteredPoints[i]);
    }

    // Now find the centroids
    for (size_t i = 0; i < ORSP_GRID_N; i++) {
        for (size_t j = 0; j < ORSP_GRID_N; j++) {
            mORSPCentroidGrid[i][j] = getCentroid<2>(mORSPGrid[i][j]);
        }
    }

    std::cout << "Done!" << std::endl;
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
void RadarImage::findValidNeighbours(Point2DList &aValidNeighbours,
                                     const size_t aGridX, const size_t aGridY) {
    // Prep valid neighbours
    aValidNeighbours.clear();

    const Eigen::Vector2d centroid = mORSPCentroidGrid[aGridX][aGridY];

    // Check around the grid square but only up to sampling factor
    const ssize_t gridX = static_cast<ssize_t>(aGridX);
    const ssize_t gridY = static_cast<ssize_t>(aGridY);
    constexpr ssize_t f = static_cast<ssize_t>(ORSP_RESAMPLE_FACTOR);
    constexpr ssize_t N = static_cast<ssize_t>(ORSP_GRID_N);

    for (ssize_t dx = -f; dx <= f; dx++) {
        // Bounds check: X
        ssize_t neighGridX = gridX + dx;
        if (neighGridX < 0 || neighGridX >= N) continue;

        for (ssize_t dy = -f; dy <= f; dy++) {
            // Bounds check: Y
            ssize_t neighGridY = gridY + dy;
            if (neighGridY < 0 || neighGridY >= N) continue;

            // Now look through all filtered points in neighbours
            const Point2DList &potentialNeighbourPoints =
                mORSPGrid[neighGridX][neighGridY];

            for (const Eigen::Vector2d &point : potentialNeighbourPoints) {
                // Check if point is within search radius
                // TODO: Extra efficiency constraint: if dx == dy == 0, then
                // guarenteed to be in
                const double dist = getDistance<double>(point, centroid);

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
 *
 * @note Updates internal ORSP feature points list
 */
void RadarImage::estimatePointDistribution() {
    std::cout << "Estimating point distribution... " << std::flush;

    // Empty existing list
    mORSPFeaturePoints.clear();

    // For each centroid, search around to find neighbours within radius
    for (size_t i = 0; i < ORSP_GRID_N; i++) {
        for (size_t j = 0; j < ORSP_GRID_N; j++) {
            // First we need to check if this grid space has a valid centroid
            const Point2DList &currPoints = mORSPGrid[i][j];

            // No valid centroid, skip
            if (currPoints.size() == 0) continue;

            // std::cout << "Parsing centroid at grid (" << i << ", " << j <<
            // ")... ";

            // Search around the grid square to find valid neighbours
            // NOTE: Smart efficiency consideration: search only within the
            // neighbouring grids
            Point2DList validNeighbours;
            findValidNeighbours(validNeighbours, i, j);

            // std::cout << "Found " << validNeighbours.size() << " valid
            // neighbours!" << std::endl;

            // TODO: Check for invalid points
            // Ensure enough neighbours to consider as valid point distribution
            if (validNeighbours.size() < ORSP_VALID_NEIGHBOUR_MIN) continue;

            // After finding valid neighbours, consider this as a potential
            // feature point Compute mean and covariance
            Eigen::Vector2d mean;
            Eigen::Vector2d normalVector;
            Eigen::Matrix2d covMatrix;

            getMeanCovariance<2>(validNeighbours, mean, covMatrix);

            // From covariance matrix, use SVD to get eigenvectors
            Eigen::EigenSolver<Eigen::Matrix2d> eigenSolver(covMatrix);

            // Smallest eigenvector is normal vector
            Eigen::Vector2d eigVal = eigenSolver.eigenvalues().real();
            Eigen::Matrix2d eigVec = eigenSolver.eigenvectors().real();

            double eigVal1 = eigVal(0);
            double eigVal2 = eigVal(1);

            if (eigVal1 < eigVal2) {
                // Fails condition criteria (ratio too high => ill-defined
                // distribution)
                if ((eigVal2 / eigVal1) > ORSP_EIGENVAL_THRESHOLD) continue;

                normalVector = eigVec.col(0);
            }
            else {
                // Fails condition criteria (ratio too high => ill-defined
                // distribution)
                if ((eigVal1 / eigVal2) > ORSP_EIGENVAL_THRESHOLD) continue;

                normalVector = eigVec.col(1);
            }

            // Build the oriented feature point and add to list
            ORSP<double> featurePt;
            featurePt.center = mean;
            featurePt.normal = normalVector;

            mORSPFeaturePoints.push_back(featurePt);
        }
    }

    std::cout << "Done!" << std::endl;
    std::cout << "Found " << mORSPFeaturePoints.size()
              << " oriented surface feature points." << std::endl;
}

/**
 * @brief Full pipeline for computing surface points
 * @see estimatePointDistribution()
 * @see downsamplePointCloud()
 */
void RadarImage::computeOrientedSurfacePoints() {
    downsamplePointCloud();
    estimatePointDistribution();
}

/**
 * @brief Clear ORSP related information, including grid, feature points vector,
 * etc. Needed for a clean slate for incremental radar image in radar feed
 */
void RadarImage::clearORSPInfo() {
    // Clear saved points
    mFilteredPoints.clear();
    mORSPFeaturePoints.clear();

    // Clear grid information
    for (size_t i = 0; i < ORSP_GRID_N; i++) {
        for (size_t j = 0; j < ORSP_GRID_N; j++) {
            mORSPGrid[i][j].clear();
            mORSPCentroidGrid[i][j].setZero();
        }
    }
}