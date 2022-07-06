/**
 * @file OrientedSurfacePointsHandler.tpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Templated implementation for functions in
 * OrientedSurfacePointsHandler.hpp
 * @version 0.1
 * @date 2022-07-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __CFEAR_ORSP_HANDLER_TPP__
#define __CFEAR_ORSP_HANDLER_TPP__

/**
 * @brief Get distance (2-norm) between 2 points/vectors in X-Dim space
 *
 * @param[in] aVec1 First vector
 * @param[in] aVec2 Second vector
 * @return Distance between these 2 vectors (2-norm)
 */
template <typename T>
const T getDistance(const VectorXT<T> &aVec1, const VectorXT<T> &aVec2) {
    return (aVec1 - aVec2).norm();
}

/**
 * @brief Get Centroid from list of X-D points
 *
 * @param[in] aPoints List of points
 * @return const Point2D
 */
template <size_t Dimension>
const VectorDimd<Dimension>
getCentroid(const VectorDimdList<Dimension> &aPoints) {
    Eigen::VectorXd centroid = Eigen::VectorXd::Zero(Dimension);

    // Empty list: Return (0,0) for centroid
    if (aPoints.size() == 0) return centroid;

    // Otherwise, calculate centroid by summing up all coordinates
    for (const VectorDimd<Dimension> &point : aPoints) {
        centroid += point;
    }

    // And dividing by size
    size_t sz = aPoints.size();
    return centroid / sz;
}

/**
 * @brief Get mean and covariance matrix from list of 2D points
 *
 * @param[in] aPoints
 * @param[in] aMean
 * @param[in] aCovMatrix
 */
template <size_t Dimension>
void getMeanCovariance(const VectorDimdList<Dimension> &aPoints,
                       VectorDimd<Dimension> &aMean,
                       MatrixDimd<Dimension> &aCovMatrix) {
    // Convert list of points into Eigen matrix, then use vectorization
    // NOTe: Eigen is col-major, so colwise access is faster.
    const size_t sz = aPoints.size();
    Eigen::MatrixXd pointsListMat(Dimension, sz);

    for (size_t i = 0; i < sz; i++) {
        pointsListMat.col(i) = aPoints[i];
    }
    pointsListMat.transposeInPlace();

    // Use vectorisation to get mean and covariance
    aMean = pointsListMat.colwise().mean();

    Eigen::MatrixXd deltaExpected(sz, Dimension);
    deltaExpected = pointsListMat.rowwise() - aMean.transpose();

    aCovMatrix = deltaExpected.transpose() * deltaExpected / sz;
}

#endif