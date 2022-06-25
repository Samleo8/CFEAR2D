/**
 * @file PoseTransformHandler.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handlers for poses and transformation matrices
 * @version 0.1
 * @date 2022-06-25
 *
 * @copyright Copyright (c) 2022
 */

#include "PoseTransformHandler.hpp"

/**
 * @brief Convert rotation and translation to transformation matrix
 *
 * @param[in] aRotMat Rotation matrix (N x N)
 * @param[in] aTrans Translation vector (N x 1)
 * @return Homogeneous transformation matrix (N+1 x N+1)
 */
const Eigen::MatrixXd poseToTransform(const Eigen::MatrixXd &aRotMat,
                                      const Eigen::VectorXd &aTrans) {
    Eigen::MatrixXd transform(aRotMat.rows() + 1, aRotMat.cols() + 1);
    transform << aRotMat, aTrans, 0, 0, 1;
    return transform;
}

/**
 * @brief Convert rotation and translation to INVERTED transformation matrix
 * Uses known formula to compute inverse of transformation matrix
 *
 * @param[in] aRotMat Rotation matrix (N x N)
 * @param[in] aTrans Translation vector (N x 1)
 * @return Homogeneous transformation matrix (N+1 x N+1)
 */
const Eigen::MatrixXd poseToTransformInverted(const Eigen::MatrixXd &aRotMat,
                                              const Eigen::VectorXd &aTrans) {
    Eigen::MatrixXd transform(aRotMat.rows() + 1, aRotMat.cols() + 1);
    const Eigen::MatrixXd rotMatTrans = aRotMat.transpose();

    transform << rotMatTrans, (-rotMatTrans * aTrans), 0, 0, 1;
    return transform;
}

/**
 * @brief Convert local to world coordinate using homogeneous transforms,
 * dynamic dimensions.
 * @pre Dimensions must be consistent:
 *      - aLocalCoordinate.rows == aWorldCoordinate.rows
 *      - aLocalCoordinate.rows + 1 == aConversionTransform.rows
 *
 * @param[in] aLocalCoordinate Local Coordinate (N x 1)
 * @param[in] aConversionTransform Homogeneous pose transform matrix from local
 * to world coordinates (N+1 x N+1)
 * @param[in] isVector Whether or not coordinate is a vector (affects
 * homogeneous value)
 *
 * @return World coordinate (N x 1)
 */
const Eigen::VectorXd
convertCoordinate(const Eigen::VectorXd &aLocalCoordinate,
                  const PoseTransformXD &aConversionTransform, bool isVector) {
    // TODO: Check for consistent dimensions
    const size_t outputDimension = aLocalCoordinate.rows();

    Eigen::VectorXd coordHomo(outputDimension + 1);
    double homoVal = (isVector) ? 0.0 : 1.0;

    coordHomo << aLocalCoordinate, homoVal;

    return (aConversionTransform * coordHomo).head(outputDimension);
}

/**
 * @brief Convert a local to a world coordinate, using locally stored world pose
 * @note (0,0) in local coordinates is center of keyframe
 *
 * @param[in] aInputORSPPoint ORSP point to be converted to different coordinate
 * @param[out] aOutputORSPPoint ORSP point in new coordinate
 * @param[in] aConversionTransform Transformation matrix used in coordinate
 * conversion
 */
void convertORSPCoordinates(const ORSP &aInputORSPPoint, ORSP &aOutputORSPPoint,
                            const PoseTransform2D &aConversionTransform) {
    aOutputORSPPoint.center =
        convertCoordinate(aInputORSPPoint.center, aConversionTransform, false);
    aOutputORSPPoint.normal =
        convertCoordinate(aInputORSPPoint.normal, aConversionTransform, true);
}
