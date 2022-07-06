/**
 * @file PointPolar.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Simple point polar class that handles conversion to Cartesian
 * coordinates
 * @version 0.1
 * @date 2022-07-06
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "PointPolar.hpp"

/**
 * @brief Conversion of polar coordinates to Cartesian coordinates
 *
 * @param[in] aCart
 */
void PointPolar::toCartesian(Eigen::Vector2d &aCart) {
    double x = R * cos(theta);
    double y = R * sin(theta);

    aCart << x, y;
}

/**
 * @brief Conversion of polar coordinates to Cartesian coordinates
 *
 * @param[in] aCart
 */
void PointPolar3D::toCartesian(Eigen::Vector3d &aCart) {
    const double RsinPhi = R * sin(phi);

    double x = RsinPhi * cos(theta);
    double y = RsinPhi * sin(theta);
    double z = R * cos(phi);

    aCart << x, y;
}