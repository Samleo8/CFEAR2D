/**
 * @file TransformDefines.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Defines and typedefs for transformation classes
 * @version 0.1
 * @date 2022-07-06
 * 
 * @copyright Copyright (c) 2022
 */
 
#ifndef __TRANSFORM_DEFINES_H__
#define __TRANSFORM_DEFINES_H__

#include <Eigen/Geometry>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

#define ANGLE_RAD_TO_DEG (180.0 / M_PI)
#define ANGLE_DEG_TO_RAD (M_PI / 180.0)

/** @brief Typedef for list of 2-dimensional points */
typedef std::vector<Eigen::Vector2d> Point2DList;

/** @brief Typedef for list of 3-dimensional points */
typedef std::vector<Eigen::Vector2d> Point3DList;

/** @brief Typedef for list of X-dimensional points */
typedef std::vector<Eigen::VectorXd> PointXDList;

/** @brief Typedef for Vector of templated type and dimension */
template <typename T, size_t Dimension>
using VectorDimT = Eigen::Matrix<T, Dimension, 1>;

/** @brief Typedef for Vector of templated dimension */
template <size_t Dimension>
using VectorDimd = Eigen::Matrix<double, Dimension, 1>;

/** @brief Typedef for NxN Matrix of templated dimension */
template <size_t Dimension>
using MatrixDimd = Eigen::Matrix<double, Dimension, Dimension>;

/** @brief Typedef for std::vector of Eigen::Vector of templated dimension */
template <size_t Dimension>
using VectorDimdList = std::vector<VectorDimd<Dimension>>;

/** @brief Typedef for 3D pose transform as Eigen isometric transform. Slightly
 * more memory but more natural. */
template <typename T>
using PoseTransform3D = Eigen::Transform<T, 3, Eigen::Isometry>;

/** @brief Typedef for 2D pose transform as Eigen isometric transform. Slightly
 * more memory but more natural. */
template <typename T>
using PoseTransform2D = Eigen::Transform<T, 2, Eigen::Isometry>;

/** @brief Typedef for a 2D vector with templated type */
template <typename T> using Vector2T = Eigen::Matrix<T, 2, 1>;

/** @brief Typedef for a 3D vector with templated type */
template <typename T> using Vector3T = Eigen::Matrix<T, 3, 1>;

/** @brief Typedef for a X-D vector with templated type */
template <typename T> using VectorXT = Eigen::Matrix<T, Eigen::Dynamic, 1>;

#endif // __TRANSFORM_DEFINES_H__