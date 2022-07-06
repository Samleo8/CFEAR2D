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