/**
 * @file ORSP.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Templated ORSP Representation class
 * @version 0.1
 * @date 2022-07-06
 * 
 * @copyright Copyright (c) 2022
 * 
 */

#ifndef __ORSP_HPP__
#define __ORSP_HPP__

#include "TransformDefines.hpp"

/**
 * @brief Oriented Surface Point representation, holds mean and normal
 * vector obtained from covariance
 */
template <typename T> class ORSP {
  public:
    /// @brief Mean / center of point
    Vector2T<T> center;

    /// @brief Normal vector
    Vector2T<T> normal;

    /**
     * @brief Empty constructor for OrientedSurfacePoint
     */
    ORSP() : center(Vector2T<T>::Zero()), normal(Vector2T<T>::Zero()) {}

    /**
     * @brief Constructor for OrientedSurfacePoint
     * @param[in] aCenter Center of point
     * @param[in] aNormal Normal vector
     */
    ORSP(const Vector2T<T> &aCenter, const Vector2T<T> &aNormal)
        : center(aCenter), normal(aNormal) {}

    /**
     * @brief Copy Constructor for OrientedSurfacePoint
     * @param[in] aORSP ORSP to copy
     */
    ORSP(const ORSP<T> &aORSP) : center(aORSP.center), normal(aORSP.normal) {}
};

/** @brief Typedef for OrientedSurfacePoint struct */
template <typename T = double> using ORSPVec = std::vector<ORSP<T>>;

#endif // __ORSP_HPP__