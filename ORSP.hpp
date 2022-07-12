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
#include <vector>

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

    // template <typename BaseType>
    // ORSP(const ORSP<BaseType> &aORSP)
    //     : center(aORSP.center.template cast<T>()),
    //       normal(aORSP.normal.template cast<T>()) {}

    /**
     * @brief Templated casting function for a different type
     *
     * @tparam CastType Type to cast to
     * @param[out] aORSP ORSP point to transfer casted data into
     */
    template <typename CastType> void cast(ORSP<CastType> &aORSP) const {
        aORSP.center = center.template cast<CastType>();
        aORSP.normal = normal.template cast<CastType>();
    }

    /**
     * @brief Function to convert into printable string
     *
     * @return std::string
     */
    std::string toString() const {
        std::stringstream ss;
        ss << "Center: " << center[0] << " " << center[1];
        ss << " | Normal: " << normal[0] << " " << normal[1];
        return ss.str();
    }

    /**
     * @brief Printing of class information using cout
     *
     * @param[in] aOutputStream Cout output stream
     * @param[in] aPose Pose class to output
     * @return std::ostream& Output stream reference
     */
    friend std::ostream &operator<<(std::ostream &aOutputStream,
                                    const ORSP<T> &aORSP) {
        aOutputStream << aORSP.toString() << std::endl;
        return aOutputStream;
    }
};

/** @brief Typedef for OrientedSurfacePoint struct */
template <typename T = double> using ORSPVec = std::vector<ORSP<T>>;

#endif // __ORSP_HPP__