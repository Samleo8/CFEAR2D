/**
 * @file Pose2D.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Pose 2D Class/Struct
 * @version 0.1
 * @date 2022-07-06
 *
 * @copyright Copyright (c) 2022
 */

#ifndef __CFEAR_POSE_2D_HPP__
#define __CFEAR_POSE_2D_HPP__

#include "TransformDefines.hpp"

#include <Eigen/Geometry>

/**
 * @brief Class for storing 2D pose information as position(x,y),
 * orientation(theta)
 * @note 3D pose will be similar but orientation will probably be a quaternion
 * or set of euler angles
 * @tparam Type for pose
 */
template <typename T = double> class Pose2D {
  public:
    Vector2T<T> position = Vector2T<T>::Zero(); ///< position as x, y
    T orientation = 0;                          ///< orientation as theta

    /** @brief Default constructor */
    Pose2D() {}

    /**
     * @brief Constructor for Pose2D using position and orientation
     * vectors/scalars
     * @param[in] aPosition Position (x, y)
     * @param[in] aOrientation Orientation (theta)
     */
    Pose2D(const Vector2T<T> &aPosition, const T aOrientation)
        : position(aPosition), orientation(aOrientation) {}

    /**
     * @brief Constructor for Pose2D using (x,y,theta)
     * @param[in] aPosition Position (x, y)
     * @param[in] aOrientation Orientation (theta)
     */
    Pose2D(const T ax, const T ay, const T aOrientation)
        : orientation(aOrientation) {
        position << ax, ay;
    }

    /**
     * @brief Printing of class information using cout
     *
     * @param[in] aOutputStream Cout output stream
     * @param[in] aPose Pose class to output
     * @return std::ostream& Output stream reference
     */
    friend std::ostream &operator<<(std::ostream &aOutputStream,
                                    const Pose2D<T> &aPose) {
        aOutputStream << aPose.position << std::endl
                      << aPose.orientation << std::endl;
        return aOutputStream;
    }
};

#endif // __CFEAR_POSE_2D_HPP__