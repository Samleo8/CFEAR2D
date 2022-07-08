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
#include <ostream>

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
    T orientation = static_cast<T>(0.0);        ///< orientation as theta

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
    Pose2D(const T &ax, const T &ay, const T &aOrientation)
        : orientation(aOrientation) {
        position << ax, ay;
    }

    /**
     * @brief Perform a deep copy from other Pose2D class
     *
     * @param[in] aPose Pose2D to copy from
     */
    void copyFrom(const Pose2D<T> &aPose) {
        position[0] = aPose.position[0];
        position[1] = aPose.position[1];

        orientation = aPose.orientation;
    }

    /**
     * @brief Operator += for Pose2D class. Adds another Pose2D (probably delta)
     * to this one.
     *
     * @param[in] aOtherPose Other pose
     *
     * @return Pose2D<T>& Reference to this pose
     */
    Pose2D<T> &operator+=(const Pose2D<T> &aOtherPose) {
        position += aOtherPose.position;
        orientation += aOtherPose.orientation;

        return *this; // return the result by reference
    }

    /**
     * @brief Operator -= for Pose2D class. Subtracts another Pose2D (probably
     * delta) from this one.
     *
     * @param[in] aOtherPose
     *
     * @return Pose2D<T>&
     */
    Pose2D<T> &operator-=(const Pose2D<T> &aOtherPose) {
        position -= aOtherPose.position;
        orientation -= aOtherPose.orientation;

        return *this; // return the result by reference
    }

    /**
     * @brief Return pose 2D as string
     *
     * @return const std::string
     */
    const std::string toString() const {
        std::stringstream ss;
        ss << "Pose2D: " << position[0] << " " << position[1] << " "
           << orientation;
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
                                    const Pose2D<T> &aPose) {
        aOutputStream << aPose.position << std::endl
                      << aPose.orientation << std::endl;
        return aOutputStream;
    }
};

#endif // __CFEAR_POSE_2D_HPP__