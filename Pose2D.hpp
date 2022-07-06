/**
 * @file Pose2D.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Pose 2D Class/Struct
 * @version 0.1
 * @date 2022-07-06
 *
 * @copyright Copyright (c) 2022
 */

#ifndef __POSE_2D_HPP__
#define __POSE_2D_HPP__

#include <Eigen/Geometry>

/**
 * @brief Struct for storing 2D pose information as position(x,y),
 * orientation(theta)
 * @note 3D pose will be similar but orientation will probably be a quaternion
 * or set of euler angles
 */
struct Pose2D {
    Eigen::Vector2d position = Eigen::Vector2d::Zero(); ///< position as x, y
    double orientation = 0; ///< orientation as theta

    /** @brief Default constructor */
    Pose2D() {}

    /**
     * @brief Constructor for Pose2D using position and orientation
     * vectors/scalars
     * @param[in] aPosition Position (x, y)
     * @param[in] aOrientation Orientation (theta)
     */
    Pose2D(const Eigen::Vector2d &aPosition, const double aOrientation)
        : position(aPosition), orientation(aOrientation) {}

    /**
     * @brief Constructor for Pose2D using (x,y,theta)
     * @param[in] aPosition Position (x, y)
     * @param[in] aOrientation Orientation (theta)
     */
    Pose2D(const double ax, const double ay, const double aOrientation)
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
                                    const Pose2D &aPose) {
        aOutputStream << aPose.position << std::endl
                      << aPose.orientation << std::endl;
        return aOutputStream;
    }
};

typedef struct Pose2D Pose2D; ///< typedef for struct Pose2D

#endif // __POSE_2D_HPP__