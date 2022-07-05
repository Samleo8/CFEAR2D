#ifndef __POSE_TRANSFORM_H__
#define __POSE_TRANSFORM_H__

#include <Eigen/Geometry>
#include <Eigen/LU>

#include "OrientedSurfacePointsHandler.hpp"

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

    friend std::ostream &operator<<(std::ostream &aOutputStream,
                                    const Pose2D &aPose);
};

typedef struct Pose2D Pose2D; ///< typedef for struct Pose2D

template <typename T>
const PoseTransform2D<T> rotTransToTransform(const Eigen::Rotation2D<T> &aRotMat,
                                             const Vector2T<T> &aTrans);

template <typename T>
const PoseTransform2D<T>
rotTransToTransform(const T &aAngleRad, const Vector2T<T> &aTrans);

template <typename T>
const PoseTransform2D<T> poseToTransform(const Pose2D &aPose);

template <typename T>
const Eigen::Vector2d
convertCoordinate(const Eigen::Vector2d &aCoordinate,
                  const PoseTransform2D<T> &aConversionTransform,
                  bool isVector = false);

template <typename T>
void convertORSPCoordinates(const ORSP &aLocalORSPPoint, ORSP &aWorldORSPPoint,
                            const PoseTransform2D<T> &aWorldPoseTransform);

#endif // __POSE_TRANSFORM_H__