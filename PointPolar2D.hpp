/**
 * @file PointPolar2D.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Class header for a 2D polar point
 * @date 2022-06-16
 *
 * @copyright Copyright (c) 2022
 */

#ifndef __POINT_POLAR_H__
#define __POINT_POLAR_H__

#include <math.h>
#include <Eigen/Core>

class PointPolar2D {
  public:
    double R;     ///< range-coordinate
    double theta; ///< azimuth-coordinate

    void toCartesian(Eigen::Vector2d &aCartPt);
};

class PointPolar3D {
  public:
    double R;     ///< range-coordinate
    double theta; ///< azimuth-coordinate
    double phi;   ///< elevation-coordinate

    void toCartesian(Eigen::Vector3d &aCartPt);
};

#endif // __POINT_POLAR_H__