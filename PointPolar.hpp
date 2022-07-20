/**
 * @file PointPolar.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Class header for a 2D polar point
 * @date 2022-06-16
 *
 * @copyright Copyright (c) 2022
 */

#ifndef __POINT_POLAR_H__
#define __POINT_POLAR_H__

#include <Eigen/Core>
#include <math.h>

class PointPolar {
  public:
    double R;     ///< range-coordinate
    double theta; ///< azimuth-coordinate

    PointPolar() : R(0), theta(0) {}
    PointPolar(double aR, double aTheta) : R(aR), theta(aTheta) {}

    void toCartesian(Eigen::Vector2d &aCartPt);
};

class PointPolar3D {
  public:
    double R;     ///< range-coordinate
    double theta; ///< azimuth-coordinate
    double phi;   ///< elevation-coordinate

    PointPolar3D() : R(0), theta(0), phi(0) {}
    PointPolar3D(double aR, double aTheta, double aPhi)
        : R(aR), theta(aTheta), phi(aPhi) {}

    void toCartesian(Eigen::Vector3d &aCartPt);
};

#endif // __POINT_POLAR_H__