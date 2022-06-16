/**
 * @file PointPolar2D.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Class header for a 2D polar point
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 */

#ifndef __POINT_POLAR_2D_H__
#define __POINT_POLAR_2D_H__

#include <math.h>

// NOTE: Forward declaration of PointCart2D class
class PointCart2D;

class PointPolar2D {
  public:
    double R;     ///< range-coordinate
    double theta; ///< azimuth-coordinate

    void toCartesian(PointCart2D &aCartPt);
};

#endif