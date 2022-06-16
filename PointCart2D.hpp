/**
 * @file PointCart2D.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Class for 2D Cartesian Point
 * @version 0.1
 * @date 2022-06-16
 * 
 * @copyright Copyright (c) 2022
 * 
 */
#ifndef __POINT_CART_2D_H__
#define __POINT_CART_2D_H__

#include <math.h>

// NOTE: Forward declaration of PointPolar2D class
class PointPolar2D; 

/**
 * @brief 2D Cartesian Point struct. Used for storing 2D Cartesian points
 */
class PointCart2D {
  public:
    double x; ///< X-coordinate
    double y; ///< Y-coordinate

    void toPolar(PointPolar2D &polar);
};

#endif