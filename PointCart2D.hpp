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

#include <Eigen/Core>
#include <opencv2/opencv.hpp>
#include <math.h>
#include <vector>

// NOTE: Forward declaration of PointPolar2D class
class PointPolar2D; 

/**
 * @brief 2D Cartesian Point class. Subclass of cv Point. Used for storing 2D Cartesian points
 */
class PointCart2D : public cv::Point2d {
  public:
    PointCart2D();
    PointCart2D(const double aX, const double aY);

    void toPolar(PointPolar2D &polar);
    void toCV(cv::Point2d &aCVPoint);
    void toEigen(Eigen::Vector2d &aEigenPoint);

    template<typename T>
    PointCart2D& operator/=(T aScalar);
};


class PointCart3D : public cv::Point3d {
  public:
    PointCart3D();
    PointCart3D(const double aX, const double aY, const double aZ);
};

/** @brief Typedef for list of 2D points */
typedef std::vector<PointCart2D> Point2DList;

/** @brief Typedef for list of 3D points */
typedef std::vector<PointCart3D> Point3DList;

#endif