#include "PointCart2D.hpp"
#include "PointPolar2D.hpp"

PointCart2D::PointCart2D() : cv::Point2d(0, 0) {
}

PointCart2D::PointCart2D(const double aX, const double aY) : cv::Point2d(aX, aY) {
}

PointCart2D& PointCart2D::operator/=(size_t aScalar){
    this->x /= aScalar;
    this->y /= aScalar;

    return *this;
}

const size_t PointCart2D::distance(const PointCart2D &aPoint, const size_t norm) const {
    size_t dist = pow(abs(x - aPoint.x), norm) + pow(abs(y - aPoint.y), norm);

    return pow(dist, 1/norm);
}

void PointCart2D::toPolar(PointPolar2D &polar) {
    polar.R = sqrt(x * x + y * y);
    polar.theta = atan2(y, x);
}

void PointCart2D::toCV(cv::Point2d &aCVPoint) {
    aCVPoint.x = x;
    aCVPoint.y = y;
}

void PointCart2D::toEigen(Eigen::Vector2d &aEigenPoint) {
    aEigenPoint[0] = x;
    aEigenPoint[1] = y;
}