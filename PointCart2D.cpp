#include "PointCart2D.hpp"
#include "PointPolar2D.hpp"

void PointCart2D::toPolar(PointPolar2D &polar) {
    polar.R = sqrt(x * x + y * y);
    polar.theta = atan2(y, x);
}

void PointCart2D::toCV(cv::Point2d &aCVPoint) {
    aCVPoint.x = x;
    aCVPoint.y = y;
}