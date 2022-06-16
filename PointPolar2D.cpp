#include "PointCart2D.hpp"
#include "PointPolar2D.hpp"

void PointPolar2D::toCartesian(PointCart2D &aCart) {
    aCart.x = R * cos(theta);
    aCart.y = R * sin(theta);
}