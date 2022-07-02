/**
 * @file OptimisationHandler.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-06-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __OPTIMISATION_HANDLER_HPP__
#define __OPTIMISATION_HANDLER_HPP__

#include "Keyframe.hpp"
#include "PoseTransformHandler.hpp"
#include "RadarImage.hpp"

#include <Eigen/Geometry>
#include <Eigen/LU>

#include <boost/circular_buffer.hpp>
#include <boost/circular_buffer/base.hpp>

#define _USE_MATH_DEFINES
#include <math.h>

#define ANGLE_RAD_TO_DEG (180.0 / M_PI)
#define ANGLE_DEG_TO_RAD (M_PI / 180.0)

// TODO: Integrate with ceres somehow
// TOOD: 2D for now
typedef struct {
    // Eigen::Quaterniond q;     ///< rotation
    double theta;                ///< rotation
    Eigen::Vector2d translation; ///< translation
} OptimParams;

const size_t KF_BUFF_SIZE = 3;
typedef boost::circular_buffer<Keyframe> KeyframeBuffer;
// typedef std::vector<Keyframe> KeyframeBuffer;

/** @brief Angle tolerance threshold in radians */
const double ANGLE_TOLERANCE_RAD = 30 * ANGLE_DEG_TO_RAD;

// TODO: See
// http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres20AutoDiffCostFunctionE

const double constrainAngle(const double aAngleRad);

const double angleBetweenVectors(const Eigen::VectorXd &aVec1,
                                 const Eigen::VectorXd &aVec2);

const double point2LineCost(const RadarImage &aRImage,
                            const Keyframe &aKeyframe,
                            const OptimParams &aParams);

/**
 * @brief Cost functor for point/line image to image registration, used by
 * Ceres for optimization
 * @todo Make a proper cost function and class variables etc for this
 * @note Important part is the operator(), which will contain the actual cost
 * function. More details @see operator(). Notably, function parameters should
 * be assumed as pointers to a base type (like double) that can take differing
 * and multiple values/dimensions.
 * @see
 * http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres20AutoDiffCostFunctionE
 */
class RegistrationCostFunctor {
  private:
    double k_;

  public:
    RegistrationCostFunctor(double k) : k_(k) {}

    template <typename T>
    bool operator()(const T *const x, const T *const y, T *e) const {
        e[0] = k_ - x[0] * y[0] - x[1] * y[1];
        return true;
    }
};

#endif // __OPTIMISATION_HANDLER_HPP__