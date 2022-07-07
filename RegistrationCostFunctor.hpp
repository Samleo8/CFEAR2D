/**
 * @file RegistrationCostFunctor.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Class containing definitions and functions for cost function and
 * functor used for Ceres optimization
 * @version 0.1
 * @date 2022-07-03
 *
 * @copyright Copyright (c) 2022
 * @see
 * http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres20AutoDiffCostFunctionE
 */

#ifndef __REGISTRATION_COST_FUNCTOR_HPP__
#define __REGISTRATION_COST_FUNCTOR_HPP__

#include "Keyframe.hpp"
#include "OptimisationHandler.hpp"
#include "RadarImage.hpp"

/** @brief number of residuals for registration cost optimization */
const int REGOPT_NUM_RESIDUALS = 1;

/** @brief number of values in position input vector */
const int REGOPT_POS_PARAM_SIZE = 2;

/** @brief number of values in orientation input vector */
const int REGOPT_ORIENT_PARAM_SIZE = 1;

/**
 * @brief Cost functor for point/line image to image registration, used by
 * Ceres for optimization
 * @todo Make a proper cost function and class variables etc for this
 * @note Important part is the operator(), which will contain the actual
 * cost function. More details @see operator(). Notably, function parameters
 * should be assumed as pointers to a base type (like double) that can take
 * differing and multiple values/dimensions.
 * @see
 * http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres20AutoDiffCostFunctionE
 */
class RegistrationCostFunctor {
  private:
    // TODO: is it ok if this is a reference?
    // TODO: Save only the vector of feature points instead of keyframe reference
    const ORSP<double> mFeaturePoint;
    const Keyframe &mKeyframe;

  public:
    // Constructors
    RegistrationCostFunctor(const ORSP<double> &aFeaturePoint,
                            const Keyframe &aKeyframe);

    // Getters
    const RadarImage &getRImg() const;
    const KeyframeBuffer &getKFBuffer() const;
    const Keyframe &getKeyframe(const size_t aIdx) const;

    // Cost function
    static ceres::CostFunction *Create(const RadarImage &aRImg,
                                       const KeyframeBuffer &aKFBuffer);

    // Actual cost functor
    template <typename T>
    [[nodiscard]] bool operator()(const T *const aPositionArray,
                    const T *const aOrientationArray, T *aResidualArray) const;
};

#include "RegistrationCostFunctor.tpp"

#endif // __REGISTRATION_COST_FUNCTOR_HPP__