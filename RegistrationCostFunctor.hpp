/**
 * @file RegistrationCostFunctor.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-07-03
 *
 * @copyright Copyright (c) 2022
 * @see
 * http://ceres-solver.org/nnls_modeling.html#_CPPv4N5ceres20AutoDiffCostFunctionE
 */

#ifndef __REGISTRATION_COST_FUNCTOR_HPP__
#define __REGISTRATION_COST_FUNCTOR_HPP__

#include "OptimisationHandler.hpp"

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
    // TODO: is it ok if this is a reference?
    const RadarImage mRImg;
    const KeyframeBuffer mKFBuffer;

  public:
    RegistrationCostFunctor(const RadarImage &aRImg,
                            const KeyframeBuffer &aKFBuffer);

    const double point2LineCost(const RadarImage &aRImage,
                                const Keyframe &aKeyframe,
                                const OptimParams &aParams);

    template <typename T>
    bool operator()(const T *const aPositionArray,
                    const T *const aOrientationArray, T *aResidualArray);
};

#endif // __REGISTRATION_COST_FUNCTOR_HPP__