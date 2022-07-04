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

#include "OptimisationHandler.hpp"
#include <ceres/types.h>

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
    // Constructors
    RegistrationCostFunctor(const RadarImage &aRImg,
                            const KeyframeBuffer &aKFBuffer);

    // Getters
    const RadarImage &getRImg() const;
    const KeyframeBuffer &getKFBuffer() const;
    const Keyframe &getKeyframe(const size_t aIdx) const;

    // Helper function for cost function
    [[nodiscard]] const bool point2LineCost(const RadarImage &aRImage,
                                            const Keyframe &aKeyframe,
                                            const OptimParams &aParams,
                                            double *aOutputCost) const;
    [[nodiscard]] const bool point2LineCost(const Keyframe &aKeyframe,
                                            const OptimParams &aParams,
                                            double *aOutputCost) const;

    // Actual cost function
    template <typename T>
    bool operator()(const T *const aPositionArray,
                    const T *const aOrientationArray, T *aResidualArray);
};

/**
 * @brief Cost function for image registration purposes, using auto
 *
 * @tparam CostFunctor Cost functor typename @see RegistrationCostFunctor()
 * @tparam kNumResiduals Number of residuals
 * @tparam kPositionParamSize Size/Number of position parameters (eg. x, y => 2)
 * @tparam kOrientationParamSize Size/Number of orientation parameters (eg.
 * theta => 1)
 */
template <typename CostFunctor,   // Cost functor type
          int kNumResiduals,      // Number of residuals, or ceres::DYNAMIC.
          int kPositionParamSize, // Size of each parameter block (position and
                                  // orientation)
          int kOrientationParamSize>
class RegistrationCostFunction
    : public ceres::SizedCostFunction<kNumResiduals, kPositionParamSize,
                                      kOrientationParamSize> {
  public:
    RegistrationCostFunction(CostFunctor *functor, ceres::Ownership ownership =
                                                       ceres::TAKE_OWNERSHIP);
    // Ignore the template parameter kNumResiduals and use
    // num_residuals instead.
    RegistrationCostFunction(
        CostFunctor *functor, int num_residuals,
        ceres::Ownership ownership = ceres::TAKE_OWNERSHIP);
};

#endif // __REGISTRATION_COST_FUNCTOR_HPP__