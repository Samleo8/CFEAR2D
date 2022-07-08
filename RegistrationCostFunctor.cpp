/**
 * @file RegistrationCostFunctor.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Class containing definitions and functions for cost function and
 * functor used for Ceres optimization
 * @version 0.1
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "RegistrationCostFunctor.hpp"

/**
 * @brief Constructor for RegistrationCostFunctor::RegistrationCostFunctor
 *
 * @param[in] aRImg Radar image to register against
 * @param[in] aKFBuffer Circular buffer of keyframes
 */

RegistrationCostFunctor::RegistrationCostFunctor(
    const ORSP<double> &aFeaturePoint, const Keyframe &aKeyframe)
    : mFeaturePoint(aFeaturePoint), mKeyframe(aKeyframe){};

/**
 * @brief Static creation of cost function with new cost functor.
 * Ceres should handle ownership.
 *
 * @param[in] aRImg Radar image to register against
 * @param[in] aKFBuffer Circular buffer of keyframes
 * @return ceres::CostFunction Ceres cost function
 */
ceres::CostFunction *
RegistrationCostFunctor::Create(const ORSP<double> &aFeaturePoint,
                                const Keyframe &aKeyframe) {
    return (new ceres::AutoDiffCostFunction<
            RegistrationCostFunctor, REGOPT_NUM_RESIDUALS,
            REGOPT_POS_PARAM_SIZE, REGOPT_ORIENT_PARAM_SIZE>(
        new RegistrationCostFunctor(aFeaturePoint, aKeyframe)));
}