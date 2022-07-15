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
 * @param[in] aFeaturePoint Feature point in LOCAL/IMAGE coordinates
 * @param[in] aKeyframeFeaturePoint Keyframe feature point in WORLD coordinates
 */

RegistrationCostFunctor::RegistrationCostFunctor(
    const ORSP<double> &aFeaturePointLocal,
    const ORSP<double> &aKeyframeFeaturePointWorld)
    : mFeaturePointLocal(aFeaturePointLocal),
      mKeyframeFeaturePointWorld(aKeyframeFeaturePointWorld){};

/**
 * @brief Static creation of cost function with new cost functor.
 * Ceres should handle ownership.
 *
 * @param[in] aFeaturePointLocal Feature point in LOCAL/IMAGE coordinates
 * @param[in] aKeyframeFeaturePointWorld Keyframe feature point in WORLD
 * coordinates
 * @return ceres::CostFunction Ceres cost function
 */
ceres::CostFunction *RegistrationCostFunctor::Create(
    const ORSP<double> &aFeaturePointLocal,
    const ORSP<double> &aKeyframeFeaturePointWorld) {
    return (
        new ceres::AutoDiffCostFunction<RegistrationCostFunctor, NUM_RESIDUALS,
                                        POS_PARAM_SIZE, ORIENT_PARAM_SIZE>(
            new RegistrationCostFunctor(aFeaturePointLocal,
                                        aKeyframeFeaturePointWorld)));
}