/**
 * @file RegistrationCostFunctor.tpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Implementation file for functions in RegistrationCostFunctor class
 * @version 0.1
 * @date 2022-07-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __REGISTRATION_COST_FUNCTOR_TPP__
#define __REGISTRATION_COST_FUNCTOR_TPP__

/**
 * @brief Constructor for RegistrationCostFunctor::RegistrationCostFunctor
 *
 * @param[in] aRImg Radar image to register against
 * @param[in] aKFBuffer Circular buffer of keyframes
 */

RegistrationCostFunctor::RegistrationCostFunctor(
    const ORSP<double> aFeaturePoint, const Keyframe &aKeyframe)
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

/**
 * @brief The key function of the cost functor. Will return the calculated
 * cost for optimization, given the inputs, as pointers of varying lengths,
 * and the end parameter as a pointer to the residuals/costs.
 *
 * @todo Extend to 3D
 * @note Size of the input parameters are variable and are determined when
 * creating the @see AutoDiffCostFunction, handled in @ref
 * OptimisationHandler.cpp
 * @tparam T Template parameter, assume it is of a base type (like double),
 * but in reality, Ceres will use it's own Jet type for auto
 * differentiation.
 * @param[in] aPositionArray Pointer as an array of position values (x, y)
 * in this 2D case
 * @param[in] aOrientationArray Pointer as an array to orientation values
 * (theta) in this 2D case
 * @param[in] aResidualArray Pointer to residual output (can consider this
 * as an array)
 * @return Whether the function is successful.
 */
template <typename T>
bool RegistrationCostFunctor::operator()(const T *const aPositionArray,
                                         const T *const aOrientationArray,
                                         T *aResidualArray) const {
    // Build parameter object from input params
    T x = aPositionArray[0];
    T y = aPositionArray[1];
    T theta = aOrientationArray[0];

    Pose2D<T> paramsAsPose(x, y, theta);
    // paramsAsPose.orientation = theta;
    // paramsAsPose.position = Vector2T<T>(x, y);

    // Transform to be applied on ORSP points in RImage to convert to world
    // coord. Cannot be cached cos casting necessary.
    const PoseTransform2D<T> rImgTransform = poseToTransform<T>(paramsAsPose);

    // Get the ORSP point in world coordinates
    // NOTE: Need templated here, because Jacobian needed for transform
    ORSP<T> worldORSPPoint;
    ORSP<T> featurePtCasted;
    mFeaturePoint.template cast<T>(featurePtCasted);

    convertORSPCoordinates<T>(featurePtCasted, worldORSPPoint, rImgTransform);

    // Because of distance calculation, need to be templated also
    ORSP<T> closestORSPPoint;
    const bool found =
        mKeyframe.findClosestORSP<T>(worldORSPPoint, closestORSPPoint);

    // Only parse if found a match
    if (found) {
        // Compute cost according to formula
        aResidualArray[0] = closestORSPPoint.normal.dot(
            worldORSPPoint.center - closestORSPPoint.center);
    }

    return found;
}

#endif // __REGISTRATION_COST_FUNCTOR_TPP__
