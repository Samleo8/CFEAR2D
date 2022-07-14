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

#include <ceres/jet.h>
#include <ostream>

/**
 * @brief The key function of the cost functor. Inputs are in the form of
 * templated pointers (thought of as doubles but are actually ceres::Jets) that,
 * in this case, refer to poses/transform parameters. Will return the calculated
 * cost between associated keyframe ORSP point and feature point after it is
 * transformmed into world coordinates using the provided transform.
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
 * @return whether the cost function successfully completed
 */
template <typename T>
bool RegistrationCostFunctor::operator()(const T *const aPositionArray,
                                         const T *const aOrientationArray,
                                         T *aResidualArray) const {
    // Get local to world transform from input parameters
    const PoseTransform2D<T> rImgL2WTransform =
        paramsToTransform<T>(aPositionArray, aOrientationArray);

    // NOTE: Need casted/templated coordinate here, because Jacobian needed for
    // transform
    ORSP<T> featurePtLocalCasted;
    mFeaturePointLocal.template cast<T>(featurePtLocalCasted);

    // Get the radar image ORSP point in world coordinates
    ORSP<T> rImgFeaturePtWorld;
    convertORSPCoordinates<T>(featurePtLocalCasted, rImgFeaturePtWorld,
                              rImgL2WTransform);

    // Because of distance calculation, need to be casted/templated also
    ORSP<T> keyframeFeaturePtWorldCasted;
    mKeyframeFeaturePointWorld.template cast<T>(keyframeFeaturePtWorldCasted);

    // Compute cost according to formula
    aResidualArray[0] = keyframeFeaturePtWorldCasted.normal.dot(
        rImgFeaturePtWorld.center - keyframeFeaturePtWorldCasted.center);

// For debugging, print out the residuals
#ifdef __DEBUG_RESIDUALS__
#include <type_traits>
    if constexpr (std::is_same<T, ceres::Jet<double, 3>>::value)
        std::cout << aResidualArray[0].a << " " << std::flush;
#endif

    return true;
}

#endif // __REGISTRATION_COST_FUNCTOR_TPP__
