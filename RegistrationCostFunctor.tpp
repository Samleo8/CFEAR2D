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

#include "OptimisationHandler.hpp" // for findClosestORSPInSet

/**
 * @brief The key function of the cost functor. Will return the calculated
 * cost for optimization, given the inputs, as pointers of varying lengths,
 * and the end parameter as a pointer to the residuals/costs.
 * @note If a match is not found, the cost is simply set to a large value @see DEFAULT_FAIL_COST
 * @todo Not sure if i should be removing the entire ResidualBlock instead of having a large cost.
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
    // Build parameter object from input params
    const T &x = aPositionArray[0];
    const T &y = aPositionArray[1];
    const T &theta = aOrientationArray[0];

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
    const bool found = findClosestORSPInSet<T>(
        worldORSPPoint, mKeyframeFeaturePoints, closestORSPPoint);

    // std::cout << "Found: " << found << std::endl
    //           << worldORSPPoint.center << std::endl
    //           << closestORSPPoint.center << std::endl
    //           << std::endl;

    // Only parse if found a match
    if (found) {
        // Compute cost according to formula
        aResidualArray[0] = closestORSPPoint.normal.dot(
            worldORSPPoint.center - closestORSPPoint.center);

        // std::cout << "Cost: " << aResidualArray[0] << std::endl;
    }
    else {
        // std::cout << "Failed!" << std::endl;
        aResidualArray[0] = static_cast<T>(DEFAULT_FAIL_COST);
    }

    return true;
}

#endif // __REGISTRATION_COST_FUNCTOR_TPP__
