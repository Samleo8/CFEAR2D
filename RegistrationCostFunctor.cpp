/**
 * @file RegistrationCostFunctor.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief
 * @version 0.1
 * @date 2022-07-03
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
    const RadarImage &aRImg, const KeyframeBuffer &aKFBuffer)
    : mRImg(aRImg), mKFBuffer(KeyframeBuffer) {}

/**
 * @brief The key function of the cost functor. Will return the calculated cost
 * for optimization, given the inputs, as pointers of varying lengths, and the
 * end parameter as a pointer to the residuals/costs.
 *
 * @todo Extend to 3D
 * @note Size of the input parameters are variable and are determined when
 * creating the @see AutoDiffCostFunction, handled in @ref
 * OptimisationHandler.cpp
 * @tparam T Template parameter, assume it is of a base type (like double), but
 * in reality, Ceres will use it's own Jet type for auto differentiation.
 * @param[in] aPositionArray Pointer as an array of position values (x, y) in
 * this 2D case
 * @param[in] aOrientationArray Pointer as an array to orientation values
 * (theta) in this 2D case
 * @param[in] aResidualArray Pointer to residual output (can consider this as an
 * array)
 * @return Whether the function is successful.
 */
template <typename T>
bool RegistrationCostFunctor::operator()(const T *const aPositionArray,
                                         const T *const aOrientationArray,
                                         T *aResidualArray) {
    T x = aPositionArray[0];
    T y = aPositionArray[1];
    T theta = aOrientationArray[0];
    
    // TODO: Point to line cost

    return true;
}