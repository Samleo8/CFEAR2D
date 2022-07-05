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
template <typename T> class RegistrationCostFunctor {
  private:
    // TODO: is it ok if this is a reference?
    const RadarImage mRImg;
    const KeyframeBuffer<T> mKFBuffer;

  public:
    // Constructors
    RegistrationCostFunctor(const RadarImage &aRImg,
                            const KeyframeBuffer<T> &aKFBuffer);

    // Getters
    const RadarImage &getRImg() const;
    const KeyframeBuffer<T> &getKFBuffer() const;
    const Keyframe &getKeyframe(const size_t aIdx) const;

    // Helper function for cost function
    [[nodiscard]] const bool
    point2LineCost(const RadarImage &aRImage, const Keyframe &aKeyframe,
                   const struct OptimParams<T> &aParams, T *aOutputCost) const;

    [[nodiscard]] const bool
    point2LineCost(const Keyframe &aKeyframe,
                   const struct OptimParams<T> &aParams, T *aOutputCost) const;

    /**
     * @brief The key function of the cost functor. Will return the calculated
     * cost for optimization, given the inputs, as pointers of varying lengths,
     * and the end parameter as a pointer to the residuals/costs.
     *
     * TODO: Needs to be here because Ceres needs it here :(
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
    template <typename _T>
    bool operator()(const _T *const aPositionArray,
                    const _T *const aOrientationArray,
                    _T *aResidualArray) const {
        // Build parameter object from input params
        _T x = aPositionArray[0];
        _T y = aPositionArray[1];
        _T theta = aOrientationArray[0];

        // TODO: Need to think in terms of manifolds, and template everything

        OptimParams<_T> params;
        params.theta = theta;
        params.translation = Vector2T<_T>(x, y);

        // TODO: Point to line cost, sum by looping through all keyframes in the
        // buffer
        double regCost = 0.0;
        bool success = false;

        //     for (size_t i = 0; i < mKFBuffer.size(); i++) {
        //         const Keyframe<_T> &keyframe =
        //             static_cast<Keyframe<_T>>(getKeyframe(i));
        //         _T p2lCost;
        //         if (point2LineCost(keyframe, params, &p2lCost)) {
        //             success = true;
        //             regCost += p2lCost;
        //         }
        //     }

        //     // TODO: Huber loss from Ceres?

        //     aResidualArray[0] = static_cast<_T>(regCost);

        return success;
    }
};

#include "RegistrationCostFunctor.tpp"

#endif // __REGISTRATION_COST_FUNCTOR_HPP__