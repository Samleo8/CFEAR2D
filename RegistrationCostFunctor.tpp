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
    const RadarImage &aRImg, const KeyframeBuffer &aKFBuffer)
    : mRImg(aRImg), mKFBuffer(aKFBuffer) {}

/**
 * @brief Get the internal radar image of cost functor
 *
 * @return const RadarImage& Internal radar image to register against
 */
const RadarImage &RegistrationCostFunctor::getRImg() const {
    return mRImg;
}

/**
 * @brief Get the internal keyframe buffer of cost functor
 *
 * @return const KeyframeBuffer& Internal circular buffer of keyframes to
 * register against
 */
const KeyframeBuffer &RegistrationCostFunctor::getKFBuffer() const {
    return mKFBuffer;
}

/**
 * @brief Get keyframe from internal keyframe buffer at indicated index
 *
 * @pre Index must be within bounds of internal keyframe buffer
 * @param[in] aIdx Index of keyframe buffer to get
 * @return const Keyframe& Keyframe at indicated index
 */
const Keyframe &RegistrationCostFunctor::getKeyframe(const size_t aIdx) const {
    return mKFBuffer[aIdx];
}

/**
 * @brief Cost between point to line given a radar image, keyframe, and
 * optimization parameters (in this case, a pose)
 *
 * @tparam T Type of data to use for optimization, used by Ceres
 * @param[in] aRImage Radar image to register against
 * @param[in] aKeyframe Keyframe to register against
 * @param[in] aParams Optimization parameters (in this case a pose) @see
 * OptimParams struct
 * @param[out] aOutputCost Pointer to output cost between point to line as
 indicated by cost function
 *
 * @return Successfully found cost between point to line
 */
template <typename T>
const bool RegistrationCostFunctor::point2LineCost(
    const RadarImage &aRImage, const Keyframe &aKeyframe,
    const struct OptimParams<T> &aParams, T *aOutputCost) const {
    // Transform to be applied on ORSP points in RImage to convert to world
    // coord
    const PoseTransform2D<T> rImgTransform =
        transformFromOptimParams<T>(aParams);

    // Loop through each point from ORSP point in RImage and get the cost from
    // formula
    bool foundMatch = false;

    T cost = static_cast<T>(0.0);
    const T HUBER_DELTA_DEFAULT_TEMPLATED = static_cast<T>(HUBER_DELTA_DEFAULT);

    const ORSPVec<double> rImgFeaturePts = aRImage.getORSPFeaturePoints();
    for (const ORSP<double> &featurePt : rImgFeaturePts) {
        // Get the ORSP point in world coordinates
        // NOTE: Need templated here, because Jacobian needed for transform
        ORSP<T> worldORSPPoint;
        ORSP<T> featurePtCasted;
        featurePt.template cast<T>(featurePtCasted);

        convertORSPCoordinates<T>(featurePtCasted, worldORSPPoint, rImgTransform);

        ORSP<T> closestORSPPoint;
        const bool found =
            aKeyframe.findClosestORSP<T>(worldORSPPoint, closestORSPPoint);

        // Only parse if found a match
        if (found) {
            foundMatch = true;

            // TODO: now compute cost according to formula
            T dotted = closestORSPPoint.normal.dot(
                worldORSPPoint.center - closestORSPPoint.center);
            
            // TODO: Huber loss here or from Ceres?
            cost += dotted;
            // cost +=
            //     HuberLoss<T>(dotted, HUBER_DELTA_DEFAULT_TEMPLATED);
        }
    }

    *aOutputCost = cost;
    return foundMatch;
}

/**
 * @brief Cost between point to line using internal radar image and given
 * keyframe, and optimization parameters (in this case, a pose)
 *
 * @param[in] aKeyframe Keyframe to register against
 * @param[in] aParams Optimization parameters (in this case a pose) @see
 * OptimParams struct
 * @param[out] aOutputCost Pointer to output cost
 * @param[out] aOutputCost Pointer to output cost between point to line as
 indicated by cost function
 *
 * @return Successfully found cost between point to line
 */
template <typename T>
const bool
RegistrationCostFunctor::point2LineCost(const Keyframe &aKeyframe,
                                        const struct OptimParams<T> &aParams,
                                        T *aOutputCost) const {
    return point2LineCost<T>(mRImg, aKeyframe, aParams, aOutputCost);
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

    // TODO: Need to think in terms of manifolds, and template everything

    OptimParams<T> params;
    params.theta = theta;
    params.translation = Vector2T<T>(x, y);

    // TODO: Point to line cost, sum by looping through all keyframes in the
    // buffer
    T regCost = static_cast<T>(0.0);
    bool success = false;

    for (size_t i = 0; i < mKFBuffer.size(); i++) {
        const Keyframe &keyframe = getKeyframe(i);
        T p2lCost;
        if (point2LineCost<T>(keyframe, params, &p2lCost)) {
            success = true;
            regCost += p2lCost;
        }
    }

    //     // TODO: Huber loss from Ceres?

    aResidualArray[0] = regCost;

    return success;
}

#endif // __REGISTRATION_COST_FUNCTOR_TPP__
