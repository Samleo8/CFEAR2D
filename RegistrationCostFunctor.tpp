/**
 * @file RegistrationCostFunctor.tpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Implementation of functions in RegistrationCostFunctor class
 * @version 0.1
 * @date 2022-07-03
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __REGISTRATION_COST_FUNCTOR_TPP__
#define __REGISTRATION_COST_FUNCTOR_TPP__

/**
 * @brief Constructor for RegistrationCostFunctor<T>::RegistrationCostFunctor
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
const KeyframeBuffer &RegistrationCostFunctor<T>::getKFBuffer() const {
    return mKFBuffer;
}

/**
 * @brief Get keyframe from internal keyframe buffer at indicated index
 *
 * @pre Index must be within bounds of internal keyframe buffer
 * @param[in] aIdx Index of keyframe buffer to get
 * @return const Keyframe& Keyframe at indicated index
 */
const Keyframe &
RegistrationCostFunctor::getKeyframe(const size_t aIdx) const {
    return mKFBuffer[aIdx];
}

/**
 * @brief Cost between point to line given a radar image, keyframe, and
 * optimization parameters (in this case, a pose)
 *
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
    double cost = 0.0;

    const ORSPVec rImgFeaturePts = aRImage.getORSPFeaturePoints();
    for (const ORSP &featurePt : rImgFeaturePts) {
        // Get the ORSP point in world coordinates
        ORSP worldORSPPoint;
        convertORSPCoordinates<T>(featurePt, worldORSPPoint, rImgTransform);

        ORSP closestORSPPoint;
        const bool found =
            aKeyframe.findClosestORSP(worldORSPPoint, closestORSPPoint);

        // Only parse if found a match
        if (found) {
            foundMatch = true;

            // TODO: now compute cost according to formula
            cost += closestORSPPoint.normal.dot(worldORSPPoint.center -
                                                closestORSPPoint.center);
        }
    }

    // TODO: Huber loss here or from Ceres?
    *aOutputCost = HuberLoss<T>(cost);
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
    return point2LineCost(mRImg, aKeyframe, aParams, aOutputCost);
}

#endif // __REGISTRATION_COST_FUNCTOR_TPP__