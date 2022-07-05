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
#include "OptimisationHandler.hpp"

/**
 * @brief Constructor for RegistrationCostFunctor<T>::RegistrationCostFunctor
 *
 * @param[in] aRImg Radar image to register against
 * @param[in] aKFBuffer Circular buffer of keyframes
 */
template <typename T>
RegistrationCostFunctor<T>::RegistrationCostFunctor(
    const RadarImage &aRImg, const KeyframeBuffer<T> &aKFBuffer)
    : mRImg(aRImg), mKFBuffer(aKFBuffer) {}

/**
 * @brief Get the internal radar image of cost functor
 *
 * @return const RadarImage& Internal radar image to register against
 */
template <typename T>
const RadarImage &RegistrationCostFunctor<T>::getRImg() const {
    return mRImg;
}

/**
 * @brief Get the internal keyframe buffer of cost functor
 *
 * @return const KeyframeBuffer& Internal circular buffer of keyframes to
 * register against
 */
template <typename T>
const KeyframeBuffer<T> &RegistrationCostFunctor<T>::getKFBuffer() const {
    return mKFBuffer;
}

/**
 * @brief Get keyframe from internal keyframe buffer at indicated index
 *
 * @pre Index must be within bounds of internal keyframe buffer
 * @param[in] aIdx Index of keyframe buffer to get
 * @return const Keyframe& Keyframe at indicated index
 */
template <typename T>
const Keyframe<T> &
RegistrationCostFunctor<T>::getKeyframe(const size_t aIdx) const {
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
const bool RegistrationCostFunctor<T>::point2LineCost(
    const RadarImage &aRImage, const Keyframe<T> &aKeyframe,
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
RegistrationCostFunctor<T>::point2LineCost(const Keyframe<T> &aKeyframe,
                                           const struct OptimParams<T> &aParams,
                                           T *aOutputCost) const {
    return point2LineCost(mRImg, aKeyframe, aParams, aOutputCost);
}