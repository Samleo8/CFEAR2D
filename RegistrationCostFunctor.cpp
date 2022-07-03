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

    // TODO: Point to line cost, sum by looping through all keyframes in the
    // buffer
    double cost = 0.0;

    return true;
}

/**
 * @brief Cost between point to line given a radar image, keyframe, and
 * optimization parameters (in this case, a pose)
 *
 * @param[in] aRImage
 * @param[in] aKeyframe
 * @param[in] aParams
 * @return const double
 */
const double
RegistrationCostFunctor::point2LineCost(const RadarImage &aRImage,
                                        const Keyframe &aKeyframe,
                                        const OptimParams &aParams) {
    // Transform to be applied on ORSP points in RImage to convert to world
    // coord
    const PoseTransform2D rImgTransform = transformFromOptimParams(aParams);

    // Loop through each point from ORSP point in RImage and get the cost from
    // formula
    bool foundMatch = false;
    double cost = 0.0;

    const ORSPVec rImgFeaturePts = aRImage.getORSPFeaturePoints();
    for (const ORSP &featurePt : rImgFeaturePts) {
        // Get the ORSP point in world coordinates
        ORSP worldORSPPoint;
        convertORSPCoordinates(featurePt, worldORSPPoint, rImgTransform);

        ORSP closestORSPPoint;
        const bool found =
            aKeyframe.findClosestORSP(worldORSPPoint, closestORSPPoint);

        // Only parse if found a match
        if (found) {
            foundMatch = true;

            // TODO: now compute cost according to formula
        }
    }

    // TODO: What if there is no match?
    if (!foundMatch) {
        return Eigen::Infinity;
    }

    return cost;
}