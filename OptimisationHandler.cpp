/**
 * @file OptimisationHandler.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Functions related to optimization and cost functions for bundle
 * adjustment
 * @version 0.1
 * @date 2022-06-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#include "OptimisationHandler.hpp"

const double constrainAngle(const double aAngleRad) {
    double constAngle = std::fmod(aAngleRad + M_PI, M_TAU);
    if (constAngle < 0) constAngle += M_TAU;
    return constAngle - M_PI;
}

/**
 * @brief Obtain the angle between two vectors. Used for comparing angle
 * tolerance between normals of associated points
 *
 * @param[in] aVec1 Vector 1 of arbitrary dimenion k
 * @param[in] aVec2 Vector 2 of arbitrary dimenion k
 * @return Constrained Angle between the two vectors in radians
 */
const double angleBetweenVectors(const Eigen::VectorXd &aVec1,
                                 const Eigen::VectorXd &aVec2) {
    double unnormalizedAngle =
        acos(aVec1.dot(aVec2) / (aVec1.norm() * aVec2.norm()));

    return constrainAngle(unnormalizedAngle);
}

const PoseTransform2D transformFromOptimParams(const OptimParams &aParams) {
    return rotTransToTransform(aParams.theta, aParams.translation);
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
const double point2LineCost(const RadarImage &aRImage,
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