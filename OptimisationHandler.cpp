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

#include "RadarImage.hpp"
#include "Keyframe.hpp"
#include "OptimisationHandler.hpp"

/**
 * @brief Obtain the angle between two vectors. Used for comparing angle
 * tolerance between normals of associated points
 *
 * @param[in] aVec1 Vector 1 of arbitrary dimenion k
 * @param[in] aVec2 Vector 2 of arbitrary dimenion k
 * @return Angle between the two vectors in radians
 */
double angleBetweenVectors(const Eigen::VectorXd &aVec1,
                           const Eigen::VectorXd &aVec2) {
    return acos(aVec1.dot(aVec2) / (aVec1.norm() * aVec2.norm()));
}

/**
 * @brief Cost between point to line given a radar image, keyframe, and optimization parameters (in this case, a pose)
 * 
 * @param[in] aRImage 
 * @param[in] aKeyframe 
 * @param[in] aParams 
 * @return const double 
 */
const double point2LineCost(const RadarImage &aRImage,
                            const Keyframe &aKeyframe,
                            const OptimParams &aParams) {
    return 0;
}