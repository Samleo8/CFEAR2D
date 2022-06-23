/**
 * @file Keyframe.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for keyframes, including class and relevant internal
 * data structures, and functions to register keyframes with the incoming scan
 * frames.
 * @version 0.1
 * @date 2022-06-23
 *
 * @copyright Copyright (c) 2022
 *
 */

/**
 * @brief Constructor for Keyframe
 * 
 * @param[in] aRadarImage 
 * @param[in] aWorldPose 
 */

#include "Keyframe.hpp"

Keyframe::Keyframe(const RadarImage &aRadarImage, const Pose2D &aWorldPose) : mWorldPose(aWorldPose) {

        }