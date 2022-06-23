/**
 * @file Keyframe.hpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for keyframes, containing class and relevant internal data
 * structures, and functions to register keyframes with the incoming scan
 * frames.
 *
 * Each keyframe has a list of feature points associated with this keyframe
 * a grid representation containing these feature points transferred
 * from RadarImage (in local coordinates), and a world pose
 *
 * @version 0.1
 * @date 2022-06-23
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __KEYFRAME_H__
#define __KEYFRAME_H__

#include "Eigen/Core"
#include "RadarImage.hpp"

typedef Eigen::Matrix3d Pose2D;

class Keyframe {
  private:
    const Pose2D mWorldPose;

  public:
    Keyframe(const RadarImage &aRadarImage, const Pose2D &aWorldPose);
    ~Keyframe();
};

#endif // __KEYFRAME_H__