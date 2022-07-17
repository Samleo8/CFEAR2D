#ifndef __BENCHMARK_HANDLER_HPP__
#define __BENCHMARK_HANDLER_HPP__

#include <fstream>
#include <vector>
#include <Eigen/Geometry>

#include "Pose2D.hpp"
#include "PoseTransformHandler.hpp"
#include "TransformDefines.hpp"

void convertPoseToKITTIFormat(const Pose2D<double> &aPose,
                              Vector3T<double> &aTransl, Eigen::Quaterniond &aQuat);

void poseMatrixToString(const Vector3T<double> &aTransl,
                        const Eigen::Quaterniond &aQuat, std::string &aOutputString);

#endif // __BENCHMARK_HANDLER_HPP__
