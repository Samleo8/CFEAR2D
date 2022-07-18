#include "BenchmarkHandler.hpp"

void convertPoseToKITTIFormat(const Pose2D<double> &aPose,
                              Vector3T<double> &aTransl,
                              Eigen::Quaterniond &aQuat) {
    // Populate translation vector
    aTransl(0) = aPose.position(0);
    aTransl(1) = aPose.position(1);
    aTransl(2) = 0;

    // Create an angle axis object, and then construct a quartenion from it
    Eigen::AngleAxisd angleAxis(aPose.orientation, Vector3T<double>::UnitZ());
    aQuat = angleAxis;
}

void transQuatToString(const Vector3T<double> &aTransl,
                       const Eigen::Quaterniond &aQuat,
                       std::string &aOutputString) {
    std::stringstream ss;

    ss << aTransl(0) << " " << aTransl(1) << " " << aTransl(2) << " "
       << aQuat.x() << " " << aQuat.y() << " " << aQuat.z() << " " << aQuat.w();

    aOutputString = ss.str();
}