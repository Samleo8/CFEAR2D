/**
 * @file ProcessGroundTruth.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Main function for processing ground truth data and outputting it to
 * file in world pose format
 * @version 0.1
 * @date 2022-07-17
 *
 * @copyright Copyright (c) 2022
 *
 */

#include <filesystem>
#include <fstream>
#include <iostream>
#include <sstream>
#include <string>
#include <vector>

#include "BenchmarkHandler.hpp"

namespace fs = std::filesystem;

/**
 * @brief Main function. Handles input from command line.
 *
 * Computes kstrong filtering and outputs a fancy image for easy saving. Can
 * also allow for auto saving.
 */
int main(int argc, char **argv) {
    if (argc < 3 || argc > 4) {
        printf("Usage: %s <dataset> <startID> [endID]\n", argv[0]);
        return EXIT_FAILURE;
    }

    const unsigned int dataset = atoi(argv[1]);
    const unsigned int startID = atoi(argv[2]);
    const int endID = (argc == 4) ? atoi(argv[3]) : -1;

    const bool isGT = (startID == static_cast<unsigned int>(-1));

    // Output the GT poses to a file, creating intermediate folders if
    // necessary
    fs::path poseBasePath(".");
    poseBasePath /= "results";
    poseBasePath /= argv[1];
    poseBasePath /= "poses";

    fs::create_directories(poseBasePath);

    // Setup to read from input
    fs::path poseInputPath(poseBasePath);

    if (isGT) {
        poseInputPath /= "gt.txt";
    }
    else {
        poseInputPath /= "poses_" + std::to_string(startID) + "_" +
                         std::to_string(endID) + ".txt";
    }

    std::ifstream poseInputFile(poseInputPath, std::ifstream::in);

    // Setup to output converted format to file
    fs::path poseOutputPath(poseBasePath);

    if (isGT) {
        poseOutputPath /= "stamped_groundtruth.txt";
    }
    else if (startID == 0 && endID == -1) {
        poseOutputPath /= "stamped_traj_estimate.txt";
    }
    else {
        poseOutputPath /= "poses_kitti_" + std::to_string(startID) + "_" +
                          std::to_string(endID) + ".txt";
    }
    std::ofstream poseOutputFile(poseOutputPath,
                                 std::ofstream::out | std::ofstream::trunc);

    // Start reading and converting, outputting to file accordingly
    size_t lineCount = 0;
    std::string line;
    while (std::getline(poseInputFile, line)) {
        double x, y, theta;
        std::string extra1;

        std::stringstream ss(line);
        ss >> extra1 >> x >> y >> theta;

        // Read in the pose
        Pose2D<double> pose(x, y, theta);

        // Convert to KITTI format
        Vector3T<double> transl;
        Eigen::Quaterniond quat;
        convertPoseToKITTIFormat(pose, transl, quat);

        // Output to file
        std::string output;
        transQuatToString(transl, quat, output);
        poseOutputFile << lineCount << " " << output << "\n";
        
        // Increment line count
        lineCount++;
    }

    std::cout << "Poses converted to KITTI format in " << poseOutputPath << "\n";

    return 0;
}