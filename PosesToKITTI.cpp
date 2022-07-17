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

    // Output the GT poses to a file, creating intermediate folders if
    // necessary
    fs::path poseBasePath(".");
    poseBasePath /= "results";
    poseBasePath /= argv[1];
    poseBasePath /= "poses";

    fs::create_directories(poseBasePath);

    // Setup to read from input
    fs::path poseInputPath(poseBasePath);
    poseInputPath /= "poses_" + std::to_string(startID) + "_" +
                     std::to_string(endID) + ".txt";

    std::ifstream poseInputFile;
    poseInputFile.open(poseInputPath, std::ifstream::in);

    // Setup to output converted format to file
    fs::path poseOutputPath(poseBasePath);
    poseOutputPath /= "poses_kitti_" + std::to_string(startID) + "_" +
                      std::to_string(endID) + ".txt";

    std::ofstream poseOutputFile;
    poseOutputFile.open(poseOutputPath,
                        std::ofstream::out | std::ofstream::trunc);

    // Start reading and converting, outputting to file accordingly
    while (poseInputFile.good()) {
        // Read in the pose
        Pose2D<double> pose;

        std::string line;
        poseInputFile >> line;

        std::stringstream ss(line);
        double x, y, theta;
        // std::string extras;
        ss >> x >> y >> theta;
        std::cout << x << " " << y << " " << theta << std::endl;

        // Convert to KITTI format
        // Vector3T<double> transl;
        // Eigen::Quaterniond quat;
        // convertPoseToKITTIFormat(pose, transl, quat);

        // Output to file
        // poseOutputFile << transl.x() << " " << transl.y() << " " <<
        // transl.z()
        //                << " " << quat.x() << " " << quat.y() << " "
        //                << quat.z() << " " << quat.w() << "\n";
    }

    // Remember to close files
    poseOutputFile.close();
    poseInputFile.close();

    return 0;
}