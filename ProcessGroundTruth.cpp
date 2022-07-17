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
#include <vector>

#include "Pose2D.hpp"
#include "RadarFeedHandler.hpp"

namespace fs = std::filesystem;

/**
 * @brief Main function. Handles input from command line.
 *
 * Computes kstrong filtering and outputs a fancy image for easy saving. Can
 * also allow for auto saving.
 */
int main(int argc, char **argv) {
    if (argc != 2) {
        printf("Usage: %s <dataset>\n", argv[0]);
        return EXIT_FAILURE;
    }

    const unsigned int dataset = atoi(argv[1]);

    /**********************************************************************
     * @section TestRadar-KeyframeToKeyframe Compute keyframe to keyframe
     **********************************************************************/
    // TODO: Refactor the RadarFeed class so that it only saves the current and
    // maybe previous image
    fs::path dataPath(".");
    dataPath /= "data";
    dataPath /= std::to_string(dataset);

    std::vector<RotTransData> gtFeedVec;
    std::vector<std::string> aImagePathVector; // NOTE: Unused
    getDataFromFolder(dataPath, aImagePathVector, gtFeedVec);

    // Output the GT poses to a file, creating intermediate folders if
    // necessary
    fs::path poseGTOutputPath(".");
    poseGTOutputPath /= "results";
    poseGTOutputPath /= argv[1];
    poseGTOutputPath /= "poses";

    fs::create_directories(poseGTOutputPath);

    poseGTOutputPath /= "gt.txt";

    std::ofstream poseGTOutputFile;
    poseGTOutputFile.open(poseGTOutputPath,
                          std::ofstream::out | std::ofstream::trunc);

    // Read in the GT poses, as delta pose data,
    // then generate world poses to output to file
    const Pose2D<double> initPose(0, 0, 0);
    Pose2D<double> worldPose(initPose);

    poseGTOutputFile << worldPose.toString() << std::endl;

    for (const RotTransData gtOdom : gtFeedVec) {
        Pose2D<double> deltaPose(gtOdom.dx, gtOdom.dy, gtOdom.dRotRad);
        worldPose += deltaPose;

        poseGTOutputFile << worldPose.toString() << std::endl;
    }

    // Remember to close file
    poseGTOutputFile.close();

    return 0;
}