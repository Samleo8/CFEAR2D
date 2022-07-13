#include <ceres/ceres.h>
#include <filesystem>
#include <fstream>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "CVColor.hpp"
#include "Keyframe.hpp"
#include "ORSP.hpp"
#include "OptimisationHandler.hpp"
#include "PoseTransformHandler.hpp"
#include "PoseTransformHandler.tpp"
#include "RadarFeed.hpp"
#include "RadarImage.hpp"
#include "RegistrationCostFunctor.hpp"
#include "TransformDefines.hpp"

/**
 * @brief Threshold for checking how far the vehicle has to move before counted
 * as non stationary
 */
constexpr double DIST_STATIONARY_THRESH = 0.05; // 5cm
constexpr double DIST_STATIONARY_THRESH_SQ =
    DIST_STATIONARY_THRESH * DIST_STATIONARY_THRESH;

namespace fs = std::filesystem;

/**
 * @brief Enum output image style
 * @see outputImgFromFrames()
 */
enum PerturbStyle { TRANS_ONLY, ROT_ONLY, BOTH };

/**
 * @brief Print ORSP points to file, potentially allowing for transform
 * 
 * @param[in] orspList 
 * @param[in] orspBaseOutputPath 
 * @param[in] frameID 
 * @param[in] doTransform 
 * @param[in] coordTransform 
 * 
 */
void printORSPToFile(const ORSPVec<double> orspList,
                     const fs::path &orspBaseOutputPath, const int frameID,
                     const bool doTransform = false, 
                     const PoseTransform2D<double> &coordTransform = PoseTransform2D<double>::Identity()) {
    std::ofstream orspOutputFile;

    fs::path orspFileOutputPath(orspBaseOutputPath);
    orspFileOutputPath /= "orsp_" + std::to_string(frameID) + ".txt";

    orspOutputFile.open(orspFileOutputPath,
                        std::ofstream::out | std::ofstream::trunc);

    for (const ORSP<double> &orsp : orspList) {
        ORSP<double> worldORSPPoint;
        if (doTransform) {
            convertORSPCoordinates(orsp, worldORSPPoint, coordTransform);
        }
        else {
            worldORSPPoint = orsp;
        }

        orspOutputFile << worldORSPPoint.toString() << std::endl;
    }

    orspOutputFile.close();
}

/**
 * @brief Main function. Handles input from command line.
 *
 * Computes kstrong filtering and outputs a fancy image for easy saving. Can
 * also allow for auto saving.
 */
int main(int argc, char **argv) {
    if (argc != 3) {
        printf("Usage: %s <dataset> <startID>\n", argv[0]);
        return EXIT_FAILURE;
    }

    const unsigned int dataset = atoi(argv[1]);
    const unsigned int startID = atoi(argv[2]);

    /**********************************************************************
     * @section TestRadar-KeyframeToKeyframe Compute keyframe to keyframe
     **********************************************************************/
    // TODO: Refactor the RadarFeed class so that it only saves the current and
    // maybe previous image
    fs::path dataPath(".");
    dataPath /= "data";
    dataPath /= std::to_string(dataset);

    RadarFeed feed(dataPath);

    feed.loadFrame(startID);
    RadarImage currRImg;
    feed.getCurrentRadarImage(currRImg);

    // K-filtering and ORSP
    const size_t K = 12;
    const double Z_min = 55;

    currRImg.performKStrong(K, Z_min);
    currRImg.computeOrientedSurfacePoints();

    // Saving of previous and current world pose (initial pose for previous
    // frame)
    Pose2D<double> currWorldPose(0, 0, 0);
    Pose2D<double> prevWorldPose(currWorldPose);

    // First image is always a keyframe. Push it to the buffer
    // Update the previous keyframe world pose used to deduce if another
    // keyframe needs to be added
    KeyframeBuffer keyframeList{ KF_BUFF_SIZE };

    Keyframe keyframe(currRImg, currWorldPose);
    keyframeList.push_back(keyframe);

    // Output ORSP to file for debugging, if flag specified
    fs::path orspBaseOutputPath(saveImagesPath);
    orspBaseOutputPath /= "orsp_debug";

    fs::create_directories(orspBaseOutputPath);

    // Output the ORSP points for the base keyframe too
    const ORSPVec<double> &orspList = currRImg.getORSPFeaturePoints();
    printORSPToFile(orspList, orspBaseOutputPath, startID, false);
    

    return 0;
}