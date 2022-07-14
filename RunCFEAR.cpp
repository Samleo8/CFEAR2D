#include <ceres/ceres.h>
#include <filesystem>
#include <fstream>
#include <stdbool.h>
#include <stdlib.h>
#include <string>

#include "Keyframe.hpp"
#include "ORSP.hpp"
#include "OptimisationHandler.hpp"
#include "PoseTransformHandler.hpp"
#include "PoseTransformHandler.tpp"
#include "RadarFeed.hpp"
#include "RadarImage.hpp"
#include "RegistrationCostFunctor.hpp"
#include "TransformDefines.hpp"

namespace fs = std::filesystem;

/**
 * @brief Main function. Handles input from command line.
 *
 * Computes kstrong filtering and outputs a fancy image for easy saving. Can
 * also allow for auto saving.
 */
int main(int argc, char **argv) {
    if (argc < 4 || argc > 5) {
        printf(
            "Usage: %s <dataset> <startID> [endID] [0|1:saveDirectToFile]]\n",
            argv[0]);
        return EXIT_FAILURE;
    }

    const unsigned int dataset = atoi(argv[1]);
    const unsigned int startID = atoi(argv[2]);
    const int endID = (argc == 4) ? atoi(argv[3]) : -1;
    const bool saveDirectly = (argc == 5 && atoi(argv[4]));

    // Create path to save images
    fs::path saveImagesPath(".");
    saveImagesPath /= "results";
    saveImagesPath /= argv[1];

    fs::create_directories(saveImagesPath);

    // Data path
    fs::path dataPath(".");
    dataPath /= "data";
    dataPath /= std::to_string(dataset);

    // Path for pose outputs
    fs::path poseOutputPath(saveImagesPath);
    poseOutputPath /= "poses";

    fs::create_directories(poseOutputPath);

    // Construct a feed and run it
    RadarFeed feed(dataPath);
    Pose2D<double> initWorldPose(0, 0, 0);

    feed.run(startID, endID, poseOutputPath, initWorldPose);
    
    return 0;
}