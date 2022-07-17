#include "RadarFeed.hpp"

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

    /**********************************************************************
     * @section TestRadar-KeyframeToKeyframe Compute keyframe to keyframe
     **********************************************************************/
    // TODO: Refactor the RadarFeed class so that it only saves the current and
    // maybe previous image
    fs::path dataPath(".");
    dataPath /= "data";
    dataPath /= std::to_string(dataset);

    RadarFeed feed(dataPath);

    // Output the GT poses to a file
    fs::path poseGTOutputPath(saveImagesPath);
    poseGTOutputPath /= "poses";

    fs::create_directories(poseGTOutputPath);

    poseGTOutputPath /= "gt.txt";

    std::ofstream poseGTOutputFile;
    poseGTOutputFile.open(poseGTOutputPath,
                        std::ofstream::out | std::ofstream::trunc);

    // Remember to close file
    poseGTOutputFile.close();

    return 0;
}