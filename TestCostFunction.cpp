#include <ceres/ceres.h>
#include <filesystem>
#include <fstream>
#include <random>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "CVColor.hpp"
#include "Keyframe.hpp"
#include "ORSP.hpp"
#include "OptimisationHandler.hpp"
#include "Pose2D.hpp"
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

const PerturbStyle perturbMode = PerturbStyle::BOTH;

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
void printORSPToFile(const ORSPVec<double> &orspList,
                     const fs::path &orspBaseOutputPath, const int frameID,
                     const bool doTransform = false,
                     const PoseTransform2D<double> &coordTransform =
                         PoseTransform2D<double>::Identity()) {
    std::ofstream orspOutputFile;

    fs::path orspFileOutputPath(orspBaseOutputPath);
    orspFileOutputPath /= "orsp_" + std::to_string(frameID) + ".txt";

    orspOutputFile.open(orspFileOutputPath,
                        std::ofstream::out | std::ofstream::trunc);

    for (const ORSP<double> &orsp : orspList) {
        ORSP<double> worldORSPPoint;
        if (doTransform) {
            convertORSPCoordinates<double>(orsp, worldORSPPoint,
                                           coordTransform);
        }
        else {
            worldORSPPoint.center = orsp.center;
            worldORSPPoint.normal = orsp.normal;
        }

        orspOutputFile << worldORSPPoint.toString() << std::endl;
    }

    orspOutputFile.close();
}

void addNoiseToTransform(const PoseTransform2D<double> &aInputTransform,
                         PoseTransform2D<double> &aOutputTransform,
                         const double aTransStdDev = 0.1,
                         const double aRotDegStdDev = 5) {
    std::default_random_engine generator;
    std::uniform_real_distribution<double> distribution(-aRotDegStdDev,
                                                        aRotDegStdDev);
    double rot = distribution(generator) * ANGLE_DEG_TO_RAD;

    // Generates random vector with values from -1 to 1
    Vector2T<double> trans = Vector2T<double>::Random() * aTransStdDev;

    PoseTransform2D<double> noiseTransform = rotTransToTransform(rot, trans);

    aOutputTransform = noiseTransform * aInputTransform;
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
    Pose2D<double> currWorldPose(20, 50, 0.5);
    Pose2D<double> prevWorldPose(currWorldPose);

    // First image is always a keyframe. Push it to the buffer
    // Update the previous keyframe world pose used to deduce if another
    // keyframe needs to be added
    KeyframeBuffer keyframeList{ KF_BUFF_SIZE };

    Keyframe keyframe(currRImg, currWorldPose);
    keyframeList.push_back(keyframe);

    // Output ORSP to file for debugging, if flag specified
    fs::path orspBaseOutputPath("results");
    orspBaseOutputPath /= argv[1];
    orspBaseOutputPath /= "orsp_debug";

    fs::create_directories(orspBaseOutputPath);

    // Output the ORSP points for the base keyframe too
    const ORSPVec<double> &orspList = currRImg.getORSPFeaturePoints();
    printORSPToFile(orspList, orspBaseOutputPath, startID, false);

    // Perturb points
    // NOTE: The perturbation is how much the robot has moved
    const Vector2T<double> perturbTrans(1, -0.5);
    const double perturbRot = 10 * ANGLE_DEG_TO_RAD;

    PoseTransform2D<double> coordTransform;
    switch (perturbMode) {
        case BOTH:
            coordTransform =
                rotTransToTransform<double>(perturbRot, perturbTrans);
            break;
        case TRANS_ONLY:
            coordTransform = rotTransToTransform<double>(0, perturbTrans);
            break;
        case ROT_ONLY:
            coordTransform = rotTransToTransform<double>(
                perturbRot, Vector2T<double>::Zero());
            break;
    }

    std::cout << "True Perturbaton: " << transformToPose<double>(coordTransform)
              << std::endl;

    // NOTE: the transform thus needs to be inverted because moving forward
    // means that the feature points are shifted backwards
    coordTransform = coordTransform.inverse();

    // Noise constants
    const double transStdDev =
        (perturbMode == PerturbStyle::ROT_ONLY) ? 0 : 0.25;
    const double rotDegStdDev =
        (perturbMode == PerturbStyle::TRANS_ONLY) ? 0 : 2;

    // Perturb the points by the transform
    ORSPVec<double> orspListPerturb;
    orspListPerturb.reserve(orspList.size());
    for (const ORSP<double> &orsp : orspList) {
        ORSP<double> orspPerturb;
        PoseTransform2D<double> coordTransformNoisy = coordTransform;

        // Add noise to the transform
        // addNoiseToTransform(coordTransform, coordTransformNoisy, transStdDev,
        //                     rotDegStdDev);

        convertORSPCoordinates<double>(orsp, orspPerturb, coordTransformNoisy);
        orspListPerturb.push_back(orspPerturb);

        // std::cout << "ORSP" << orsp.toString() << std::endl;
        // std::cout << "Perturbed" << orspPerturb.toString() << std::endl;
    }

    // Velocity Prop
    PoseTransform2D<double> noisyPropTransform = coordTransform.inverse();
    // addNoiseToTransform(noisyPropTransform, noisyPropTransform, transStdDev,
    //                     rotDegStdDev);
    currWorldPose += transformToPose(noisyPropTransform);

    Pose2D<double> currWorldPoseProp(currWorldPose);

    printORSPToFile(orspListPerturb, orspBaseOutputPath, startID + 1, true,
                    poseToTransform(currWorldPose));

    // Solve optimization problem
    bool succ = buildAndSolveRegistrationProblem(orspListPerturb, keyframeList,
                                                 currWorldPose);

    // Output the ORSP points for the perturbed keyframe too
    PoseTransform2D<double> optimTransf =
        poseToTransform(currWorldPose) * poseToTransform(currWorldPoseProp);

    printORSPToFile(orspListPerturb, orspBaseOutputPath, startID + 2, true,
                    optimTransf);

    return 0;
}