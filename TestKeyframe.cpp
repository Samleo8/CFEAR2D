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

// Flag for whether to output ORSP to file for plotting
#define __DEBUG_ORSP__

#ifndef OUT_BUFFER_SIZE
#define OUT_BUFFER_SIZE 5000
#endif

namespace fs = std::filesystem;

static const double defaultFontScale = 10;
static const int newlineSpaceDefault = 50;

static const cv::Scalar black(0, 0, 0);
static const cv::Scalar white(255, 255, 255);

/**
 * @brief Enum output image style
 * @see outputImgFromFrames()
 */
enum OutputStyle { NORMAL, NO_FOOTER, ON_THE_SIDE };

void writeText(cv::Mat aImg, const std::string &aText, const int ax,
               const int ay, const cv::Scalar &aColor = CVColor::blue,
               const double aFontScale = defaultFontScale) {
    // Empty String: Do nothing
    if (aText == "") return;

    // Calculate stuff
    int x = (ax > 0) ? ax : 0;
    int y = (ay > 0) ? ay : 0;

    const int newlineSpace = newlineSpaceDefault * aFontScale;

    std::stringstream ss(aText);
    std::string token;
    while (std::getline(ss, token)) {
        y += newlineSpace;
        cv::putText(aImg, token, cv::Point2d(x, y), cv::FONT_HERSHEY_SIMPLEX,
                    aFontScale, aColor, aFontScale * 2, cv::LINE_8, false);
    }
}

/**
 * @brief Draws point on image at specified coordinate. Actually a small circle
 *
 * @param[in,out] aImg Input/Output image
 * @param[in] aCoord Coordinate of point
 * @param[in] aPointSize Radius of point
 * @param[in] aColor Color of point
 */
void drawPoint(cv::Mat &aImg, const cv::Point2d &aCoord,
               const int aPointSize = 5,
               const cv::Scalar aColor = CVColor::red) {
    cv::circle(aImg, aCoord, aPointSize, aColor, cv::FILLED, cv::LINE_8);
}

/**
 * @brief Draws point on image at specified coordinate. Actually a small circle
 *
 * @param[in,out] aImg Input/Output image
 * @param[in] aCoord Coordinate of point
 */
void drawLine(cv::Mat &aImg, const cv::Point2d &aCoordStart,
              const cv::Point2d &aCoordEnd, const int aLineThickness = 10,
              const cv::Scalar aColor = CVColor::green) {
    cv::arrowedLine(aImg, aCoordStart, aCoordEnd, aColor, aLineThickness,
                    cv::LINE_8);
}

/**
 * @brief Applies header and footer texts to images and concatenates them for
 * easy saving
 *
 * @param[in] displayImages[] Array of images to display
 * @param[in] headerTexts[] Array of header texts to display
 * @param[in] footerTexts[] Array of footer texts to display
 * @param[in] nImgs Number of images in the array
 * @param[out] outputImg Output image
 *
 * @pre len(displayImages) == len(headerTexts) == len(footerTexts) == nImgs
 */
void concatImagesWithText(cv::Mat displayImages[],
                          const std::string headerTexts[],
                          const std::string footerTexts[], const int nImgs,
                          cv::Mat &outputImg, enum OutputStyle outStyle) {
    // Number of endl in output
    const int numberOfEndl = (outStyle == NORMAL) ? 7 : 0;

    const int padding_top = newlineSpaceDefault * defaultFontScale;
    const int padding_bottom =
        (outStyle == NORMAL)
            ? (newlineSpaceDefault * defaultFontScale * (numberOfEndl + 1))
            : 0;
    const int padding_left = 5;
    const int padding_right = 5;

    const int footerTextStartY = padding_top + displayImages[0].rows;

    // Pad images with header and footer text
    for (int i = 0; i < nImgs; i++) {
        cv::copyMakeBorder(displayImages[i], displayImages[i], padding_top,
                           padding_bottom, padding_left, padding_right,
                           cv::BORDER_CONSTANT, white);
        writeText(displayImages[i], headerTexts[i], padding_left + 10, 0);
        if (outStyle == NORMAL)
            writeText(displayImages[i], footerTexts[i], 0, footerTextStartY);
    }

    // Concat images for output
    cv::hconcat(displayImages, nImgs, outputImg);

    // Write text on the side
    if (outStyle == ON_THE_SIDE) {
        const int textSize = 400;
        const int startX = outputImg.cols + 20;
        cv::copyMakeBorder(outputImg, outputImg, 0, 0, 0, textSize,
                           cv::BORDER_CONSTANT, white);

        writeText(outputImg, footerTexts[0], startX, padding_top + 20, black);
    }
}

/**
 * @brief Generate image path from dataset and image numbers
 *
 * @param[in] basePath Base folder path to save image
 * @param[in] dataset Dataset number
 * @param[in] r1ID First image ID
 * @param[out] outPathStr String to path to image
 */
void genImagePath(fs::path &basePath, const unsigned int dataset,
                  const unsigned int r1ID, std::string &outPathStr,
                  const std::string &appendStr = "") {
    fs::path outPath(basePath);
    outPath /= std::to_string(r1ID) + appendStr + ".jpg";
    outPathStr = outPath.string();
}

/**
 * @brief Generate output image given 2 frames
 * @param[in] dataset Dataset number
 * @param[in] r1ID First image ID
 * @param[in] filterSize Filter size
 * @param[out] outputImgFiltered Output image for filtered points
 * @param[out] outputImgORSP Output image for ORSP visualization
 */
void outputImgFromRImg(const RadarImage &aRImg, cv::Mat &outputImgORSP,
                       enum OutputStyle outStyle = NORMAL) {
    // Prepare for drawing
    const cv::Mat outputImgGray = aRImg.getImage(aRImg.RIMG_CART);
    // TODO: Make the background lighter?
    const float BACKGROUND_LIGHTNESS_FACTOR = 3;
    if (BACKGROUND_LIGHTNESS_FACTOR > 1)
        outputImgGray /= BACKGROUND_LIGHTNESS_FACTOR;

    cv::cvtColor(outputImgGray, outputImgORSP, cv::COLOR_GRAY2BGR);

    const cv::Point2d imgCenterPx =
        cv::Point2d(outputImgORSP.cols, outputImgORSP.rows) / 2;

    // Compute ORSP and draw those points with vectors
    const ORSPVec<double> &featurePoints = aRImg.getORSPFeaturePoints();

    // Draw ORSP points
    const double VEC_LEN = 2;
    for (size_t i = 0, sz = featurePoints.size(); i < sz; i++) {
        const ORSP<double> &featPt = featurePoints[i];

        // Calculate center of point and end of normal vector
        Eigen::Vector2d featPtCenter = featPt.center;
        Eigen::Vector2d featPtNormal = featPt.normal;
        Eigen::Vector2d featPtEnd = featPtCenter + featPtNormal * VEC_LEN;

        // Convert to CV
        cv::Point2d pointCVStart(featPtCenter(0), featPtCenter(1));
        cv::Point2d pointCVEnd(featPtEnd(0), featPtEnd(1));

        // Draw filtered point on image
        // NOTE: Point is in meters, but we want to display it in pixels
        //       It is also with reference to the center of the frame, so we
        //       need to re-center it
        pointCVStart /= RANGE_RESOLUTION;
        pointCVEnd /= RANGE_RESOLUTION;

        pointCVStart += imgCenterPx;
        pointCVEnd += imgCenterPx;

        // Draw Corresponding line
        drawLine(outputImgORSP, pointCVStart, pointCVEnd, 8);

        // Draw Point
        drawPoint(outputImgORSP, pointCVStart, 8);
    }
}

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

    // Output the frames to a file
    fs::path poseOutputPath(saveImagesPath);
    poseOutputPath /= "poses";

    fs::create_directories(poseOutputPath);

    poseOutputPath /= "poses_" + std::to_string(startID) + "_" +
                      std::to_string(endID) + ".txt";

    std::ofstream poseOutputFile;
    poseOutputFile.open(poseOutputPath,
                        std::ofstream::out | std::ofstream::trunc);

    // Output ORSP to file for debugging, if flag specified

#ifdef __DEBUG_ORSP__
    fs::path orspBaseOutputPath(saveImagesPath);
    orspBaseOutputPath /= "orsp";

    fs::create_directories(orspBaseOutputPath);

    printORSPToFile(currRImg.getORSPFeaturePoints(), orspBaseOutputPath,
                    startID, false);
#endif

    // Keep finding frames
    while (feed.nextFrame()) {
        size_t currFrameId = feed.getCurrentFrameIndex();
        if (currFrameId == endID) break;

        feed.getCurrentRadarImage(currRImg);

        // K-filtering and ORSP
        currRImg.performKStrong(K, Z_min);
        currRImg.computeOrientedSurfacePoints();

        // Output image
        // cv::Mat outputImgORSP;
        // outputImgFromRImg(currRImg, outputImgORSP);

        // Save world pose for velocity propagation later
        prevWorldPose.copyFrom(currWorldPose);

        // Ceres build and solve problem
        const bool success = buildAndSolveRegistrationProblem(
            currRImg, keyframeList, currWorldPose);

        // TODO: What to do when registration fails?
        if (!success) {
            std::cout << "Registration failed!" << std::endl;
            currWorldPose = prevWorldPose;
            continue;
        }

        // Obtain transform from previous keyframe to current frame
        PoseTransform2D<double> kf2FrameTransf = getTransformsBetweenPoses(
            keyframeList.back().getPose(), currWorldPose);

        // TODO: Add keyframe if necessary
        Pose2D<double> kfDeltaPose = transformToPose<double>(kf2FrameTransf);
        double kfDistSq = kfDeltaPose.position.squaredNorm();
        double kfRot = std::abs(kfDeltaPose.orientation);

        if (kfDistSq >= Keyframe::KF_DIST_THRESH_SQ ||
            kfRot >= Keyframe::KF_ROT_THRESH_RAD) {
            std::cout << "New keyframe added!" << std::endl;

            Keyframe keyframe2(currRImg, currWorldPose);
            keyframeList.push_back(keyframe2);

            poseOutputFile << "kf ";
        }

        // Save pose to file
        poseOutputFile << currWorldPose.toString() << std::endl;

        // Obtain transform from current world pose to previous pose for
        // velocity/seed pose propagation
        PoseTransform2D<double> frame2FrameTransf =
            getTransformsBetweenPoses(prevWorldPose, currWorldPose);

        // NOTE: Stationary checking
        // Move the pose by a constant velocity based on the previous frame
        // But only if movement exceeds a certain threshold
        double deltaDistSq = frame2FrameTransf.translation().squaredNorm();
        if (deltaDistSq <= DIST_STATIONARY_THRESH_SQ) {
            // TODO: Do we revert to previous pose or propagate by 0 velocity?
            currWorldPose = prevWorldPose;

            std::cout << "Stationary. Reverting back to previous pose."
                      << std::endl
                      << std::endl;
        }
        else {
            // TODO: For 3D probably need to make this a transformation matrix operation
            Pose2D deltaPose = transformToPose<double>(frame2FrameTransf);
            currWorldPose += deltaPose;

            std::cout << "Movement with delta: " << deltaPose.toString()
                      << std::endl
                      << std::endl;
        }

#ifdef __DEBUG_ORSP__
        PoseTransform2D<double> worldPoseTransform =
            poseToTransform(currWorldPose);

        printORSPToFile(currRImg.getORSPFeaturePoints(), orspBaseOutputPath,
                        startID, true, worldPoseTransform);
#endif
    }

    // Remember to close file
    poseOutputFile.close();

    return 0;
}