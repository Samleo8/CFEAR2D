#include <filesystem>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>

#include "RadarImage.hpp"

#ifndef OUT_BUFFER_SIZE
#define OUT_BUFFER_SIZE 5000
#endif

namespace fs = std::filesystem;

static const double defaultFontScale = 0.5;
static const int newlineSpaceDefault = 50;

static const cv::Scalar black(0, 0, 0);
static const cv::Scalar white(255, 255, 255);

/**
 * @brief Enum output image style
 * @see outputImgFromFrames()
 */
enum OutputStyle { NORMAL, NO_FOOTER, ON_THE_SIDE };

void writeText(cv::Mat aImg, const std::string &aText, const int ax,
               const int ay, const cv::Scalar &aColor = black,
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
        cv::putText(aImg, token, cv::Point2d(x, y), cv::FONT_HERSHEY_SIMPLEX,
                    aFontScale, aColor, 1);
        y += newlineSpace;
    }
}

/**
 * @brief Draws point on image at specified coordinate. Actually a small circle
 *
 * @param[in,out] aImg Input image
 * @param[in] aCoord Coordinate of point
 */
void drawPoint(cv::Mat &aImg, const cv::Point2d &aCoord) {
    const int pointSize = 4;
    const cv::Scalar color(0, 0, 255);

    cv::circle(aImg, aCoord, pointSize, color, cv::FILLED, cv::LINE_8);
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
        newlineSpaceDefault * defaultFontScale * (numberOfEndl + 1);
    const int padding_left = 5;
    const int padding_right = 5;

    const int footerTextStartY = padding_top + displayImages[0].rows + 20;

    // Pad images with header and footer text
    for (int i = 0; i < nImgs; i++) {
        cv::copyMakeBorder(displayImages[i], displayImages[i], padding_top,
                           padding_bottom, padding_left, padding_right,
                           cv::BORDER_CONSTANT, white);
        writeText(displayImages[i], headerTexts[i], padding_left + 10, 20);
        if (outStyle == NORMAL)
            writeText(displayImages[i], footerTexts[i], 20, footerTextStartY);
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
    outPath /= std::to_string(dataset) + "_" + std::to_string(r1ID) +
               appendStr + ".jpg";
    outPathStr = outPath.string();
}

/**
 * @brief Generate output image given 2 frames
 * @param[in] dataset Dataset number
 * @param[in] r1ID First image ID
 * @param[in] filterSize Filter size
 * @param[out] outputImg Output image
 */
void outputImgFromFrames(const unsigned int dataset, const unsigned int r1ID,
                         cv::Mat &outputImg,
                         enum OutputStyle outStyle = NORMAL) {
    // Obtain radar images
    RadarImage r1(dataset, r1ID, true);

    // TODO: For now, do nothing with K strongest filtering

    // K-filtering
    const size_t K = 10;
    const double Z_min = 55;
    r1.performKStrong(K, Z_min);

    // Get filtered points and display them on image
    FeaturePointsVec featurePoints = r1.getFeaturePoints();

    // Draw feature points
    const cv::Mat outputImgGray = r1.getImage(r1.RIMG_CART);
    cv::cvtColor(outputImgGray, outputImg, cv::COLOR_GRAY2BGR);

    const cv::Point2d imgCenter =
        cv::Point2d(outputImg.cols, outputImg.rows) / 2;

    for (size_t i = 0, sz = featurePoints.size(); i < sz; i++) {
        FeaturePoint point = featurePoints[i];

        cv::Point2d pointCV;
        point.toCV(pointCV);

        // std::cout << "Point " << i << ": (" << point.x << ", " << point.y <<
        // ")";

        // Draw point
        // NOTE: Point is in meters, but we want to display it in pixels
        //       It is also with reference to the center of the frame, so we
        //       need to re-center it

        pointCV /= RANGE_RESOLUTION;
        pointCV += imgCenter;

        // std::cout << "| CV: " << "(" << pointCV.x << ", " << pointCV.y <<
        // ")";
        // std::cout << std::endl;
        
        drawPoint(outputImg, pointCV);
    }
}

/**
 * @brief Main function. Handles input from command line.
 *
 * Computes kstrong filtering and outputs a fancy image for easy saving. Can
 * also allow for auto saving.
 */
int main(int argc, char **argv) {
    if (argc < 3 || argc > 4) {
        printf("Usage: %s <dataset> <image1ID> [0|1:saveDirectToFile]]\n",
               argv[0]);
        return EXIT_FAILURE;
    }

    const unsigned int dataset = atoi(argv[1]);
    const unsigned int r1ID = atoi(argv[2]);
    const bool saveDirectly = (argc == 4 && atoi(argv[3]));

    // Create path to save images
    fs::path saveImagesPath(".");
    saveImagesPath /= "results";

    fs::create_directories(saveImagesPath);

    /**********************************************************************
     * @section TestRadar-KeyframeToKeyframe Compute keyframe to keyframe
     **********************************************************************/
    cv::Mat outputImg;
    outputImgFromFrames(dataset, r1ID, outputImg, NORMAL);

    // Display or save image
    if (saveDirectly) {
        std::string outputImgPathStr;
        genImagePath(saveImagesPath, dataset, r1ID, outputImgPathStr);
        cv::imwrite(outputImgPathStr, outputImg);
    }
    else {
        cv::imshow("Frame " + std::to_string(r1ID), outputImg);

        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return 0;
}