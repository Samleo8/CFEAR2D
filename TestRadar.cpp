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
 * @brief Get radar image data from 2 radar images. Also returns the rotated r2
 * image.
 *
 * @param[in] r1 Radar image 1
 * @param[in] r2 Radar image 2
 * @param[out] r2Rot Rotated 2nd image based on image data
 * @param[out] data Rotation translation data
 * @param[in] filterSize Filter size
 * @param[in] displayHighpass Flag whether or not to display highpass image
 */
void getRadarImageData(RadarImage &r1, RadarImage &r2, cv::Mat &r2Rot,
                       RotTransData &data,
                       const double filterSize = DEFAULT_FILTER_SIZE,
                       const bool displayHighpass = false) {
    double rotDifference = r1.getRotationDifference(r2, filterSize);

    cv::Point2d translation;
    r1.getTranslationDifference(r2, translation, rotDifference, filterSize,
                                displayHighpass);

    performImageRotation(r1.getImageCoarseCart(), r2Rot, rotDifference);

    data.dRotRad = rotDifference;
    data.dx = translation.x;
    data.dy = translation.y;
}

/**
 * @brief Obtains text from data
 * @param[in] data Rotation translation data
 * @param[out] outputStr Output string
 */
void getTextFromData(const RotTransData &data, std::string &outputStr) {
    const double transX = data.dx;
    const double transY = data.dy;
    const double rotDifference = data.dRotRad;
    const double confidence = computeConfidenceLevel(data);

    char output[OUT_BUFFER_SIZE];
    int len = 0;
    len += snprintf(output + len, OUT_BUFFER_SIZE - len, "Rot (rad): %lf\n",
                    rotDifference);
    len += snprintf(output + len, OUT_BUFFER_SIZE - len, "Rot (deg): %lf\n\n",
                    RAD_TO_DEG * rotDifference);

    len += snprintf(output + len, OUT_BUFFER_SIZE - len,
                    "Translation (pixels): %lf %lf\n",
                    transX / RANGE_RESOLUTION, transY / RANGE_RESOLUTION);
    len += snprintf(output + len, OUT_BUFFER_SIZE - len,
                    "Translation (m): %lf %lf\n\n", transX, transY);

    len += snprintf(output + len, OUT_BUFFER_SIZE - len, "Confidence: %lf\n",
                    confidence);

    printf("%s", output);
    fflush(stdout);

    outputStr = std::string(output);
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
 * @param[in] r2ID Second image ID
 * @param[out] outPathStr String to path to image
 */
void genImagePath(fs::path &basePath, const unsigned int dataset,
                  const unsigned int r1ID, const unsigned int r2ID,
                  std::string &outPathStr, const std::string &appendStr = "") {
    fs::path outPath(basePath);
    outPath /= std::to_string(dataset) + "_" + std::to_string(r1ID) + "_" +
               std::to_string(r2ID) + appendStr + ".jpg";
    outPathStr = outPath.string();
}

/**
 * @brief Generate output image given 2 frames
 * @param[in] dataset Dataset number
 * @param[in] r1ID First image ID
 * @param[in] r2ID Second image ID
 * @param[in] filterSize Filter size
 * @param[out] outputImg Output image
 */
void outputImgFromFrames(const unsigned int dataset, const unsigned int r1ID,
                         const unsigned int r2ID, const double filterSize,
                         cv::Mat &outputImg,
                         enum OutputStyle outStyle = NORMAL) {
    // Init radar images
    RadarImage r1(dataset, r1ID, true);
    RadarImage r2(dataset, r2ID, true);
    RotTransData data;
    cv::Mat r2Rot;
    std::string outputStr;

    // r1.displayImage(r1.coarseCart, std::to_string(r1ID));
    // r2.displayImage(r2.coarseCart, std::to_string(r2ID));

    getRadarImageData(r1, r2, r2Rot, data, filterSize);

    getTextFromData(data, outputStr);

    // Fancy display that's easier to save
    cv::Mat r1Img, r2Img;
    r1Img = r1.getImage(RadarImage::coarseCart);
    r2Img = r2.getImage(RadarImage::coarseCart);

    // Images to pad
    cv::Mat displayImages[] = { r1Img, r2Rot, r2Img };
    const int nImgs = sizeof(displayImages) / sizeof(displayImages[0]);

    std::string headerTexts[nImgs] = { "Frame " + std::to_string(r1ID),
                                       "Frame " + std::to_string(r1ID) +
                                           " (Rot)",
                                       "Frame " + std::to_string(r2ID) };

    std::string footerTexts[nImgs] = { outputStr, "", "" };

    // Concat images for display
    concatImagesWithText(displayImages, headerTexts, footerTexts, nImgs,
                         outputImg, outStyle);
}

/**
 * @brief Main function. Handles input from command line.
 *
 * Computes phase corr for data, and outputs a fancy image for easy saving. Can
 * also allow for auto saving.
 */
int main(int argc, char **argv) {
    if (argc < 4 || argc > 6) {
        printf("Usage: %s <dataset> <image1ID> <image2ID> [filter "
               "size [0|1:saveDirectToFile]]\n",
               argv[0]);
        return EXIT_FAILURE;
    }

    const unsigned int dataset = atoi(argv[1]);
    const unsigned int r1ID = atoi(argv[2]), r2ID = atoi(argv[3]);

    const double filterSize =
        ((argc == 5) ? atof(argv[4]) : DEFAULT_FILTER_SIZE);
    const bool saveDirectly = (argc == 6 && atoi(argv[5]));

    // Create path to save images
    fs::path saveImagesPath(".");
    saveImagesPath /= "raw_output";
    saveImagesPath /= "error_images";

    fs::create_directories(saveImagesPath);

    /**********************************************************************
     * @section TestRadar-KeyframeToKeyframe Compute keyframe to keyframe
     **********************************************************************/
    cv::Mat outputImg;
    outputImgFromFrames(dataset, r1ID, r2ID, filterSize, outputImg, NORMAL);

    // Display or save image
    if (saveDirectly) {
        std::string outputImgPathStr;
        genImagePath(saveImagesPath, dataset, r1ID, r2ID, outputImgPathStr,
                     "_keyframes");
        cv::imwrite(outputImgPathStr, outputImg);
    }
    else {
        cv::imshow("Frame " + std::to_string(r1ID) + " against " +
                       std::to_string(r2ID),
                   outputImg);

        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    /**********************************************************************
     * @section TestRadar-FrameToFrame Compute frame to frame range
     **********************************************************************/
    cv::Mat outputImg2;
    int start = MIN(r1ID, r2ID);
    int end = MAX(r1ID, r2ID);

    // Nothing to do
    if (start + 1 == end || !saveDirectly) return 0;

    for (int i = start; i < end; i++) {
        cv::Mat outputImgTemp;
        outputImgFromFrames(dataset, i, i + 1, filterSize, outputImgTemp,
                            ON_THE_SIDE);
        if (i == start)
            outputImg2 = outputImgTemp;
        else
            cv::vconcat(outputImg2, outputImgTemp, outputImg2);
    }

    // Display or save image
    if (saveDirectly) {
        std::string outputImgPathStr;
        genImagePath(saveImagesPath, dataset, r1ID, r2ID, outputImgPathStr,
                     "_allframes");
        cv::imwrite(outputImgPathStr, outputImg2);
    }
    else {
        cv::imshow("All frames from " + std::to_string(r1ID) + " to " +
                       std::to_string(r2ID),
                   outputImg2);

        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    return 0;
}