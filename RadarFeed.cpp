/**
 * @file RadarFeed.cpp
 *
 * @brief Radar Image class that performs pre-processing on the images. Will
 * contain all the needed information about the radar image, such as id,
 * pre-processed image cv::Mat etc.
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#include "RadarFeed.hpp"
#include "Pose2D.hpp"
#include <chrono>
#include <filesystem>

namespace fs = std::filesystem;

/***********************************************************
 * @section RadarFeed-ConstInit Constructions and Init
 ***********************************************************/

// Constructors
/**
 * @brief Empty constructor for RadarFeed
 * @note User is required to load in the feed from a folder
 *       using getDataFromFolder()
 * @see RadarFeedHandler.cpp
 */
RadarFeed::RadarFeed() {}

/**
 * @brief Constructor for RadarFeed
 * @param[in] aFolderPath Path to folder where feed (images + ground truth) is
 */
RadarFeed::RadarFeed(const std::filesystem::path &aFolderPath) {
    loadData(aFolderPath);
}

/**
 * @brief Constructor for RadarFeed
 * @param[in] aFolderPath Path to folder where feed (images + ground truth) is
 */
RadarFeed::RadarFeed(const std::string &aFolderPath) {
    loadData(aFolderPath);
}

/**
 * @brief Loads data into feed from folder
 * @param[in] aFolderPath Folder path to load data from
 */
void RadarFeed::loadData(const std::string &aFolderPath) {
    mImagePaths.reserve(EXPECTED_NUM_IMAGES);
    mGroundTruths.reserve(EXPECTED_NUM_IMAGES);

    getDataFromFolder(aFolderPath, mImagePaths, mGroundTruths);
}

/**
 * @brief Loads data into feed from folder
 * @param[in] aFolderPath Folder path to load data from
 */
void RadarFeed::loadData(const std::filesystem::path &aFolderPath) {
    mImagePaths.reserve(EXPECTED_NUM_IMAGES);
    mGroundTruths.reserve(EXPECTED_NUM_IMAGES);

    getDataFromFolder(aFolderPath, mImagePaths, mGroundTruths);
}

/***********************************************************
 * @section RadarFeed-Getters and Setters
 ***********************************************************/

/**
 * @brief Check whether frame number to load is within bounds
 * @param[in] aFrameIdx Frame index to check
 *
 * @return Whether frame index is within bounds
 */
const bool RadarFeed::isWithinBounds(const size_t aFrameIdx) const {
    return aFrameIdx >= 0 && aFrameIdx < mImagePaths.size();
}

/**
 * @brief Get current frame
 * @return Current frame
 */
const size_t RadarFeed::getCurrentFrameIndex() const {
    return mCurrentFrameIdx;
}

/**
 * @brief Set current frame
 * @param[in] aFrameIndex New current frame
 * @param[in] aLoad Flag to specify if frame should be loaded
 * @pre aFrameIndex within bounds, otherwise error
 * @return Success status
 */
const bool RadarFeed::setCurrentFrameIndex(const size_t aFrameIndex,
                                           const bool aLoad) {
    mCurrentFrameIdx = aFrameIndex;

    if (aLoad) {
        return loadFrame(mCurrentFrameIdx);
    }

    return true;
}

/**
 * @brief Get Current Radar Image
 *
 * @return const RadarImage& Current radar image stored in radar feed
 */
const RadarImage &RadarFeed::getCurrentRadarImage() const {
    return mCurrentRImage;
}

/**
 * @brief Get Current Radar Image
 *
 * @param[out] aOutputRadarImage Output radar image
 *
 * @return const RadarImage& Current radar image stored in radar feed
 */
void RadarFeed::getCurrentRadarImage(RadarImage &aOutputRadarImage) const {
    aOutputRadarImage = mCurrentRImage;
}

/***********************************************************
 * @section RadarFeed-Frame handling
 ***********************************************************/

/**
 * @brief Go to the next frame. Updates current frame index and loads next
 * frame.
 * @pre aFrameIndex within bounds, otherwise error
 * @note Equivalent to calling loadFrame(getCurrentFrameIndex() + 1)
 * @return Success status
 */
const bool RadarFeed::nextFrame() {
    return loadFrame(mCurrentFrameIdx + 1);
}

/**
 * @brief Updates current frame index and loads in relevant radar image
 * @param[in] aFrameIndex Frame index to load image from.
 * @pre aFrameIndex within bounds, otherwise error
 * @return Success status
 */
const bool RadarFeed::loadFrame(const size_t aFrameIndex) {
    if (!isWithinBounds(aFrameIndex)) {
        return false;
    }

    // TODO: Possibly load a new image each time
    mCurrentRImage.loadImage(mImagePaths[aFrameIndex]);
    mCurrentRImage.preprocessImage();

    mCurrentFrameIdx = aFrameIndex;

    return true;
}

/***********************************************************
 * @subsection RadarFeed-GroundTruthData Ground Truth Data
 ***********************************************************/

/**
 * @brief Gets the entire ground truth data vector
 * @return Reference to ground truth vector
 */
const std::vector<RotTransData> &RadarFeed::getGroundTruthFeed() const {
    return mGroundTruths;
}

/**
 * @brief Obtain the ground truth of the `aIndex`-th image, in relation to the
 * PREVIOUS image
 * @note getGroundTruth(data, n) will get the ground truth odometry data in the
 * transition from frame n-1 to frame n,
 *
 * @param[out] aGroundTruth Ground truth odometry data we want
 * @param[in] aIndex Index of image
 * @pre 0 < aIndex < capacity
 */
bool RadarFeed::getGroundTruth(RotTransData &aGroundTruth,
                               const size_t aIndex) const {
    if (!aIndex) {
        printf("No ground truth data for frame 0!\n");
        return false;
    }

    if (!isWithinBounds(aIndex)) {
        return false;
    }

    aGroundTruth = mGroundTruths[aIndex - 1];

    return true;
}

/**
 * @brief Obtain the ground truth of the current frame of the image
 * @note Equivalent to getGroundTruth(.., mCurrentFrame)
 */
bool RadarFeed::getGroundTruth(RotTransData &aGroundTruth) const {
    return getGroundTruth(aGroundTruth, mCurrentFrameIdx);
}

/***********************************************************
 * @subsection RadarFeed-SeqFeed Process Sequence/Feed
 ***********************************************************/

/**
 * @brief Start running the feed sequentially, and all the functions related
 * to feed processing, such as keyframing
 * @param[in] aStartFrame Frame to start from
 * @param[in] aEndFrame Frame to end at. Negative value for all the way at
 * the end
 * @param[in] aOutputToFile If desire to output to a file, specify the
 * dataset. -1 otherwise. Defaults to raw_output directory. purposes
 */
void RadarFeed::run(const int aStartFrame, const int aEndFrame,
                    const fs::path &aPoseOutputFilePath, const Pose2D<double> &aInitPose) {
                        // TODO: port from test keyframe

    return;
}
