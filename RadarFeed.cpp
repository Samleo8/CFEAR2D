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
#include <chrono>

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
    getDataFromFolder(aFolderPath, mImagePaths, mGroundTruths);

    // The sequence will hold this much
    // so resize for optimising future speed
    mSequence.reserve(mImagePaths.size());
    mMaxSize = mImagePaths.size();
}

/**
 * @brief Loads data into feed from folder
 * @param[in] aFolderPath Folder path to load data from
 */
void RadarFeed::loadData(const std::filesystem::path &aFolderPath) {
    getDataFromFolder(aFolderPath, mImagePaths, mGroundTruths);

    // The sequence will hold this much
    // so resize for optimising future speed
    mSequence.reserve(mImagePaths.size());
    mMaxSize = mImagePaths.size();
}

/**
 * @brief Initialise the visualiser
 * @param[in] aWidth Starting width of the visualiser
 * @param[in] aHeight Starting height of the visualiser
 * @param[in] aDisplay Flag of whether to do display the mVis window,
 * @param[in] aDisplayTitle Title of display window
 */
void RadarFeed::initVisualiser(const unsigned int aWidth,
                               const unsigned int aHeight, const bool aDisplay,
                               const std::string &aDisplayTitle) {
    mVis = OdometryVisualiser(aWidth, aHeight, aDisplay, aDisplayTitle);
    mVisInitialised = true;
}

/***********************************************************
 * @section RadarFeed-GetterSetter Get/Setters
 ***********************************************************/

/**
 * @brief Helper function that checks whether frame index is valid and within
 * bounds
 * @param[in] aFrameIndex Frame index to check
 * @param[in] aPrintError Flag: print error or not
 * @return Whether frame index within bounds
 */
bool RadarFeed::isWithinBounds(const size_t aFrameIndex,
                               const bool aPrintError) {
    if (aFrameIndex >= mMaxSize) {
        if (aPrintError) {
            printf("Out of bounds: Requested %zu-th image in feed of "
                   "size %zu\n",
                   aFrameIndex, mMaxSize);
        }
        return false;
    }

    return true;
}

/**
 * @brief Retrieves current feed in the form of a vector of RadarImage objects
 * @return Reference to vector of RadarImages stored in getCurrentFeed
 */
const std::vector<RadarImage> &RadarFeed::getCurrentFeed() {
    return mSequence;
}

/***********************************************************
 * @subsection RadarFeed-Frames Frames
 ***********************************************************/

/**
 * @brief Get current frame
 * @return Current frame
 */
size_t RadarFeed::getCurrentFrame() {
    return mCurrentFrame;
}

/**
 * @brief Set current frame
 * @param[in] aFrameIndex New current frame
 * @param[in] aLoad Flag to specify if frame should be loaded
 * @pre aFrameIndex within bounds, otherwise error
 * @return Success status
 */
bool RadarFeed::setCurrentFrame(const size_t aFrameIndex, const bool aLoad) {
    // Check within bounds; note that function settles error printing
    if (!isWithinBounds(aFrameIndex, true)) {
        return false;
    }

    mCurrentFrame = aFrameIndex;

    if (aLoad) {
        return loadFrame(mCurrentFrame, false);
    }

    return true;
}

/**
 * @brief Go to the next frame. Updates current frame index and loads next
 * frame.
 * @note Equivalent to calling loadFrame(getCurrentFrame() + 1, true)
 */
bool RadarFeed::nextFrame() {
    if (!isWithinBounds(mCurrentFrame + 1, false)) {
        printf("Reached end of feed!\n");
        return false;
    }

    mCurrentFrame += 1;
    return loadFrame(mCurrentFrame, false);
}

/**
 * @brief Updates current frame index and loads image into
 * RadarImage sequence from input file paths.
 * @param[in] aFrameIndex Frame index to load image from.
 * @param[in] aSetCurrentFrame Flag of whether or not to set current frame index
 */
bool RadarFeed::loadFrame(const size_t aFrameIndex,
                          const bool aSetCurrentFrame) {
    if (!isWithinBounds(aFrameIndex, true)) {
        return false;
    }

    // Don't reload images that have already been loaded
    RadarImage rImage;
    if (!getRadarImage(rImage, aFrameIndex, false)) {
        rImage.loadImage(mImagePaths[aFrameIndex]);
        rImage.preprocessImages();

        addRadarImage(rImage, aFrameIndex);
    }

    if (aSetCurrentFrame) mCurrentFrame = aFrameIndex;

    return true;
}

/**
 * @brief Go to frame. Exactly equivalent (alias) to loadFrame
 */
bool RadarFeed::gotoFrame(const size_t aFrameIndex,
                          const bool aSetCurrentFrame) {
    return loadFrame(aFrameIndex, aSetCurrentFrame);
}

/***********************************************************
 * @subsection RadarFeed-RadarImages RadarImages
 ***********************************************************/

/**
 * @brief Gets radar image at current frame position.
 *
 * Equivalent to getRadarImage(aRImage, mCurrentFrame)
 *
 * @param[out] aRImage RadarImage we want
 * @return Success status
 */
bool RadarFeed::getCurrentRadarImage(RadarImage &aRImage) {
    return getRadarImage(aRImage, mCurrentFrame);
}

/**
 * @brief Retrieves the aFrameIndex-th RadarImage from the sequence
 *
 * @param[out] aRImage RadarImage we want
 * @param[in] aFrameIndex Frame index of the RadarImage in the sequence
 * @param[in] aPrintErrors Flag specifying whether errors should be printed
 *
 * @pre aFrameIndex < len(mSequence), return false otherwise
 * @return Success status
 */
bool RadarFeed::getRadarImage(RadarImage &aRImage, const size_t aFrameIndex,
                              const bool aPrintErrors) {
    // Check within bounds; note that function settles error printing
    if (!isWithinBounds(aFrameIndex, aPrintErrors)) {
        return false;
    }

    // Check if accessible
    size_t seqSize = mSequence.size();
    if (aFrameIndex >= seqSize) {
        if (aPrintErrors)
            printf("No radar image loaded into frame %zu yet.", aFrameIndex);
        return false;
    }

    aRImage = mSequence[aFrameIndex];

    return true;
}

/**
 * @brief Add to back of feed (next frame). Equivalent to addToFeed without
 * specifying frame index
 * @param[in] aRImage Radar image to push to feed
 * @param[in] aOverwrite Flag specifying whether existing aRImage should be
 * overwritten
 */
void RadarFeed::pushRadarImage(const RadarImage &aRImage,
                               const bool aOverwrite) {
    addRadarImage(aRImage, aOverwrite);
}

/**
 * @brief Add to back of feed (next frame). Equivalent to pushToFeed
 * @param[in] aRImage Radar image to push to feed
 * @param[in] aOverwrite Flag specifying whether existing aRImage should be
 * overwritten
 */
void RadarFeed::addRadarImage(const RadarImage &aRImage,
                              const bool aOverwrite) {
    addRadarImage(aRImage, mCurrentFrame + 1, aOverwrite);
}

/**
 * @brief Add to feed at specific frame index. If already initialised, do
 * nothing unless overwrite specified. Handles necessary resizing of array if
 * necessary.
 * @param[in] aRImage Radar image to push to feed
 * @param[in] aFrameIndex Frame index to add radar image at
 * @param[in] aOverwrite Flag specifying whether existing aRImage should be
 * overwritten
 *
 * @pre aFrameIndex within bounds
 */
void RadarFeed::addRadarImage(const RadarImage &aRImage,
                              const size_t aFrameIndex, const bool aOverwrite) {
    // Ensure within bounds, otherwise print error and return
    if (!isWithinBounds(aFrameIndex, true)) {
        return;
    }

    // Check if need to extend capacity
    size_t seqSize = mSequence.size();
    if (aFrameIndex >= seqSize) {
        mSequence.resize(aFrameIndex + 1);
    }

    // Overwrite only if required
    if (!mSequence[aFrameIndex].isLoaded() || aOverwrite)
        mSequence[aFrameIndex] = aRImage;

    return;
}

/***********************************************************
 * @subsection RadarFeed-GroundTruthData Ground Truth Data
 ***********************************************************/

/**
 * @brief Negates every value in the RotTransData struct
 *
 * @param[in,out] aData Data to negate
 */
void RadarFeed::negateData(RotTransData &aData) {
    aData.dx = -aData.dx;
    aData.dy = -aData.dy;
    aData.dRotRad = -aData.dRotRad;
}

/**
 * @brief Gets the entire ground truth data vector
 * @return Reference to ground truth vector
 */
const std::vector<RotTransData> &RadarFeed::getGroundTruthFeed() {
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
                               const size_t aIndex) {
    if (!aIndex) {
        printf("No ground truth data for frame 0!\n");
        return false;
    }

    if (!isWithinBounds(aIndex, true)) {
        return false;
    }

    aGroundTruth = mGroundTruths[aIndex - 1];

    return true;
}

/**
 * @brief Obtain the ground truth of the current frame of the image
 * @note Equivalent to getGroundTruth(.., mCurrentFrame)
 */
bool RadarFeed::getGroundTruth(RotTransData &aGroundTruth) {
    return getGroundTruth(aGroundTruth, mCurrentFrame);
}

/**
 * @brief Get reference to visualiser
 * @param[out] aVis Visualiser object
 * @return False if visualiser not yet initialised; true if success
 */
bool RadarFeed::getVisualiser(OdometryVisualiser &aVis) {
    if (!mVisInitialised) return false;

    aVis = mVis;
    return true;
}

/**
 * @brief Get filter size
 * @return RadarFeed filter size
 */
int RadarFeed::getFilterSize() {
    return mFilterSize;
}

/**
 * @brief Set filter size
 * @param[in] aFilterSize Filter size to set
 */
void RadarFeed::setFilterSize(const int aFilterSize) {
    mFilterSize = aFilterSize;
}

/***********************************************************
 * @subsection RadarFeed-SeqFeed Process Sequence/Feed
 ***********************************************************/

/**
 * @brief Print instructions for start function
 * @see start
 *
 * @param[in] aSaveImagesPath Path to save images at
 * @param[in] aDisplay Flag of whether to display on visualiser
 * @param[in] aDisplayTime Time in ms on how long to display the message
 */
void RadarFeed::printInstructions(const std::string &aSaveImagesPath,
                                  const bool aDisplay, const int aDisplayTime) {
    const std::string instructionString =
        "===============INSTRUCTIONS===============\n"
        "[Legend]\n"
        "Green: Ground truth\n"
        "Orange: Predicted (frame to frame)\n"
        "Blue: Predicted (keyframes) \n\n"
        "[Keyboard Shortcuts]\n"
        "Q/Esc: Quit\n"
        "P: Toggle Pause/Play\n"
        "Space/Enter: Next frame\n"
        "S: Save current image into " +
        aSaveImagesPath + "\n";

    std::cout << instructionString << "\n";

    if (aDisplay && mVisInitialised) {
        mVis.display();

        if (!mVis.displayMessage(instructionString, aDisplayTime)) {
            printf("No QT support: Unable to display instructions text as "
                   "overlay!\n\n");

            mVis.writeText(instructionString, 10, 10, mVis.Color.black, 0.5);
            mVis.display();
            cv::waitKey(0);
            mVis.clear();
        }
    }
}

/**
 * @brief Updates rotation matrices and translation vectors according to the
 * rotation and translation data. Used for ground truth as well.
 *
 * @param[in] aPredData Rotation and translation data
 * @param[out] aRotMat Running rotation matrix
 * @param[out] aTransVec Translation vector
 */
void RadarFeed::updateRotMatTransVec(const RotTransData &aPredData,
                                     Eigen::Rotation2D<double> &aRotMat,
                                     Eigen::Vector2d &aTransVec) {
    aRotMat *= Eigen::Rotation2D<double>(aPredData.dRotRad);
    aTransVec = aRotMat * Eigen::Vector2d(aPredData.dx, aPredData.dy);
    return;
}

/**
 * @brief Start running the feed sequentially, and all the functions related
 * to feed processing, such as keyframing
 * @param[in] aStartFrame Frame to start from
 * @param[in] aEndFrame Frame to end at. Negative value for all the way at
 * the end
 * @param[in] aVisualise Flag on whether to start visualiser for
 * visualisation
 * @param[in] aVisualiserTitle Title to display on the visualiser window
 * @param[in] aOutputToFile If desire to output to a file, specify the
 * dataset. -1 otherwise. Defaults to raw_output directory. purposes
 */
void RadarFeed::run(const int aStartFrame, const int aEndFrame,
                    const bool aVisualise, std::string aVisualiserTitle,
                    const int aOutputToFile) {
    // Titles
    const std::string &radarImageTitle =
        "Radar: Course Cartesian" +
        ((aOutputToFile >= 0) ? " (" + std::to_string(aOutputToFile) + ")"
                              : "");
    const std::string &visualiserTitle = aVisualiserTitle;

    // Default image saving location
    fs::path saveImagesPath(".");
    saveImagesPath /= "vis_saved_images";

    // Default output saving location
    fs::path outputTextFilePath(".");
    outputTextFilePath /= "raw_output";
    outputTextFilePath /=
        std::to_string(aOutputToFile) + "_" + std::to_string(aStartFrame) +
        "_" + ((aEndFrame < 0) ? "end" : std::to_string(aEndFrame)) + ".txt";

    // Radar Images
    RadarImage prevImage, currImage;

    // Visualiser setup
    if (aVisualise) {
        const unsigned int initialWidth = 500;
        const unsigned int initialHeight = 500;

        if (!mVisInitialised) {
            initVisualiser(initialWidth, initialHeight, true, aVisualiserTitle);
        }

        mVis.clear();
    }

    // Points for drawing
    const unsigned int startX = 400;
    const unsigned int startY = 400;

    // GT Pred point
    cv::Point2d currGTPoint = cv::Point2d(startX, startY);
    cv::Point2d &prevGTPoint = currGTPoint;
    const cv::Scalar &gtColor = mVis.getPointColor(mVis.GROUND_TRUTH);

    cv::Point2d currPredPoint = cv::Point2d(startX, startY);
    cv::Point2d &prevPredPoint = currPredPoint;
    const cv::Scalar &predColor = mVis.getPointColor(mVis.ODOMETRY_PREDICTION);

    // Keyframes
    cv::Point2d currKeyframePoint = cv::Point2d(startX, startY);
    cv::Point2d &prevKeyframePoint = currKeyframePoint;
    const cv::Scalar &predKeyframeColor =
        mVis.getPointColor(mVis.ODOMETRY_PREDICTION_KEYFRAMES);

    printInstructions(saveImagesPath.string(), aVisualise);

    if (aVisualise) {
        mVis.drawPoint(currGTPoint, gtColor);
        mVis.drawPoint(currPredPoint, predColor);
        mVis.drawPoint(currKeyframePoint, predKeyframeColor);
    }

    // Initialise frames
    bool success;
    success = setCurrentFrame(aStartFrame, true);
    success = getCurrentRadarImage(prevImage);

    if (success) {
        if (aVisualise) {
            prevImage.displayImage(RadarImage::coarseCart, radarImageTitle,
                                   false, false);
        }
    }
    else {
        printf("Failed to load image!\n");
        return;
    }

    // Rotation matrices and translation vectors
    Eigen::Rotation2D<double> rotMatGT(0);
    Eigen::Rotation2D<double> rotMatPred(0);
    Eigen::Rotation2D<double> rotMatPredKeyframes(0);
    Eigen::Vector2d transVecGT, transVecPred, transVecPredKeyframes;

    // Output to text file for error checking
    if (aOutputToFile >= 0) {
        mOutputTextFile = fopen(outputTextFilePath.string().c_str(), "w+");
    }
    else {
        mOutputTextFile = stdout;
    }

    // Pose graph (with keyframe candidates embedded)
    // NOTE: In this case to save computation times, we save the struct of
    // RotTransData computed relative to the previous keyframe
    bool foundKeyframe = false;
    const int slidingWindowSize = DEFAULT_SLIDING_WINDOW_SIZE;

    // Ceres Problem Options
    // Needed to enable fast removal
    ceres::Problem::Options ceresProblemOptions;
    ceresProblemOptions.enable_fast_removal = true;

    // Initialise pose graph with origin node
    PoseGraphNodeData originNodePose(prevKeyframePoint, 0);
    KeyframeNode originNode(prevImage, originNodePose);
    PoseGraph poseGraph(originNode, slidingWindowSize, mFilterSize,
                        ceresProblemOptions);
    poseGraph.setOutputTextFile(mOutputTextFile);

    // Timing
    // double f2f_time = 0, kf_time = 0;

    // Start loop
    bool paused = aVisualise;
    while (nextFrame()) {
        size_t currFrame = getCurrentFrame();

        getCurrentRadarImage(currImage);

        // First begin calculating frame by frame odometry
        RotTransData predData;

        // auto start_f2f = std::chrono::high_resolution_clock::now();
        double confidencePred =
            poseGraph.calculateOdometry(prevImage, currImage, predData);
        updateRotMatTransVec(predData, rotMatPred, transVecPred);
        // auto end_f2f = std::chrono::high_resolution_clock::now();
        // f2f_time += std::chrono::duration_cast<std::chrono::milliseconds>(
        //                 end_f2f - start_f2f)
        //                 .count();

        fprintf(mOutputTextFile, "%04zu: %lf[rad] %lf[x] %lf[y] | Conf: %lf\n",
                currFrame, predData.dRotRad, predData.dx, predData.dy,
                confidencePred);

        prevImage = currImage;

        // Parse Keyframes

        // auto start_kf = std::chrono::high_resolution_clock::now();
        foundKeyframe = poseGraph.findKeyframe(currImage);
        // auto end_kf = std::chrono::high_resolution_clock::now();
        // kf_time += std::chrono::duration_cast<std::chrono::milliseconds>(
        //                end_kf - start_kf)
        //                .count();

        if (foundKeyframe) {
            // printf("Time taken for frame to frame processings since last "
            //        "keyframe: %.2lfms\n",
            //        f2f_time);
            // printf("Time taken for keyframe processings since last "
            //        "keyframe: %.2lfms\n",
            //        kf_time);
            // f2f_time = 0;
            // kf_time = 0;

            // TODO: Print or calculate or sth
            // updateRotMatTransVec(newKeyframeData.data_wrt_keyframe,
            //                      rotMatPredKeyframes, transVecPredKeyframes);

            // fprintf(mOutputTextFile, "%lf[rad] %lf[x] %lf[y] | Conf: %lf\n",
            //         newKeyframeData.data_wrt_keyframe.dRotRad,
            //         newKeyframeData.data_wrt_keyframe.dx,
            //         newKeyframeData.data_wrt_keyframe.dy,
            //         newKeyframeData.confidenceKeyframe);
        }

        // Obtain ground truth data
        RotTransData gt;
        if (!getGroundTruth(gt)) {
            printf_err("Ground truth not found!\n");
            printf("Ground truth not found! Please download it!\n");
            break;
        }

        // negateData(gt);
        double confidenceGT = computeConfidenceLevel(gt);
        updateRotMatTransVec(gt, rotMatGT, transVecGT);

        fprintf(mOutputTextFile, "[GT]: %lf[rad] %lf[x] %lf[y] | Conf: %lf\n\n",
                gt.dRotRad, gt.dx, gt.dy, confidenceGT);

        if (aVisualise) {
            // Ground truth
            // NOTE: Negating for visualiser purposes
            currGTPoint =
                prevGTPoint + cv::Point2d(transVecGT[0], transVecGT[1]);

            // Predicted (frame-to-frame)
            currPredPoint =
                prevPredPoint + cv::Point2d(transVecPred[0], transVecPred[1]);

            // Predicted (keyframe)
            mVis.drawPoint(currGTPoint, gtColor);
            mVis.drawPoint(currPredPoint, predColor);

            // Draw all the locked in nodes
            if (foundKeyframe) {
                const PoseGraphNodes &lockedInNodes =
                    poseGraph.getLockedInNodes();
                for (PoseGraphNodes::const_iterator it = lockedInNodes.begin(),
                                                    end = lockedInNodes.end();
                     it != end; ++it) {
                    currKeyframePoint.x = (*it).x;
                    currKeyframePoint.y = (*it).y;
                    mVis.drawPose(currKeyframePoint, (*it).yaw,
                                  predKeyframeColor);
                }
            }

            prevGTPoint = currGTPoint;
            prevPredPoint = currPredPoint;
            if (foundKeyframe) prevKeyframePoint = currKeyframePoint;

            // Draw on canvas
            mVis.display();

            // Display radar image
            currImage.displayImage(RadarImage::coarseCart, radarImageTitle,
                                   false, false);
        }

        if (currFrame == aEndFrame) {
            if (aOutputToFile >= 0) fclose(mOutputTextFile);
            return;
        }

        // GUI: Keypresses
        unsigned char k;
        unsigned char keys[] = { 'q', 'p', mVis.KEYCODE_SPACE, mVis.KEYCODE_ESC,
                                 's' };

        if (paused) {
            k = mVis.waitForKeypress(keys, sizeof(keys));
        }
        else {
            k = mVis.delay(1);
        }

        switch (k) {
            case 'q':
            case OdometryVisualiser::KEYCODE_ESC:
                if (aOutputToFile >= 0) fclose(mOutputTextFile);
                return;
            case 'p':
                paused ^= true;
                break;
            // Save
            case 's': {
                // Create directory if not exist
                fs::create_directories(saveImagesPath);

                // Pad with zeros
                char buf[5];
                snprintf(buf, 5, "%04zu", currFrame);
                std::string currFramePadded = buf;

                // Save visualisation canvas
                fs::path saveVisPath =
                    saveImagesPath / ("vis_" + currFramePadded + ".jpg");
                cv::imwrite(saveVisPath.string(), mVis.getCanvas());

                // Save course cart image
                fs::path saveRadarPath =
                    saveImagesPath / ("radar_" + currFramePadded + ".jpg");
                cv::imwrite(saveRadarPath.string(),
                            currImage.getImageCoarseCart());

                printf("Visualisation canvas saved to %s\n",
                       saveVisPath.string().c_str());
                printf("Radar image saved to %s\n",
                       saveRadarPath.string().c_str());
                break;
            }
            default:
                break;
        }
    }

    if (aOutputToFile >= 0) {
        fclose(mOutputTextFile);
    }

    return;
}
