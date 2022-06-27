/**
 * @file RadarFeed.hpp
 *
 * @brief RadarFeed class that holds a feed in the form of a vector of
 * RadarImages. In particular, the @see @a run() function performs radar
 * odometry on a stream/feed of images, as inputted in the @see @a
 * getDataFromFolder() function, and visualises it on the @see
 * OdometryVisualiser
 *
 * Pose graph optimisation is also run in conjunction with the feed.
 *
 * @see RadarImage
 * @see RadarFeedHandler.cpp
 * @see PoseGraph
 * @see OdometryVisualiser
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#ifndef __RADAR_FEED_H__
#define __RADAR_FEED_H__

#include <Eigen/Geometry>
#include <opencv2/opencv.hpp>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "OdometryVisualiser.hpp" // needed for visualiser
// #include "PoseGraph.hpp"          // needed for pose graph optimisation
#include "RadarFeedHandler.hpp"   // needed to handle file path and data

/**
 * @brief RadarFeed class that holds a feed in the form of a vector of
 * RadarImages. In particular, the @see @a run() function performs radar
 * odometry on a stream/feed of images, as inputted in the @see @a
 * getDataFromFolder() function, and visualises it on the @see
 * OdometryVisualiser
 *
 * Pose graph optimisation is also run in conjunction with the feed.
 *
 * @see RadarFeed::run()
 *
 * @see RadarImage
 * @see PoseGraph
 * @see OdometryVisualiser
 */
class RadarFeed {
  private:
    /** @brief Sequence of RadarImages */
    std::vector<RadarImage> mSequence;

    /**
     * @brief Vector of paths to images.
     * Read from getDataFromFolder() in RadarFeedHandler.cpp in sequential order
     * (timestamp wise)
     */
    std::vector<std::string> mImagePaths;

    /**
     * @brief Vector of ground truth data.
     * Read from getDataFromFolder() in RadarFeedHandler.cpp in sequential order
     * (timestamp wise)
     * @see RotTransData struct
     */
    std::vector<RotTransData> mGroundTruths;

    /** @brief Current frame */
    size_t mCurrentFrame = 0;

    /** @brief Max size */
    size_t mMaxSize = 0;

    /**
     * @brief Filter size
     * @note Default from RadarImage, but changable
     */
    int mFilterSize = DEFAULT_FILTER_SIZE;

    /**
     * @brief Output text file for error
     */
    FILE *mOutputTextFile = stdout;

    // Visualiser stuff
    /**
     * @brief Visualiser
     * @see OdometryVisualiser
     */
    OdometryVisualiser mVis;

    /**
     * @brief Flag of whether visualiser initialised
     * @see initVisualiser()
     */
    bool mVisInitialised = false;

    void printInstructions(const std::string &aSaveImagesPath,
                           const bool aDisplay = true,
                           const int aDisplayTime = 3000);

  public:
    // Constructs & Inits
    RadarFeed();

    RadarFeed(const std::string &aFolderPath);
    RadarFeed(const std::filesystem::path &aFolderPath);

    void loadData(const std::filesystem::path &aFolderPath);
    void loadData(const std::string &aFolderPath);

    void
    initVisualiser(const unsigned int aWidth = 500,
                   const unsigned int aHeight = 500, bool aDisplay = false,
                   const std::string &aDisplayTitle = "Odometry Visualiser");

    // Getters/Setters
    bool isWithinBounds(const size_t aFrameIndex,
                        const bool aPrintError = false) const;
    const std::vector<RadarImage> &getCurrentFeed() const;
    const std::vector<RotTransData> &getGroundTruthFeed() const;
    bool getVisualiser(OdometryVisualiser &aVis) const;

    // Frames
    size_t getCurrentFrame() const;
    bool setCurrentFrame(const size_t aFrameIndex, const bool aLoad = false);
    bool loadFrame(const size_t aFrameIndex,
                   const bool aSetCurrentFrame = true);
    bool gotoFrame(const size_t aFrameIndex,
                   const bool aSetCurrentFrame = true);
    bool nextFrame();

    // Radar Images
    bool getRadarImage(RadarImage &aRImage, const size_t aFrameIndex,
                       bool aPrintErrors = false) const;
    bool getCurrentRadarImage(RadarImage &aRImage) const;

    void pushRadarImage(const RadarImage &aRImage,
                        const bool aOverwrite = false);
    void addRadarImage(const RadarImage &aRImage,
                       const bool aOverwrite = false);
    void addRadarImage(const RadarImage &aRImage, const size_t aFrameIndex,
                       bool aOverwrite = false);

    // Ground truth and other data
    bool getGroundTruth(RotTransData &aGroundTruth) const;
    bool getGroundTruth(RotTransData &aGroundTruth, const size_t aIndex) const;

    // Sequential Processing or Feed
    void updateRotMatTransVec(const RotTransData &aPredData,
                              Eigen::Rotation2D<double> &aRotMat,
                              Eigen::Vector2d &aTransVec) const;

    void run(int aStartFrame = 0, int aEndFrame = -1,
             const bool aVisualise = true,
             std::string aVisualiserTitle = "Odometry Visualiser",
             const int aOutputToFile = -1);
};

#endif