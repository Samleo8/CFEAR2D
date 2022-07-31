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
#include <cstddef>
#include <filesystem>
#include <opencv2/opencv.hpp>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "Keyframe.hpp"
#include "MotionUndistort.hpp"
#include "OptimisationHandler.hpp"
#include "PoseTransformHandler.hpp"
#include "RadarFeedHandler.hpp" // needed to handle file path and data
#include "RadarImage.hpp"
#include "RadarImageHandler.hpp"
#include "TransformDefines.hpp"

namespace fs = std::filesystem;

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
    // CONSTANTS (FLAGS)
    /** @brief Whether to perform motion undistortion */
    static constexpr bool DO_MOTION_UNDISTORTION = true;

    /** @brief Whether to simulate IMU via by propagating rot GT */
    static constexpr bool DO_SIMULATED_IMU = true;

    // CONSTANTS FOR SEQUENTIAL PROCESSING

    /** @brief Radar scan frequency in Hz. Used for motion undistortion */
    static constexpr double RADAR_SCAN_FREQ = 4;

    /** @brief Radar scan period in seconds. Used for motion undistortion */
    static constexpr double RADAR_SCAN_PERIOD = 1.0 / RADAR_SCAN_FREQ;

    /** @brief Number of azimuths in a radar scan. Used for motion
     * undistortion */
    static constexpr int RADAR_NAZIMUTHS = RADAR_IMAGE_POLAR_N_AZIMUTHS_PX;

    /** @brief Time vector used for motion distortion */
    const VectorDimd<RADAR_NAZIMUTHS> mMotionTimeVector =
        generateTimeVector<RADAR_NAZIMUTHS>(RADAR_SCAN_PERIOD);

    /** @brief Expected number of images in feed path, used for reserving
     * vector space */
    static constexpr size_t EXPECTED_NUM_IMAGES = 8000;

    // CONSTANTS FOR FILTERING / OPTIMISATION HANDLING

    /** @brief Filtering: number of points */
    static constexpr size_t K = 12;

    /** @brief Filtering: power min threshold */
    static constexpr double Z_MIN = 55;

    /** @brief Filtering: algorithm */
    static constexpr RadarImage::FilteringAlgorithm
        DEFAULT_FILTERING_ALGORITHM =
            RadarImage::FilteringAlgorithm::K_STRONGEST;

    /** @brief Keyframe buffer size */
    static constexpr int KF_BUFF_SIZE = 3;

    // CONSTANTS FOR STAITONARY CHECKS

    /** @brief Whether to perform stationary check on keyframes
     */
    static constexpr bool PERFORM_STATIONARY_CHECK = true;

    /**
     * @brief Rotation threshold for statiionary checking, in radians
     */
    static constexpr double ROT_STATIONARY_THRESH_RAD = 1.5 * ANGLE_DEG_TO_RAD;

    /**
     * @brief Distance threshold for stationary checking, in metres
     */
    static constexpr double DIST_STATIONARY_THRESH = 0.05; // 5cm

    /** @brief Square of @see DIST_STATIONARY_THRESH for speed */
    static constexpr double DIST_STATIONARY_THRESH_SQ =
        DIST_STATIONARY_THRESH * DIST_STATIONARY_THRESH;

    // MEMBER VARIABLES

    /** @brief Current RadarImage in feed */
    RadarImage mCurrentRImage;

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
    size_t mCurrentFrameIdx = 0;

  public:
    // Constructs & Inits
    RadarFeed();

    RadarFeed(const std::string &aFolderPath);
    RadarFeed(const std::filesystem::path &aFolderPath);

    void loadData(const std::filesystem::path &aFolderPath);
    void loadData(const std::string &aFolderPath);

    // Getters/Setters
    const std::vector<RadarImage> &getCurrentFeed() const;
    const std::vector<RotTransData> &getGroundTruthFeed() const;

    const RadarImage &getCurrentRadarImage() const;
    void getCurrentRadarImage(RadarImage &aOutputRadarImage) const;

    // Frames
    const bool isWithinBounds(const size_t aFrameIdx) const;
    const size_t getCurrentFrameIndex() const;
    const bool setCurrentFrameIndex(const size_t aFrameIndex,
                                    const bool aLoad = false);

    const bool loadFrame(const size_t aFrameIndex);
    const bool nextFrame();

    // Ground truth and other data
    bool getGroundTruth(RotTransData &aGroundTruth) const;
    bool getGroundTruth(RotTransData &aGroundTruth, const size_t aIndex) const;

    // Sequential Processing or Feed
    void run(const int aStartFrameID, const int aEndFrameID,
             const fs::path &aPoseOutputFilePath,
             const Pose2D<double> &aInitPose = Pose2D<double>(0, 0, 0));
};

#endif