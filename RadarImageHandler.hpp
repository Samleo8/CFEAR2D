/**
 * @file RadarImageHandler.hpp
 * @brief Handler and helper functions for RadarImage class
 * @see RadarImage.cpp
 * @see RadarImage.hpp
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#ifndef __RADAR_IMAGE_HANDLER_H__
#define __RADAR_IMAGE_HANDLER_H__

/** @note Needed on Windows to ensure math constants like M_PI are included */
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <math.h>
#endif // !_USE_MATH_DEFINES

#include <filesystem>
#include <fstream>
#include <opencv2/opencv.hpp>
#include <queue>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "PointCart2D.hpp"
#include "PointPolar2D.hpp"

/**
 * @brief Translation data struct (no rotation). Used in pose graph and sliding
 * window to store only translation data between 2 consecutive frames.
 */
typedef struct {
    double dx; ///< Translational change along x-axis [m]
    double dy; ///< Translational change along y-axis [m]
} TransData;

/**
 * @brief Rotation Translation data struct. Used in storing RadarImage to
 * RadarImage data (including ground truth)
 */
typedef struct {
    double dx;      ///< Translational change along x-axis [m]
    double dy;      ///< Translational change along y-axis [m]
    double dRotRad; ///< Rotational change [rad]
} RotTransData;

/** @brief Feature Point */
typedef PointCart2D FilteredPoint;

/** @brief Typedef for vector of feature points */
typedef std::vector<FilteredPoint> FilteredPointsVec;

/** @brief Typedef for list of metadata information (vector of doubles) */
template <typename T>
using MetaDataList = std::vector<T>;

/**
 * @brief Struct of metadata information (timestamps and azimuths)
 */
typedef struct {
    MetaDataList<int64_t> timestamps;
    MetaDataList<double> azimuths;
    MetaDataList<bool> isValid;
} MetaData;

/**
 * @brief Typedef for a value index pair. Used in k-strong filtering.
 * @see RadarImage::getTopK()
 */
typedef std::pair<double, size_t> ValueIndexPair;

// Defines
/** @brief Simple MAX function */
#ifndef MAX
#define MAX(a, b) (((a) > (b)) ? (a) : (b))
#endif

/** @brief Simple MIN function */
#ifndef MIN
#define MIN(a, b) (((a) < (b)) ? (a) : (b))
#endif

/** @brief Simple ABS function */
#ifndef ABS
#define ABS(a) ((a < 0) ? (-a) : (a))
#endif

/** @brief Print to stderr, with support for Windows' _s */
#ifndef printf_err
#ifdef _WIN32
#define printf_err(...) fprintf_s(stderr, __VA_ARGS__);
#else
#define printf_err(...) fprintf(stderr, __VA_ARGS__);
#endif
#endif

/** @brief Defines how much we should downsample the image by */
#define RADAR_IMAGE_DOWNSAMPLED_RANGE 336

/** @brief Defines how much we should cut the image by */
#define RADAR_IMAGE_SUB_RANGE 500

/** @brief Radar's maximum range in meters */
#define RADAR_MAX_RANGE_M 165

/** @brief Optimisation for checking if distance is less than radar max range */
#define RADAR_MAX_RANGE_M_2 (0.707 * RADAR_MAX_RANGE_M) // range * 1/sqrt(2)

/** @brief Square of radar max range */
#define RADAR_MAX_RANGE_M_SQUARED (RADAR_MAX_RANGE_M * RADAR_MAX_RANGE_M)

/** @brief Encoder size */
#define ENCODER_SIZE 5600

/** @brief Converts sweep counter values into azimuth in radians */
#define SWEEP_COUNTER_TO_AZIM (2 * M_PI / ENCODER_SIZE)

/** @brief Number of columns in metadata for timestamp information */
#define TIMESTAMP_N_COLS 8

/** @brief Number of columns in metadata for sweep counter information */
#define SWEEP_COUNTER_N_COLS 2

// Functions
bool imagePathFromTimestamp(std::string &aImagePath, unsigned int aSetNumber,
                            unsigned int aImageNumber);

bool readImageFromPath(cv::Mat &aImage, const std::string &aImagePath);

bool imageFromRadarData(cv::Mat &aImage, unsigned int aSetNumber,
                        unsigned int aImageNumber);

void imageDownsampleRadial(const cv::Mat &aSrcImage, cv::Mat &aDestImage,
                           int aDownsampleWidth);

void imagePolarToCartesian(const cv::Mat &aSrcImage, cv::Mat &aDestImage);

void imageCartesianToLogPolar(const cv::Mat &aSrcImage, cv::Mat &aDestImage);

void imageLogPolarToCartesian(const cv::Mat &aSrcImage, cv::Mat &aDestImage);

void imageCropRange(const cv::Mat &aSrcImage, cv::Mat &aDestImage,
                    const unsigned int aCropStart,
                    const unsigned int aCropWidth,
                    const bool aAsReference = false);

const MetaData extractMetaDataFromImage(const cv::Mat &aMetaDataImg);

double computeConfidenceLevel(const RotTransData &aData);

void convertRotTransToTransData(TransData &aTransData,
                                const RotTransData &aRotTransData);

#endif