/**
 * @file RadarFeedHandler.hpp
 * @brief Handler and helper functions for RadarFeed class
 * @see RadarFeed.cpp/hpp
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#ifndef __RADAR_FEED_HANDLER_H__
#define __RADAR_FEED_HANDLER_H__

/** @note Needed on Windows to include math constants like M_PI */
#ifndef _USE_MATH_DEFINES
#define _USE_MATH_DEFINES
#include <cmath>
#endif // !_USE_MATH_DEFINES

#include <filesystem>
#include <fstream>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "RadarImage.hpp"
#include "RadarImageHandler.hpp"

/** @brief Estimated file size so we can reserve this amount in the vector */
#ifndef ESTIMATED_FILE_SIZE
#define ESTIMATED_FILE_SIZE 9000
#endif // !ESTIMATED_FILE_SIZE

bool getDataFromFolder(const std::filesystem::path &aFolderPath,
                       std::vector<std::string> &aImagePathVector,
                       std::vector<RotTransData> &aGroundTruthVector);

bool getDataFromFolder(const std::string &aFolderName,
                       std::vector<std::string> &aImagePathVector,
                       std::vector<RotTransData> &aGroundTruthVector);

bool getImagePathsFromTimestampFile(const std::string &aTimestampPath,
                                    std::vector<std::string> &aImageFileVector,
                                    const std::string &aBasePath = ".",
                                    const std::string &aFileFormat = "png");

bool getImagePathsFromTimestampFile(const std::filesystem::path &aTimestampPath,
                                    std::vector<std::string> &aImageFileVector,
                                    const std::string &aBasePath = ".",
                                    const std::string &aFileFormat = "png");

bool getGroundTruthFromCSVFile(const std::filesystem::path &aCSVFilePath,
                               std::vector<RotTransData> &aGroundTruthVector);

bool getGroundTruthFromCSVFile(const std::string &aCSVFilePath,
                               std::vector<RotTransData> &aGroundTruthVector);

#endif