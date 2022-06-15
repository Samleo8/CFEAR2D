/**
 * @file RadarFeedHandler.cpp
 * @brief Handler and helper functions for RadarFeed class
 * @see RadarFeed.cpp/hpp
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#include "RadarFeedHandler.hpp"

namespace fs = std::filesystem;

/**
 * @brief Gets all data from the specified folder: vector of file paths of radar
 * images as well as ground truth data
 * @note Function does not load the RadarImages as that takes too long
 *
 * @param[in] aFolderPath Path to folder to get data from
 * @param[out] aImagePathVector Vector of RadarImage that holds the feed
 * @param[out] aGroundTruthVector Vector of ground truth structs which holds the
 * ground truth data
 *
 * @return Success status
 */
bool getDataFromFolder(const std::filesystem::path &aFolderPath,
                       std::vector<std::string> &aImagePathVector,
                       std::vector<RotTransData> &aGroundTruthVector) {
    const fs::path &basePath = aFolderPath;

    // Get timestamp file and read from it
    fs::path timestampPath = fs::path(basePath);
    timestampPath /= "radar.timestamps";

    fs::path imageBasePath = basePath / "radar";

    bool success = getImagePathsFromTimestampFile(
        timestampPath, aImagePathVector, imageBasePath.string());

    if (!success) {
        printf("Failed to read from timestamp file %s\n",
               timestampPath.string().c_str());
        return false;
    }

    fs::path csvFilePath = basePath / "gt";
    csvFilePath /= "radar_odometry.csv";

    success = getGroundTruthFromCSVFile(csvFilePath, aGroundTruthVector);

    if (!success) {
        printf("Failed to read data from CSV file %s\n",
               csvFilePath.string().c_str());
        return false;
    }

    return true;
}

/**
 * @brief Gets all data from the specified folder: vector of file paths of radar
 * images as well as ground truth data
 * @note Function does not load the RadarImages as that takes too long
 *
 * @param[in] aFolderName Path to folder to get data from
 * @param[out] aImagePathVector Vector of RadarImage that holds the feed
 * @param[out] aGroundTruthVector Vector of ground truth structs which holds the
 * ground truth data
 *
 * @return Success status
 */
bool getDataFromFolder(const std::string &aFolderName,
                       std::vector<std::string> &aImagePathVector,
                       std::vector<RotTransData> &aGroundTruthVector) {
    return getDataFromFolder(fs::path(aFolderName), aImagePathVector,
                             aGroundTruthVector);
}

/**
 * @brief Reads image paths from timestamp file and adds them to a vector of
 * image paths (strings)
 *
 * @param[in] aTimestampPath Path to timestamp file with which to read image
 * paths from
 * @param[out] aImageFileVector Vector of strings of image files
 * @param[in] aBasePath Base path to prepend to image path
 * @param[in] aFileFormat Format of image (eg. "png" or "jpg")
 *
 * @return Success status
 */
bool getImagePathsFromTimestampFile(const std::string &aTimestampPath,
                                    std::vector<std::string> &aImageFileVector,
                                    const std::string &aBasePath,
                                    const std::string &aFileFormat) {
    std::ifstream timestampFile(aTimestampPath);

    if (!timestampFile.is_open()) {
        printf("Timestamp file %.100s not found\n", aTimestampPath.c_str());

        timestampFile.close();
        return false;
    }

    std::string imageName;
    int validity;

    fs::path basePath(aBasePath);

    // Optimise speed by pre-reserving memory
    aImageFileVector.reserve(aImageFileVector.size() + ESTIMATED_FILE_SIZE);

    // Read in image paths
    while (timestampFile >> imageName >> validity) {
        fs::path imagePath = basePath / imageName;
        aImageFileVector.push_back(imagePath.string() + "." + aFileFormat);
    }

    timestampFile.close();
    return true;
}

/**
 * @brief Reads image paths from timestamp file and adds them to a vector of
 * image paths (strings)
 *
 * @param[in] aTimestampPath Path to timestamp file with which to read image
 * paths from
 * @param[out] aImageFileVector Vector of strings of image files
 * @param[in] aBasePath Base path to prepend to image path
 * @param[in] aFileFormat Format of image (eg. "png" or "jpg")
 *
 * @return Success status
 */
bool getImagePathsFromTimestampFile(const std::filesystem::path &aTimestampPath,
                                    std::vector<std::string> &aImageFileVector,
                                    const std::string &aBasePath,
                                    const std::string &aFileFormat) {
    return getImagePathsFromTimestampFile(
        aTimestampPath.string(), aImageFileVector, aBasePath, aFileFormat);
}

/**
 * @brief Reads the ground truth data from a CSV file and stores the
 * relevant info in a vector of structs
 *
 * @param[in] aCSVFilePath Path to CSV file
 * @param[out] aGroundTruthVector Vector of ground truth structs
 *
 * @return Success status
 */
bool getGroundTruthFromCSVFile(const std::string &aCSVFilePath,
                               std::vector<RotTransData> &aGroundTruthVector) {
    std::ifstream csvFile(aCSVFilePath);

    // Wanted columns are x,y,yaw
    // NOTE: Should always end with TOTAL_WANTED_COLS
    enum wantedColumns { X, Y, YAW, TOTAL_WANTED_COLS };

    int wantedColIds[TOTAL_WANTED_COLS];
    int colId = 0, wantedCnt = 0;

    // Read the column names
    std::string line, colName;
    if (csvFile.good()) {
        // Extract the first line in the file
        std::getline(csvFile, line);

        // Create a stringstream from line
        std::stringstream ss(line);

        // Extract each column name
        while (std::getline(ss, colName, ',')) {
            if (colName == "x") {
                wantedColIds[X] = colId;
                wantedCnt++;
            }
            else if (colName == "y") {
                wantedColIds[Y] = colId;
                wantedCnt++;
            }
            else if (colName == "yaw") {
                wantedColIds[YAW] = colId;
                wantedCnt++;
            }

            colId++;
        }
    }
    else {
        printf("Ground truth CSV File %s invalid!\n", aCSVFilePath.c_str());
        return false;
    }

    // Check to make sure all wanted cols exist, otherwise return false
    if (wantedCnt != TOTAL_WANTED_COLS) {
        printf("At least one of the wanted columns does not exist!\n");
        return false;
    }

    // Optimise speed by pre-reserving memory
    aGroundTruthVector.reserve(aGroundTruthVector.size() + ESTIMATED_FILE_SIZE);

    // Read data, line by line
    double data;
    while (std::getline(csvFile, line)) {
        // Create a stringstream of the current line
        std::stringstream ss(line);

        // Keep track of the current column index
        colId = 0;
        wantedCnt = 0;

        RotTransData gt;
        // Extract each integer
        while (ss >> data) {
            if (colId == wantedColIds[X]) {
                gt.dx = data;
                wantedCnt++;
            }
            else if (colId == wantedColIds[Y]) {
                gt.dy = data;
                wantedCnt++;
            }
            else if (colId == wantedColIds[YAW]) {
                gt.dRotRad = data;
                wantedCnt++;
            }

            // If the next token is a comma, ignore it and move on
            if (ss.peek() == ',') ss.ignore();

            // Increment the column index
            colId++;
        }

        // TODO: Should we use emplace back instead?
        aGroundTruthVector.push_back(gt);
    }

    return true;
}
/**
 * @brief Reads the ground truth data from a CSV file and stores the
 * relevant info in a vector of structs
 *
 * @param[in] aCSVFilePath Path to CSV file
 * @param[out] aGroundTruthVector Vector of ground truth structs
 *
 * @return Success status
 */
bool getGroundTruthFromCSVFile(const std::filesystem::path &aCSVFilePath,
                               std::vector<RotTransData> &aGroundTruthVector) {
    return getGroundTruthFromCSVFile(aCSVFilePath.string(), aGroundTruthVector);
}
