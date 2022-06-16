/**
 * @file RadarImageHandler.cpp
 * @brief Handler and helper functions for RadarImage class
 * @see RadarImage.cpp
 * @see RadarImage.hpp
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#include "RadarImageHandler.hpp"

namespace fs = std::filesystem;

/**
 * @brief Returns the image path of the `aImageNumber`-th image in dataset
 * `setNumber_str` according to the order specified in the relevant
 * timestamp file.
 *
 * @param[out] aImagePath String to path of image. Unchanged string if invalid.
 * @param[in] aSetNumber: radar dataset id/number
 * @param[out] aImageNumber: radar image number, 0-indexed
 *
 * @return Success status
 */
bool imagePathFromTimestamp(std::string &aImagePath, unsigned int aSetNumber,
                            unsigned int aImageNumber) {
    // Get base path
    fs::path base_path(".");
    base_path /= "data";
    base_path /= std::to_string(aSetNumber);

    // Get timestamp file and read from it
    fs::path timestamp_path = fs::path(base_path);
    timestamp_path /= "radar.timestamps";

    std::ifstream timestamp_file(timestamp_path);

    // Invalid file
    if (!timestamp_file.is_open()) {
        printf("Timestamp file %.100s not found\n",
               timestamp_path.string().c_str());

        timestamp_file.close();
        return false;
    }

    // Valid file
    /*
    printf("Reading from timestamp file %s... ",
               timestamp_path.string().c_str());
    */

    std::string image_name;
    unsigned int i = 0;
    int validity;
    bool found = false;

    while (timestamp_file >> image_name >> validity) {
        if (i == aImageNumber) {
            found = true;
            break;
        }
        i++;
    }

    timestamp_file.close();

    // ERROR: File not found
    if (!found) {
        printf("Image not found!\n");

#ifdef _WIN32
        fprintf(stderr,
                "Error: Could not find image %u from timestamp file %s!\n",
                aImageNumber, timestamp_path.string().c_str());
#else
        fprintf(stderr,
                "Error: Could not find image %u from timestamp file %s!\n",
                aImageNumber, timestamp_path.string().c_str());
#endif

        return false;
    }

    // Image file has been found!
    // printf("Image %.16s found!\n", image_name.c_str());

    fs::path radar_image_path = fs::path(base_path);
    radar_image_path /= "radar";
    radar_image_path /= image_name + ".png";

    aImagePath = radar_image_path.string();

    if (!validity) {
        printf("Image found but marked not valid.\n");
        return false;
    }

    return true;
}

/**
 * @brief Imreads image from given file path
 *
 * @param[in] image_path Path to radar image
 * @param[out] aImage Image (empty if invalid)
 *
 * @return Success status
 */
bool readImageFromPath(cv::Mat &aImage, const std::string &image_path) {
    // printf("Reading image from %s... ", image_path.c_str());

    if (image_path == "" || !fs::exists(image_path)) {
        // printf("Failed!\n");
        printf("Failed to read from file %s: Does not exist.\n",
               image_path.c_str());

#ifdef _WIN32
        fprintf_s(stderr, "Error: file %s does not exist!\n",
                  image_path.c_str());
#else
        fprintf(stderr, "Error: file %s does not exist!\n", image_path.c_str());
#endif

        return false;
    }

    aImage = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
    // printf("Success!\n");

    return true;
}

/**
 * @brief Returns a Mat image as an OpenCV imread of radar image from set
 * `aSetNumber`, image `aImageNumber` (0-indexed, ordered by timestamp)
 *
 * @param[in] aSetNumber Radar dataset id/number
 * @param[in] aImageNumber Radar image number, 0-indexed
 * @param[out] aImage OpenCV Mat image (empty Mat if error)
 *
 * @return Success status
 */
bool imageFromRadarData(cv::Mat &aImage, unsigned int aSetNumber,
                        unsigned int aImageNumber) {
    std::string radar_image_path;
    bool read_success =
        imagePathFromTimestamp(radar_image_path, aSetNumber, aImageNumber);

    if (read_success) {
        readImageFromPath(aImage, radar_image_path);
        return true;
    }
    else {
        printf("Cannot get image %u from dataset %u\n", aImageNumber,
               aSetNumber);
        return false;
    }

    return false;
}

/**
 * @brief Downsamples input image to a certain size in radial (x) direction.
 *
 * @param[in] aSrcImage Source radar image
 * @param[out] aDestImage Destination downsampled radar image
 * @param[in] aDownsampleWidth downsample to this size
 * @note original image if error
 */
void imageDownsampleRadial(const cv::Mat &aSrcImage, cv::Mat &aDestImage,
                           int aDownsampleWidth) {
    cv::Mat downsampled;
    cv::Size imageSize = aSrcImage.size();
    cv::Size downsampleSize = aSrcImage.size();

    downsampleSize.width = aDownsampleWidth;

    if (aDownsampleWidth >= imageSize.width) {
        aDestImage = aSrcImage;
        return;
    }

    // ? should we use INTER_NEAREST (fastest) or LINEAR?
    cv::resize(aSrcImage, aDestImage, downsampleSize, 0, 0, cv::INTER_LINEAR);

    return;
}

/**
 * @brief Converts input image from polar to Cartesian form
 *
 * @param[in] aSrcImage Source radar image (polar)
 * @param[out] aDestImage Destination radar image (Cartesian)
 */
void imagePolarToCartesian(const cv::Mat &aSrcImage, cv::Mat &aDestImage) {
    cv::Mat imageCart;
    cv::Size imageSize = aSrcImage.size();
    cv::Size imageCartSize = cv::Size();

    double maxRadius = imageSize.width;
    imageCartSize.width = static_cast<int>(maxRadius * 2);
    imageCartSize.height = imageCartSize.width;

    cv::Point2d center(imageCartSize.width / 2, imageCartSize.height / 2);

    // NOTE: Using INTER_NEAREST (0)
    int flags =
        cv::INTER_NEAREST + cv::WARP_INVERSE_MAP + cv::WARP_FILL_OUTLIERS;

    cv::warpPolar(aSrcImage, aDestImage, imageCartSize, center, maxRadius,
                  flags);

    return;
}

/**
 * @brief Converts input image from Cartesian to log polar form
 *
 * @param[in] aSrcImage Source radar image (Cartesian)
 * @param[out] aDestImage Destination radar image (polar)
 */
void imageCartesianToLogPolar(const cv::Mat &aSrcImage, cv::Mat &aDestImage) {
    cv::Size imageSize = aSrcImage.size();
    cv::Size imageLogPolarSize = aSrcImage.size();

    double maxRadius = imageSize.width / 2;
    cv::Point2d center(maxRadius, imageLogPolarSize.height / 2);

    // TODO: Test different interpolation
    int flags = cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS;

    cv::warpPolar(aSrcImage, aDestImage, imageLogPolarSize, center, maxRadius,
                  flags + cv::WARP_POLAR_LOG);
}

/**
 * @brief Converts input image from log polar to Cartesian form
 *
 * @param[in] aSrcImage Source radar image (polar)
 * @param[out] aDestImage Destination radar image (Cartesian)
 */
void imageLogPolarToCartesian(const cv::Mat &aSrcImage, cv::Mat &aDestImage) {
    cv::Size imageSize = aSrcImage.size();
    cv::Size imageCartSize = aSrcImage.size();

    double maxRadius = imageSize.width / 2;
    cv::Point2d center(maxRadius, imageCartSize.height / 2);

    // TODO: Test different interpolation
    int flags = cv::INTER_LINEAR + cv::WARP_FILL_OUTLIERS;

    cv::warpPolar(aSrcImage, aDestImage, imageCartSize, center, maxRadius,
                  flags + cv::WARP_INVERSE_MAP + cv::WARP_POLAR_LOG);
}

/**
 * @brief Crops out a sub image from input image, but crops only the range
 * (happens to be width of image)
 *
 * @param[in] aSrcImage Source image to crop
 * @param[out] aDestImage Destination image to crop
 * @param[in] aCropStart Crop starting from here (pixels)
 * @param[in] aCropWidth Crop up to this many pixels
 *
 * @return OpenCV Mat image (original image if error)
 */
void imageCropRange(const cv::Mat &aSrcImage, cv::Mat &aDestImage,
                    const unsigned int aCropStart,
                    const unsigned int aCropWidth, const bool aAsReference) {
    cv::Mat subImage;
    cv::Size imageSize = aSrcImage.size();

    // Ensure crop within range
    cv::Rect roi(aCropStart, 0, MIN(imageSize.width, aCropWidth), imageSize.height);

    cv::Mat subImageRef(aSrcImage, roi);
    subImageRef.copyTo(subImage);

    if (aAsReference)
        aDestImage = subImageRef;
    else
        aDestImage = subImage;
}

/**
 * @brief Compute confidence value of odometry and heading data according to
 * paper's equation
 * @param[in] aData Rotational and translational difference
 * @return Confidence value
 */
double computeConfidenceLevel(const RotTransData &aData) {
    double angleFromYX, angleFromHeading;

    angleFromYX = atan2(aData.dy, aData.dx);
    angleFromHeading = aData.dRotRad;

    return exp(-abs(angleFromYX - angleFromHeading));
}

/**
 * @brief Convert from RotTransData to TransData
 * @param[out] aTransData TransData to be converted to from RotTransData
 * @param[in] aRotTransData Reference to RotTransData to convert from
 */
void convertRotTransToTransData(TransData &aTransData,
                                const RotTransData &aRotTransData) {
    aTransData.dx = aRotTransData.dx;
    aTransData.dy = aRotTransData.dy;
}

/**
 * @brief Extract format information from a polar radar beam
 * @param[in] 
 */
const MetaData extractMetaDataFromImage(const cv::Mat &aMetaDataImg) {
	const int M = aMetaDataImg.rows;
    const int N = aMetaDataImg.cols;

    // Reserve meta data information, locally cached for efficiency
    MetaData metaData;
    MetaDataList timestamps;
    MetaDataList azimuths;

    timestamps.reserve(M);
    azimuths.reserve(M);


    // Every row of image (each azimuth value)
    // NOTE:
    /// - cols 0-7: Timestamp
    /// - cols 8-9: Sweep counter (used for azimuth calculation)
    
    double timestamp, azimuth, sweep_counter;

	for (int i = 0; i < M; i++) {
        std::string str_info;
    
        // TODO: Make this more efficient - matrix iteration like this can be quite slow
        // TODO: Potentially use unions?

        // Extract timestamp information
        for (int j = 0; j < 8; j++) {
            int val = static_cast<int>(aMetaDataImg.at<double>(i, j));
            std::string str = std::bitset<8>(val).to_string();
            str_info = str + str_info;
        }
        
        timestamp = static_cast<double>(std::stoll(str_info, nullptr, 2));
        timestamps.push_back(timestamp);

        // Extract azimuth information
        for (int j = 8; j < 10; j++) {
            int val = static_cast<int>(aMetaDataImg.at<double>(i, j));
            std::string str = std::bitset<8>(val).to_string();
            str_info = str + str_info;
        }

        sweep_counter = static_cast<double>(std::stoll(str_info, nullptr, 2));
        azimuth = sweep_counter * SWEEP_COUNTER_TO_AZIM;
        azimuths.push_back(azimuth);
    }

    metaData.azimuths = azimuths;
    metaData.timestamps = timestamps;

	return metaData;
}