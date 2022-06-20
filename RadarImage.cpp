/**
 * @file RadarImage.cpp
 *
 * @brief Radar Image class that performs pre-processing on the images. Will
 * contain all the needed information about the radar image, such as id,
 * pre-processed image cv::Mat etc.
 *
 * @see RadarImageHandler.cpp/hpp
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#include "RadarImage.hpp"
#include "RadarImageHandler.hpp"

/******************************************************
 * @section RadarImage-ConstInit Constructors
 ******************************************************/

/**
 * @brief Loads raw image into RadarImage class. Takes different possible inputs
 *
 * @param[in] aImage OpenCV Mat
 *
 * @return Success status
 */
bool RadarImage::loadImage(const cv::Mat &aImage) {
    if (!aImage.data) {
        mLoaded = false;
        return false;
    }

    mRawImage = aImage;
    mLoaded = true;
    return mLoaded;
}

/**
 * @brief Loads raw image into RadarImage class. Takes different possible inputs
 *
 * @param[in] aImagePath Path to raw image
 *
 * @return Success status
 */
bool RadarImage::loadImage(const std::string &aImagePath) {
    mLoaded = readImageFromPath(mRawImage, aImagePath);
    return mLoaded;
}

/**
 * @brief Loads raw image into RadarImage class. Takes different possible inputs
 *
 * @param[in] aSetNumber Set number
 * @param[in] aImageNumber Image number
 *
 * @return Success status
 */
bool RadarImage::loadImage(unsigned int aSetNumber, unsigned int aImageNumber) {
    mLoaded = imageFromRadarData(mRawImage, aSetNumber, aImageNumber);
    return mLoaded;
}

/**
 * @brief Constructs RadarImage class. Loads raw image and preprocesses them.
 * Can load by OpenCV Mat, image path or set/image number.
 * Use this empty construtor if you want to load raw image without
 * pre-processing it.
 *
 * NOTE: This empty constructor does nothing. To load and pre-process the
 * images, you need to use the respective functions explicitly.
 */
RadarImage::RadarImage() {
    mLoaded = false;
    mPreprocessed = false;
}

/**
 * @brief Constructs RadarImage class. Loads raw image and preprocesses them.
 * Can load by OpenCV Mat, imagePath or set/image number
 *
 * @param[in] aImage Raw image (OpenCV Mat format)
 * @param[in] aPreprocess Flag whether or not to preprocess images
 */
RadarImage::RadarImage(const cv::Mat &aImage, const bool aPreprocess) {
    bool success = loadImage(aImage);

    if (success) {
        if (aPreprocess) preprocessImages();
    }
    else {
        printf("Failed to load image! No processing done!");
    }
}

/**
 * @brief Constructs RadarImage class. Loads raw image and preprocesses them.
 * Can load by OpenCV Mat, image path or set/image number
 *
 * @param[in] aImagePath Path to raw image
 * @param[in] aPreprocess Flag whether or not to preprocess images
 */
RadarImage::RadarImage(const std::string &aImagePath, const bool aPreprocess) {
    bool success = loadImage(aImagePath);

    if (success) {
        if (aPreprocess) preprocessImages();
    }
    else {
        printf("Failed to load image! No processing done!");
    }
}

/**
 * @brief Constructs RadarImage class. Loads raw image and preprocesses them.
 * Can load by OpenCV Mat, image path or set/image number
 *
 * NOTE: Data is found in the ./data folder and organised by
 * `./data/<aSetNumber>/radar/<images>`. See README for more details.
 * @param[in] aSetNumber Set number of input data
 * @param[in] aImageNumber Image number in set `aSetNumber` according to the
 * radar.timestamps file
 * @param[in] aPreprocess Flag whether or not to preprocess images
 */
RadarImage::RadarImage(unsigned int aSetNumber, unsigned int aImageNumber,
                       const bool aPreprocess) {
    bool success = loadImage(aSetNumber, aImageNumber);

    if (success) {
        mLoaded = true;
        if (aPreprocess) preprocessImages();
    }
    else {
        printf_err("Failed to load image! No processing done!");
        mLoaded = false;
        mPreprocessed = false;
    }
}

/******************************************************
 * @section RadarImage-GetterSetter Getters/Setters
 ******************************************************/

/**
 * @brief Checks if image is loaded
 */
bool RadarImage::isLoaded() {
    return mLoaded;
}

/**
 * @brief Checks if image has been preprocessed
 */
bool RadarImage::isProcessed() {
    return mPreprocessed;
}

/******************************************************
 * @section RadarImage-ImageInterface Image Interfaces
 ******************************************************/

/**
 * @brief Get raw image (includes metadata)
 * @return Get raw image
 */
const cv::Mat &RadarImage::getImageRaw() {
    return mRawImage;
}

/**
 * @brief Get polar image (i.e. range-azimuth image)
 * @return Get polar (range-azimuth) image
 */
const cv::Mat &RadarImage::getImagePolar() {
    return mPolarImage;
}

/**
 * @brief Get polar image (i.e. range-azimuth image)
 * @return Get polar (range-azimuth) image
 */
const cv::Mat &RadarImage::getImageMetaData() {
    return mMetaDataImage;
}

/**
 * @brief Get cartesian image (preprocessed from image downsampled in
 * radial direction) NOTE: Preprocessing must be done first!
 *
 * @return Coarse cartesian aImage
 */
const cv::Mat &RadarImage::getImageCart() {
    return mCartImage;
}

/**
 * @brief Get coarse log-polar image (preprocessed from image downsampled in
 * radial direction)
 * @deprecated Log polar image currently unused
 * NOTE: Preprocessing must be done first!
 * @todo If preprocessing is not done, then perform the relevant conversion
 * function
 *
 * @return Coarse log-polar aImage
 */
const cv::Mat &RadarImage::getImageLogPolar() {
    return mLogPolarImage;
}

/**
 * @brief Gets one of `type` (enum) image associated with this radar image.
 *        For example, obtain raw image via `getImage(RIMG_RAW)`
 *
 * NOTE: This aImage can only be obtained after preprocessing is done
 *
 * @returns Reference to aImage of specified type; raw image if error
 */
const cv::Mat &RadarImage::getImage(ImageType aType) {
    switch (aType) {
        case RIMG_RAW:
            return getImageRaw();
        case RIMG_POLAR:
            [[fallthrough]];
        case RIMG_RANGE_AZIM:
            return getImagePolar();
        case RIMG_CART:
            return getImageCart();
        case RIMG_LOGPOLAR:
            return getImageLogPolar();
        default:
            printf_err("Invalid image type! Returning raw image!\n");
            return mRawImage;
    }
}

/**
 * @brief Display image using OpenCV
 * @note Mainly for testing only
 * @param[in] aType Image type to display
 * @param[in] aTitle Image title
 * @param[in] aWaitKey Specifies whether or not to wait for a keyPress to
 * continue. Otherwise, the user is responsible for destroying the window
 * @param[in] aDestroy Specifies whether or not to destroy the window. Only
 * valid if aWaitKey = true
 */
void RadarImage::displayImage(ImageType aType, const std::string &aTitle,
                              const bool aWaitKey, const bool aDestroy) {
    const cv::Mat img = getImage(aType);
    cv::Mat imgToShow;

    if (img.type() == CV_32FC1 || img.type() == CV_64FC1) {
        cv::normalize(img, imgToShow, 0, 1, cv::NORM_MINMAX);
    }
    else {
        imgToShow = img;
    }

    cv::namedWindow(aTitle, cv::WINDOW_AUTOSIZE);
    cv::imshow(aTitle, imgToShow);

    if (aWaitKey) {
        cv::waitKey(0);
        if (aDestroy) cv::destroyWindow(aTitle);
    }

    return;
}

/**
 * @brief Display image using OpenCV
 * @note Mainly for testing only
 * @param[in] aType Image type to display
 * @param[in] aWaitKey Specifies whether or not to wait for a keyPress to
 * continue. Otherwise, the user is responsible for destroying the window
 * @param[in] aDestroy Specifies whether or not to destroy the window. Only
 * valid if aWaitKey = true
 */
void RadarImage::displayImage(ImageType aType, const bool aWaitKey,
                              const bool aDestroy) {
    const std::string title = "Display Radar Image";

    displayImage(aType, title, aWaitKey, aDestroy);
    return;
}

// (Pre-)Processing
/**
 * @brief Performs preprocessing of images. Generates Cartesian and
 * polar from raw log-polar image.
 * @note Need to
 * @todo Try different filtering methods
 */
void RadarImage::preprocessImages() {
    if (!mRawImage.data) {
        printf_err("Error: Cannot preprocess image: Image has no data!\n");

        mPreprocessed = false;
        return;
    }

    // Crop raw image to get metadata and polar image, AS REFERENCE of raw image
    // NOTE: Oxford Dataset only
    // TODO: Figure out why referencing does not work
    imageCropRange(mRawImage, mMetaDataImage, 0, 10, false);
    imageCropRange(mRawImage, mPolarImage, 11, mRawImage.cols - 11, false);

    // Process the metadata here
    mMetaData = extractMetaDataFromImage(mMetaDataImage);

    // Convert to Cartesian and Polar
    imagePolarToCartesian(mPolarImage, mCartImage);

    // TODO: Possibly test a downsampling in the radial direction before k-max
    // processing

    // Obtain log polar image
    // NOTE: currently unused
    // imageCartesianToLogPolar(mCartImage, mLogPolarImage);

    /*
    // Downsample Image first
    imageDownsampleRadial(mRawImage, mDownsampledImage,
                          RADAR_IMAGE_DOWNSAMPLED_RANGE);

    // Obtain coarse cartesian image
    imagePolarToCartesian(mDownsampledImage, mCartImage);

    // Obtain cartesian of sub image (in full res)
    // TODO: Try different ranges and starting positions
    imageCropRange(mRawImage, mSubImage, 0, RADAR_IMAGE_SUB_RANGE);
    imagePolarToCartesian(mSubImage, mSubImageCart);
    */

    mPreprocessed = true;
    return;
}

/**
 * @brief Efficient way of getting top k indices and values via priority
 * queue
 * @note Should be approximately n log kpq
 * @ref
 * https://stackoverflow.com/questions/14902876/indices-of-the-k-largest-elements-in-an-unsorted-length-n-array
 * @param[in] aAzim Pointer to azimuth array
 * @param[in] aSize
 * @param[out] aTopKVec Vector of top k
 */
void RadarImage::getTopK(const uint8_t *aAzim, const size_t aSize,
                         const size_t aK,
                         std::vector<ValueIndexPair> &aTopKVec) {
    std::priority_queue<ValueIndexPair> pq;

    // Push relevant ValueIndexPairs into the PQ (fast)
    // While pruning away the elements that don't need to be in the queue,
    // thus reducing the size of the queue and improving efficiency
    for (size_t i = 0; i < aSize; i++) {
        double val = aAzim[i];
        if (pq.size() < aK) {
            pq.push(std::make_pair(val, i));
        }
        else if (val > pq.top().first) {
            pq.pop();
            pq.push(std::make_pair(val, i));
        }
    }

    // Now pop the top k elements of our pruned PQ and populate the vector
    aTopKVec.clear();
    aTopKVec.reserve(aK);

    size_t kpq = pq.size();
    for (size_t i = 0; i < kpq; i++) {
        aTopKVec.push_back(pq.top());
        pq.pop();
    }
}

/**
 * @brief Perform K Strong filtering on radar image
 * @note Internally adds onto or populates the mFilteredPoints vector
 *
 * @param[in] aK Number of points (k) to keep after filtering
 * @param[in] aZmin Minimum power value to count as valid feature point
 * @param[in] aClearOld Whether to clear existing feature points vector
 */
void RadarImage::performKStrong(const size_t aK, const double aZmin,
                                const bool aClearOld) {
    if (!mPreprocessed) {
        printf_err("Error: Cannot perform K-Strong filtering: Image has not "
                   "been preprocessed!\n");
        return;
    }

    // TODO: Possibly try point-cloud generating technique from RadarSLAM

    // Setup mFilteredPoints vector
    if (aClearOld) mFilteredPoints.clear();

    const size_t sz = mFilteredPoints.size();
    mFilteredPoints.reserve(sz + aK);

    // Get metadata information
    const MetaDataList<double> azimuths = mMetaData.azimuths;
    const MetaDataList<bool> isValid = mMetaData.isValid;

    // Perform the k-strong filtering by looping over each row (azimuth)
    // of the image And getting the top k points that are above threshold
    const cv::Mat &imgPolar = getImagePolar();
    const int M = imgPolar.rows;
    const int N = imgPolar.cols;

    for (int i = 0; i < M; i++) {
        // Get relevant values from metadata
        const double azimuth = azimuths[i];

        // Get the top k value index pairs
        const uint8_t *Mi_const = imgPolar.ptr<uint8_t>(i);
        std::vector<ValueIndexPair> topKVec;

        getTopK(Mi_const, N, aK, topKVec);

        // Now loop through the top k and add them to the feature points vector
        for (int j = 0; j < aK; j++) {
            const ValueIndexPair &pair = topKVec[j];

            const double val = pair.first;
            const size_t idx = pair.second;

            // Only accept values which are also above Zmin
            if (val > aZmin) {
                PointCart2D pointCart;
                PointPolar2D pointPolar;

                const double range =
                    (static_cast<double>(idx) + 1.0) * RANGE_RESOLUTION;

                // std::cout << "Range: " << range << std::endl;

                pointPolar.R = range;
                pointPolar.theta = azimuth;

                pointPolar.toCartesian(pointCart);

                mFilteredPoints.push_back(pointCart);
            }
        }
    }
}

/**
 * @brief Get member variable of filtered points
 *
 * @return Vector of filtered points
 */
const FilteredPointsVec &RadarImage::getFilteredPoints() {
    return mFilteredPoints;
}

/**
 * @brief Performs FFT on image using OpenCV implementation of DFT
 *
 * @param[in] aSrcImageType Enum type of source image to perform FFT on
 * @param[out] aDestImage Destination image
 */
void RadarImage::performFFTOnImage(ImageType &aSrcImageType,
                                   cv::Mat &aDestImage) {
    cv::Mat srcImage = getImage(aSrcImageType);
    performFFTOnImage(srcImage, aDestImage);

    return;
}

/**
 * @brief Performs FFT on image using OpenCV implementation of DFT
 *
 * @param[in] aSrcImage Source image to perform FFT on
 * @param[out] aDestImage Destination image
 *
 * @return FFT magnitude cv::Mat
 */
void RadarImage::performFFTOnImage(const cv::Mat &aSrcImage,
                                   cv::Mat &aDestImage) {
    cv::Mat F0[2];
    ForwardFFT(aSrcImage, F0, true);
    cv::magnitude(F0[0], F0[1], aDestImage);

    return;
}