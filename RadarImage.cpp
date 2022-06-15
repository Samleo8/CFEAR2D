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
        printf("Failed to load image! No processing done!");
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
 * @brief Get coarse cartesian image (preprocessed from image downsampled in
 * radial direction) NOTE: Preprocessing must be done first!
 *
 * @return Coarse cartesian aImage
 */
const cv::Mat &RadarImage::getImageCoarseCart() {
    return mCoarseCartImage;
}

/**
 * @brief Get coarse log-polar image (preprocessed from image downsampled in
 * radial direction)
 * NOTE: Preprocessing must be done first!
 *
 * @return Coarse log-polar aImage
 */
const cv::Mat &RadarImage::getImageLogPolar() {
    return mCoarseLogPolarImage;
}

/**
 * @brief Gets one of `type` (enum) image associated with this radar image.
 *        For example, obtain raw image via `getImage(raw)`
 *
 * NOTE: This aImage can only be obtained after preprocessing is done
 *
 * @returns Reference to aImage of specified type; raw image if error
 */
const cv::Mat &RadarImage::getImage(ImageType aType) {
    switch (aType) {
        case raw:
            return mRawImage;
        case downsampled:
            return mDownsampledImage;
        case coarseCart:
            return mCoarseCartImage;
        case coarseLogPolar:
            return mCoarseLogPolarImage;
        case sub:
            return mSubImage;
        case subCart:
            return mSubImageCart;
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

    // Crop away metadata
    // NOTE: Oxford Dataset only
    imageCropRange(mRawImage, mRawImage, 11, mRawImage.cols);

    // Convert to Cartesian and Polar 

    // TODO: Possibly test a downsampling in the radial direction before k-max processing
    // TODO: Possibly try point-cloud generating technique from RadarSLAM


    /*
    // Downsample Image first
    imageDownsampleRadial(mRawImage, mDownsampledImage,
                          RADAR_IMAGE_DOWNSAMPLED_RANGE);

    // Obtain coarse cartesian image
    imagePolarToCartesian(mDownsampledImage, mCoarseCartImage);

    // Obtain log polar image
    imageCartesianToLogPolar(mCoarseCartImage, mCoarseLogPolarImage);

    // Obtain cartesian of sub image (in full res)
    // TODO: Try different ranges and starting positions
    imageCropRange(mRawImage, mSubImage, 0, RADAR_IMAGE_SUB_RANGE);
    imagePolarToCartesian(mSubImage, mSubImageCart);
    */

    mPreprocessed = true;
    return;
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