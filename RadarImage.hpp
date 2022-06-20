/**
 * @file RadarImage.hpp
 *
 * @brief Radar Image class that performs pre-processing on the images. Will
 * contain all the needed information about the radar image, such as id,
 * pre-processed image cv::Mat etc.
 *
 * @see RadarImageHandler.cpp
 * @see RadarImageHandler.hpp
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#ifndef __RADAR_IMAGE_H__
#define __RADAR_IMAGE_H__

#include "ImageProcessing.hpp"   // needed for Fourier transforms
#include "RadarImageHandler.hpp" // needed to handle file path and data
#include "OrientedSurfacePointsHandler.hpp"

// Constants
/** @brief Tau = 2 * Pi. Used for radian conversion and img proc */
#ifndef M_TAU
#define M_TAU 6.2831853071795864769
#endif

/** @brief Range resolution per pixel in M */
const double RANGE_RESOLUTION = 0.49107142857142855; // m

/** @brief Default high-pass filter size */
const double DEFAULT_FILTER_SIZE = 150;

/**
 * @brief Precalcuated multiplier for normalising angle in getRotationDiff
 * @note Image width = 672px in this case
 */
static const unsigned int IMAGE_WIDTH_PIXELS = 672;
static const double NORMALISE_ANGLE_MULTIPLIER = (M_TAU / IMAGE_WIDTH_PIXELS);

/**
 * @brief Sub pixel systematic error
 * @note Centroid function of OpenCV phaseCorrelate() seems to cause a
 * systematic error
 * @todo To remove and replace with dynamic covariance
 */
const double TRANSLATION_SYSTEMATIC_ERROR = 0.5;

/**
 * @brief RadarImage class that performs pre-processing on the images. Will
 * contain all the needed information about the radar image, such as id,
 * pre-processed image cv::Mat etc.
 *
 * @see RadarImageHandler.cpp
 * @see RadarImageHandler.hpp
 */
class RadarImage {
  private:
    // All preprocessed images
    // TODO: Remove the ones we don't need to store, esp raw

    cv::Mat mRawImage;      ///< Raw range-azimuth image
    cv::Mat mPolarImage;    ///< Raw range-azimuth image
    cv::Mat mMetaDataImage; ///< Metadata image (Oxford dataset only)
    cv::Mat mCartImage;     ///< Cartesian image

    MetaData mMetaData; ///< Metadata information (azimuth and timestamps)

    /** 
     * @brief Points (Cartesian) found from filtering
     * @note In sensor coordinates
     */
    FilteredPointsVec mFilteredPoints;

    /** 
     * @brief Downsampled grid of vector of filtered points
     * @note While one can save memory by using a representation involving only a running mean, 
     * it is more computationally efficient in the second step when searching for neighbours
     * because we only need to search points in the grid neighbours around the original grid squares.
     */
    FilteredPointsVec mORSPGrid[ORSP_GRID_N][ORSP_GRID_N];

    /** 
     * @brief Downsampled grid of centroid of filtered points
     */
    PointCart2D mORSPCentroidGrid[ORSP_GRID_N][ORSP_GRID_N];

    // TODO: Unused for now
    cv::Mat mLogPolarImage; ///< Downsampled log-polar image
    // cv::Mat mDownsampledImage;    ///< Downsampled range-azimuth image
    // cv::Mat mSubImage;     ///< Cropped range-azimuth sub-image
    // cv::Mat mSubImageCart; ///< Cropped Cartesian sub-image

    bool mLoaded = false;       ///< Flag of whether image has been loaded
    bool mPreprocessed = false; ///< Flag of whether image has been preprocessed

  public:
    // Constructors
    // by ids, by filename, by original cv::Mat
    bool loadImage(const cv::Mat &aImage);
    bool loadImage(const std::string &aImagePath);
    bool loadImage(unsigned int aSetNumber, unsigned int aImageNumber);

    RadarImage();
    RadarImage(const cv::Mat &aImage, const bool aPreprocess = true);
    RadarImage(const std::string &aImagePath, const bool aPreprocess = true);
    RadarImage(unsigned int aSetNumber, unsigned int aImageNumber,
               bool aPreprocess = true);

    // Getters Setters
    bool isLoaded();
    bool isProcessed();

    /**
     * @brief Interface to images. Used for specifying image type.
     */
    enum ImageType {
        RIMG_RAW,
        RIMG_METADATA,
        RIMG_POLAR,
        RIMG_RANGE_AZIM,
        // downsampled,
        RIMG_CART,
        RIMG_LOGPOLAR,
        // sub,
        // subCart
    };

    const cv::Mat &getImage(ImageType aType);
    const cv::Mat &getImageRaw();
    const cv::Mat &getImageMetaData();
    const cv::Mat &getImagePolar();
    const cv::Mat &getImageCart();
    const cv::Mat &getImageLogPolar();
    // const cv::Mat &getImageSubFullCart();

    // Image Display
    void displayImage(ImageType aType, const bool aWaitKey = true,
                      const bool aDestroy = true);
    void displayImage(ImageType aType, const std::string &aTitle,
                      bool aWaitKey = true, const bool aDestroy = true);

    // Image (pre-)processing
    void preprocessImages();

    // Filtering process
    void getTopK(const uint8_t *aAzim, const size_t aSize, const size_t aK,
                 std::vector<ValueIndexPair> &aTopKVec);
    void performKStrong(const size_t aK, const double aZmin,
                        const bool aClearOld = true);
    const FilteredPointsVec &getFilteredPoints();

    // Generating Oriented Surface Points
    void downsamplePointCloud();

    // NOTE: UNUSED
    void performFFTOnImage(ImageType &aSrcImageType, cv::Mat &aDestImage);
    void performFFTOnImage(const cv::Mat &aSrcImage, cv::Mat &aDestImage);
};

#endif