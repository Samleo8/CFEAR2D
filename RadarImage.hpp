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

#ifndef __RADAR_IMAGE_2D_HPP__
#define __RADAR_IMAGE_2D_HPP__

#include "ORSP.hpp"
#include "OrientedSurfacePointsHandler.hpp" // some ORSP-related functions for the
#include "Pose2D.hpp"
#include "RadarImageHandler.hpp" // needed to handle file path and data
                                 // RadarImage class are defined here

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
     * @note While one can save memory by using a representation involving only
     * a running mean, it is more computationally efficient in the second step
     * when searching for neighbours because we only need to search points in
     * the grid neighbours around the original grid squares.
     */
    FilteredPointsVec mORSPGrid[ORSP_GRID_N][ORSP_GRID_N];

    /**
     * @brief Downsampled grid of centroid of filtered points
     */
    Eigen::Vector2d mORSPCentroidGrid[ORSP_GRID_N][ORSP_GRID_N];

    /** @brief Vector of oriented surface points, local coordinates */
    ORSPVec<double> mORSPFeaturePoints;

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
    bool isLoaded() const;
    bool isProcessed() const;

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

    /**
     * @brief Enum for filtering algorithm
     * @return Raw image
     */
    enum FilteringAlgorithm { K_STRONGEST, OS_CFAR };

    const cv::Mat &getImage(ImageType aType) const;
    const cv::Mat &getImageRaw() const;
    const cv::Mat &getImageMetaData() const;
    const cv::Mat &getImagePolar() const;
    const cv::Mat &getImageCart() const;
    const cv::Mat &getImageLogPolar() const;
    // const cv::Mat &getImageSubFullCart();

    // Image Display
    void displayImage(ImageType aType, const bool aWaitKey = true,
                      const bool aDestroy = true) const;
    void displayImage(ImageType aType, const std::string &aTitle,
                      bool aWaitKey = true, const bool aDestroy = true) const;

    // Image (pre-)processing
    void preprocessImages();

    // Filtering process
    void performRadarFiltering(FilteringAlgorithm aFilterAlgo, const size_t aK,
                               const double aZmin, const bool aClearOld = true);

    void getTopK(const uint8_t *aIntensities, const size_t aSize, const size_t aK,
                 std::vector<ValueIndexPair> &aTopKVec) const;
    void performKStrong(const size_t aK, const double aZmin,
                        const bool aClearOld = true);
    const FilteredPointsVec &getFilteredPoints() const;

    // Generating Oriented Surface Points
    // NOTE: Implemented in OrientedSurfacePointsHandler.cpp
    void clearORSPInfo();
    void downsamplePointCloud();
    void findValidNeighbours(Point2DList &aValidNeighbours, const size_t aGridX,
                             const size_t aGridY);
    void estimatePointDistribution();
    void computeOrientedSurfacePoints();

    const ORSPVec<double> &getORSPFeaturePoints() const;

    // Motion Undistortion
    void performMotionUndistortion(const Pose2D<double> &aVelocity,
                                   const VectorXT<double> &aTimeVector);

    // NOTE: UNUSED
    void performFFTOnImage(ImageType &aSrcImageType, cv::Mat &aDestImage);
    void performFFTOnImage(const cv::Mat &aSrcImage, cv::Mat &aDestImage);
};

#endif // __RADAR_IMAGE_2D_HPP__