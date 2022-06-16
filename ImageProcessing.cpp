/**
 * @file ImageProcessing.cpp
 * @brief Contains all the functions needed for image processing, such as FFT
 * and FMT
 * @see https://github.com/Smorodov/LogPolarFFTTemplateMatcher
 */
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"

#include "ImageProcessing.hpp"

/**
 * @brief Radians to Degrees
 * @param[in] aAngleRad
 * @return Value of angle in degrees
 */
double radToDeg(double aAngleRad) {
    return aAngleRad * RAD_TO_DEG;
}

/**
 * @brief Recombine image sectors.
 * ? Seems to only be used for visualisation
 *
 * @param src Source Mat
 * @param dst Destination Mat
 */
void recombineSectors(const cv::Mat &src, cv::Mat &dst) {
    int cx = src.cols >> 1;
    int cy = src.rows >> 1;
    cv::Mat tmp;
    tmp.create(src.size(), src.type());
    src(cv::Rect(0, 0, cx, cy)).copyTo(tmp(cv::Rect(cx, cy, cx, cy)));
    src(cv::Rect(cx, cy, cx, cy)).copyTo(tmp(cv::Rect(0, 0, cx, cy)));
    src(cv::Rect(cx, 0, cx, cy)).copyTo(tmp(cv::Rect(0, cy, cx, cy)));
    src(cv::Rect(0, cy, cx, cy)).copyTo(tmp(cv::Rect(cx, 0, cx, cy)));
    dst = tmp;
}

/**
 * @brief Performs FFT (discrete, 2D) on source Mat image
 *
 * @param Src Source Mat image
 * @param FImg Pointer to 2D array containing planes (output)
 *              ? What is in the array
 * @param do_recomb Performs recombination of planes (likely visualisation only)
 *                  @see recombineSectors()
 *
 * @see InverseFFT()
 *
 */
void ForwardFFT(const cv::Mat &Src, cv::Mat *FImg, bool do_recomb) {
    int M = cv::getOptimalDFTSize(Src.rows);
    int N = cv::getOptimalDFTSize(Src.cols);

    cv::Mat padded;
    copyMakeBorder(Src, padded, 0, M - Src.rows, 0, N - Src.cols,
                   cv::BORDER_CONSTANT, cv::Scalar::all(0));
    cv::Mat planes[] = { cv::Mat_<float>(padded),
                         cv::Mat::zeros(padded.size(), CV_32F) };
    cv::Mat complexImg;
    cv::merge(planes, 2, complexImg);
    cv::dft(complexImg, complexImg);
    cv::split(complexImg, planes);

    planes[0] =
        planes[0](cv::Rect(0, 0, planes[0].cols & -2, planes[0].rows & -2));
    planes[1] =
        planes[1](cv::Rect(0, 0, planes[1].cols & -2, planes[1].rows & -2));
    if (do_recomb) {
        recombineSectors(planes[0], planes[0]);
        recombineSectors(planes[1], planes[1]);
    }

    planes[0] /= float(M * N);
    planes[1] /= float(M * N);

    FImg[0] = planes[0].clone();
    FImg[1] = planes[1].clone();
}

/**
 * @brief Perform inverse FFT (discrete, 2D) on FImg planes
 *
 * @param FImg Input FImg planes
 * @param dst Destination Mat image
 * @param do_recomb Performs recombination of planes (likely visualisation only)
 *                  @see recombineSectors()
 *
 * @see ForwardFFT()
 */
void InverseFFT(cv::Mat *FImg, cv::Mat &dst, bool do_recomb) {
    if (do_recomb) {
        recombineSectors(FImg[0], FImg[0]);
        recombineSectors(FImg[1], FImg[1]);
    }
    cv::Mat complexImg;
    cv::merge(FImg, 2, complexImg);
    cv::idft(complexImg, complexImg);
    cv::split(complexImg, FImg);
    dst = FImg[0].clone();
}

/**
 * @brief Performs high pass filter operation on input image
 * @note Currently using OpenCV's native Laplacian filter
 * @deprecated
 *
 * @param[in] aSrcImg Source image to perform filter on
 * @param[out] aDestImg Destination image (high-pass filtered)
 * @param[in] aKernelSize kernel size
 */
void performHighpass(const cv::Mat &aSrcImg, cv::Mat &aDestImg,
                     int aKernelSize) {
    cv::Laplacian(aSrcImg, aDestImg, aSrcImg.depth(), aKernelSize);
}

/**
 * @brief Efficient image rotation for 90deg multiples.
 * Used as microoptimsation for performImageRotation()
 * @see performImageRotation()
 *
 * @param[in] aSrcImg Source image to rotate
 * @param[out] aDestImg Destination image to rotate
 * @param[in] rotFlag One of the following rotation flags
 *            @arg @c 1 90 deg CW
 *            @arg @c 2 90 deg CCW
 *            @arg @c 3 180 deg rotation
 */
void rot90(const cv::Mat &aSrcImg, cv::Mat &aDestImg, int rotFlag) {
    // 1=CW, 2=CCW, 3=180
    if (rotFlag == 1) {
        transpose(aSrcImg, aDestImg);
        flip(aSrcImg, aDestImg, 1);
    }
    else if (rotFlag == 2) {
        transpose(aSrcImg, aDestImg);
        flip(aSrcImg, aDestImg, 0);
    }
    else if (rotFlag == 3) {
        flip(aSrcImg, aDestImg, -1);
    }
}

/**
 * @brief Performs image rotation. Currently using OpenCV warpAffine.
 * @todo Change to some other implementation
 * @param[in] aSrcImg Source image to rotate
 * @param[out] aDestImg Destination image to rotate
 * @param[in] aAngleRad Angle to rotate in radians
 */
void performImageRotation(const cv::Mat &aSrcImg, cv::Mat &aDestImg,
                          const double aAngleRad) {
    double angleDeg = radToDeg(aAngleRad);

    cv::Point2d centerPoint(aSrcImg.cols * 0.5, aSrcImg.rows * 0.5);
    cv::Mat rotMat = cv::getRotationMatrix2D(centerPoint, angleDeg, 1.0);

    // TODO: Apparently it's too rough?
    int interpolation = cv::INTER_LINEAR;
    cv::warpAffine(aSrcImg, aDestImg, rotMat, aSrcImg.size(), interpolation);
}

/**
 * @brief Generates high pass filter for freq domain. Used for filtering in
 * phase correlation step.
 * @note IDEAL high pass filter (0, 1); May want to refine this
 * @see phaseCorrelate2()
 * @deprecated Unused
 *
 * @param[out] aDestFilter Destination matrix (the filter). Note that it needs
 * to be the size of image matrix it is filtering.
 * @param[in] aFilterSize Size of the filter. Larger values mean stronger
 * filtering
 */
void genFFTHighpassFilter(cv::Mat &aDestFilter, double aFilterSize) {
    assert(aDestFilter.rows > 0 && aDestFilter.cols > 0);

    cv::Point centre(aDestFilter.rows / 2, aDestFilter.cols / 2);
    double radius;

    for (int i = 0; i < aDestFilter.rows; i++) {
        for (int j = 0; j < aDestFilter.cols; j++) {
            radius = (double)sqrt(pow((i - centre.x), 2.0) +
                                  pow((double)(j - centre.y), 2.0));
            if (radius > aFilterSize) {
                aDestFilter.at<float>(i, j) = (float)1;
            }
            else {
                aDestFilter.at<float>(i, j) = (float)0;
            }
        }
    }
}

/**
 * @brief Phase correlate function
 * @note Modified from OpenCV source code. Now includes high pass filter and
 * hanning window by default.
 * @deprecated Unused. Now returns a Point(0,0)
 *
 * @param _src1 Source image 1.
 * @param _src2 Source image 2.
 * @param aFilterSize Filter size for high-pass filter.
 * @param aDisplayFilter Flag of whether or not to display filter.
 *
 * @see
 * https://github.com/opencv/opencv/blob/master/modules/imgproc/src/phasecorr.cpp
 * @return Point with maximum peak location
 */

cv::Point2d phaseCorrelate2(cv::InputArray _src1, cv::InputArray _src2,
                            double aFilterSize, bool aDisplayFilter) {
    return cv::Point2d(0,0);
}