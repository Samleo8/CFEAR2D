/**
 * @file ImageProcessing.cpp
 * @brief Contains all the functions needed for image processing, such as FFT
 * and FMT
 * @see https://github.com/Smorodov/LogPolarFFTTemplateMatcher
 */
#include "opencv2/core.hpp"
#include "opencv2/imgproc.hpp"
#include "opencv2/opencv.hpp"
#include "phasecorr.hpp"

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
    cv::Mat src1 = _src1.getMat();
    cv::Mat src2 = _src2.getMat();

    CV_Assert(src1.type() == src2.type());
    CV_Assert(src1.type() == CV_32FC1 || src1.type() == CV_64FC1);
    CV_Assert(src1.size == src2.size);

    int M = cv::getOptimalDFTSize(src1.rows);
    int N = cv::getOptimalDFTSize(src1.cols);

    // Hanning window to reduce edge effects
    const bool applyWindow = true;

    // Check if hanning window has been cached
    cv::Mat hanningWindow;
    const bool hanningWindowSaved = (savedHanningWindowSize == cv::Size(M, N));

    if (applyWindow && !hanningWindowSaved)
        cv::createHanningWindow(hanningWindow, src1.size(), CV_64FC1);

    if (!hanningWindow.empty()) {
        CV_Assert(src1.type() == hanningWindow.type());
        CV_Assert(src1.size == hanningWindow.size);
    }

    cv::Mat padded1, padded2, paddedHanningWindow;

    if (M != src1.rows || N != src1.cols) {
        cv::copyMakeBorder(src1, padded1, 0, M - src1.rows, 0, N - src1.cols,
                           cv::BORDER_CONSTANT, cv::Scalar::all(0));
        cv::copyMakeBorder(src2, padded2, 0, M - src2.rows, 0, N - src2.cols,
                           cv::BORDER_CONSTANT, cv::Scalar::all(0));

        if (applyWindow) {
            if (!hanningWindowSaved && !hanningWindow.empty()) {
                cv::copyMakeBorder(hanningWindow, paddedHanningWindow, 0,
                                   M - hanningWindow.rows, 0,
                                   N - hanningWindow.cols, cv::BORDER_CONSTANT,
                                   cv::Scalar::all(0));
            }
        }
    }
    else {
        padded1 = src1;
        padded2 = src2;
        if (applyWindow && !hanningWindowSaved && !hanningWindow.empty()) {
            paddedHanningWindow = hanningWindow;
        }
    }

    // Apply hanning window to both images if valid
    // Optimisation: Check for cached version of hanning window
    if (applyWindow) {
        if (hanningWindowSaved) {
            cv::multiply(savedHanningWindow, padded1, padded1);
            cv::multiply(savedHanningWindow, padded2, padded2);
        }
        else if (!paddedHanningWindow.empty()) {
            cv::multiply(paddedHanningWindow, padded1, padded1);
            cv::multiply(paddedHanningWindow, padded2, padded2);

            savedHanningWindow = paddedHanningWindow;
            savedHanningWindowSize = paddedHanningWindow.size();
        }
    }

    cv::Mat FFT1, FFT2, P, Pm, C;

    // execute phase correlation equation
    // Reference: http://en.wikipedia.org/wiki/Phase_correlation
    cv::dft(padded1, FFT1, cv::DFT_REAL_OUTPUT);
    cv::dft(padded2, FFT2, cv::DFT_REAL_OUTPUT);

    // Perform high pass filtering here
    // Save and avoid recalculating filter (expensive)
    if (aFilterSize > 0) {
        cv::Mat filter;
        if (aFilterSize == savedFilterSize) {
            cv::mulSpectrums(FFT1, savedFilter, FFT1, 0, false);
            cv::mulSpectrums(FFT2, savedFilter, FFT2, 0, false);
        }
        else {
            filter = cv::Mat::zeros(FFT1.rows, FFT1.cols, CV_64FC1);
            genFFTHighpassFilter(filter, aFilterSize);

            cv::mulSpectrums(FFT1, filter, FFT1, 0, false);
            cv::mulSpectrums(FFT2, filter, FFT2, 0, false);

            savedFilterSize = aFilterSize;
            savedFilter = filter;
        }
    }

    if (aDisplayFilter) {
        cv::Mat display1, display2;
        cv::idft(FFT1, display1);
        cv::idft(FFT2, display2);

        cv::normalize(display1, display1, 0, 1, cv::NORM_MINMAX);
        cv::normalize(display2, display2, 0, 1, cv::NORM_MINMAX);

        cv::imshow("Highpass 1", display1);
        cv::imshow("Highpass 2", display2);

        // cv::normalize(src1, display1, 0, 1, cv::NORM_MINMAX);
        // cv::normalize(src2, display2, 0, 1, cv::NORM_MINMAX);

        // cv::imshow("Display 1", display1);
        // cv::imshow("Display 2", display2);

        cv::waitKey(0);
        cv::destroyAllWindows();
    }

    cv::mulSpectrums(FFT1, FFT2, P, 0, true);

    magSpectrums(P, Pm);
    divSpectrums(
        P, Pm, C, 0,
        false); // FF* / |FF*| (phase correlation equation completed here...)

    cv::idft(C, C); // gives us the nice peak shift location...

    fftShift(C); // shift the energy to the center of the frame.

    // locate the highest peak
    cv::Point peakLoc;
    cv::minMaxLoc(C, NULL, NULL, NULL, &peakLoc);

    // TODO: Play with this
    // get the phase shift with sub-pixel accuracy, 5x5 window seems about right
    // here...
    cv::Point2d t;
    t = weightedCentroid(C, peakLoc, cv::Size(5, 5), 0);

    // TODO: Change?
    // adjust shift relative to image center...
    cv::Point2d center((double)padded1.cols / 2.0, (double)padded1.rows / 2.0);

    return (center - t);
}