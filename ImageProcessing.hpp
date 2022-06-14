#ifndef __IMAGE_PROCESSING_H__
#define __IMAGE_PROCESSING_H__

/**
 * @file ImageProcessing.hpp
 * @brief Contains all the functions needed for image processing, such as FFT
 * and FMT
 * @see https://github.com/Smorodov/LogPolarFFTTemplatecv::Matcher
 */
#include "opencv2/core.hpp"
#include "opencv2/opencv.hpp"
#include <math.h>

/** @brief Multiplier for radian to degree conversion */
#ifndef RAD_TO_DEG
#define RAD_TO_DEG 57.29577951308232
#endif // !RAD_TO_DEG

/**
 * @brief Cached filter size 
 * @see phaseCorrelate2()
 * @note static because global variable, want to scope to this file
 */
static double savedFilterSize = 0;

/**
 * @brief Cached highpass filter
 * @see phaseCorrelate2()
 * @note Avoiding recreation of highpass filter as much as possible because it
 * is an expensive operation
 * @note static because global variable, want to scope to this file
 */
static cv::Mat savedFilter;

/**
 * @brief Cached hanning window size
 * @see phaseCorrelate2()
 */
static cv::Size savedHanningWindowSize = cv::Size(0, 0);

/**
 * @brief Cached hanning window
 * @see phaseCorrelate2()
 * @note Avoiding recreation of hanning window because it is an expensive
 * operation
 * @note static because global variable, want to scope to this file
 */
static cv::Mat savedHanningWindow;

/** @brief recombineSectorsines image sectors */
void recombineSectors(const cv::Mat &src, cv::Mat &dst);

/** 
 * @brief Performs FFT (discrete, 2D) on source cv::Mat image 
 * @see InverseFFT()
 */
void ForwardFFT(const cv::Mat &Src, cv::Mat *FImg, bool do_recomb = false);

/**
 * @brief Perform inverse FFT (discrete, 2D) on FImg planes
 * @see ForwardFFT()
 */
void InverseFFT(cv::Mat *FImg, cv::Mat &dst, bool do_recomb = false);

/**
 * @brief Performs high pass filter operation on input image
 * @note Currently using OpenCV's native Laplacian filter
 * @deprecated
 */
void performHighpass(const cv::Mat &aSrcImg, cv::Mat &aDestImg,
                     int aKernelSize = 3);

/**
 * @brief Performs image rotation. Currently using OpenCV warpAffine.
 */
void performImageRotation(const cv::Mat &aSrcImg, cv::Mat &aDestImg,
                          const double aAngleRad);

/**
 * @brief Generates high pass filter for freq domain. Used for filtering in
 * phase correlation step.
 * @note IDEAL high pass filter (0, 1); May want to refine this
 * @see phaseCorrelate2()
 */
void genFFTHighpassFilter(cv::Mat &aDestFilter, double aFilterSize);

/**
 * @brief Phase correlate function
 * @note Modified from OpenCV source code
 * @see
 * https://github.com/opencv/opencv/blob/master/modules/imgproc/src/phasecorr.cpp
 */
cv::Point2d phaseCorrelate2(cv::InputArray _src1, cv::InputArray _src2,
                            double aFilterSize = -1,
                            bool aDisplayFilter = false);

#endif