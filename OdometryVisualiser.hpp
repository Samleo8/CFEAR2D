/**
 * @file OdometryVisualiser.hpp
 *
 * @brief OdometryVisualiser class that helps visualise odometry path. Contains
 * all the necessary functions for displaying, waiting for key, drawing
 * points/lines, and more.
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#ifndef __ODOMETRY_VISUALISER_H__
#define __ODOMETRY_VISUALISER_H__

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/opencv.hpp>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#include "ImageProcessing.hpp"  // needed for Fourier transforms
#include "RadarFeedHandler.hpp" // needed for feed to visualise
#include "RadarImage.hpp"

// Color library
#include "CVColor.hpp"

/**
 * @brief OdometryVisualiser class that contains all the necessary functions for
 * displaying, waiting for key, drawing points/lines, and more.
 */
class OdometryVisualiser {
    // Global Constants
  public:
    // Common Keycodes
    static const unsigned char KEYCODE_BACKSPACE = 8; ///< Keycode for backspace
    static const unsigned char KEYCODE_TAB = 9;       ///< Keycode for tab key
    static const unsigned char KEYCODE_ENTER = 13;    ///< Keycode for enter key
    static const unsigned char KEYCODE_ESC = 27;      ///< Keycode for esc key
    static const unsigned char KEYCODE_ESCAPE = KEYCODE_ESC; ///< Alias for esc
    static const unsigned char KEYCODE_SPACE = 32; ///< Keycode for space key
    static const unsigned char KEYCODE_LEFT_ARROW =
        37; ///< Keycode for left arrow
    static const unsigned char KEYCODE_UP_ARROW = 38; ///< Keycode for up arrow
    static const unsigned char KEYCODE_RIGHT_ARROW =
        39; ///< Keycode for right arrow
    static const unsigned char KEYCODE_DOWN_ARROW =
        40;                                          ///< Keycode for down arrow
    static const unsigned char KEYCODE_DELETE = 127; ///< Keycode for del key

    /**
     * @brief Prediction type. Used for indicating line and point colors.
     * @note Always ends with NUM_PRED_TYPES to tell how many elements in enum
     */
    enum PredictionType : size_t {
        GROUND_TRUTH = 0,
        ODOMETRY_PREDICTION,
        ODOMETRY_PREDICTION_KEYFRAMES,
        NUM_PRED_TYPES
    };

    /** @brief Color Class */
    CVColor Color;

  private:
    /** @brief Canvas as OpenCV Mat */
    cv::Mat mCanvas;

    /** 
     * @brief Canvas Width 
     * @note Excludes negative padding
     */
    unsigned int mCanvasWidth = 0;

    /** 
     * @brief Canvas Height 
     * @note Excludes negative padding
     */
    unsigned int mCanvasHeight = 0;

    /** @brief Canvas background color */
    cv::Scalar mBackgroundColor = Color.white;

    /** @brief How much to pad a point by when resizing canvas */
    static const int mPadSpace = 50;

    /** @brief Border around canvas edges specifying when to extend canvas when
     * point falls within it */
    static const int mBoundsBorder = 5;

    /** @brief How much canvas has extended in the negative direction */
    cv::Point2d mNegativeDelta = cv::Point2d(0.0, 0.0);

    /** @brief Title of visualiser */
    std::string mTitle;

    /** @brief Thickness of line */
    int mLineThickness = 1;

    /** @brief Line type (OpenCV) */
    static const int mLineType = cv::LINE_8;

    /**
     * @brief Size of point
     * @note Point is a circle, so this denotes circle radius
     */
    static const int mPointSize = 2;

    /** @brief Number of pred types */
    static const size_t mNumPredTypes =
        static_cast<size_t>(PredictionType::NUM_PRED_TYPES);

    /**
     * @brief Line (and, for now, point) colors
     * @note BGR Format
     */
    cv::Scalar mLineColors[mNumPredTypes] = { Color.forestgreen,
                                              Color.orangered,
                                              Color.deepskyblue };

    // Safety Checks
    bool pointWithinBounds(const int ax, const int ay, const int aBorder = 0);
    bool pointWithinBounds(const cv::Point2d &aPoint, const int aBorder = 0);

    // Helper function for wait key
    void keyCodeToDisplayString(const unsigned char aKeyCode,
                                std::string &aDisplayString);

  public:
    // Constructors, destructors and init
    OdometryVisualiser();
    OdometryVisualiser(const unsigned int aCanvasWidth,
                       const unsigned int aCanvasHeight,
                       const bool aDisplay = true,
                       const std::string &aDisplayTitle = "Visualiser");

    void init(const unsigned int aCanvasWidth, const unsigned int aCanvasHeight,
              const bool aDisplay = true,
              const std::string &aDisplayTitle = "Visualiser");

    // Get/Set params of visualiser
    const cv::Scalar &getLineColor(PredictionType aPredType);
    void setLineColor(PredictionType aType, cv::Scalar &aLineColor);

    const cv::Scalar &getPointColor(PredictionType aPredType);
    void setPointColor(PredictionType aType, cv::Scalar &aLineColor);

    const cv::Scalar &getBackgroundColor();
    void setBackgroundColor(const cv::Scalar &aBackgroundColor);

    const int getLineThickness();
    void setLineColor(int aLineThickness);

    const cv::Mat &getCanvas();
    const unsigned int getCanvasWidth();
    const unsigned int getCanvasHeight();
    const unsigned int getFullCanvasWidth();
    const unsigned int getFullCanvasHeight();

    const cv::Point2d &getNegativeDelta();

    const std::string &getTitle();
    void setTitle(const std::string &aTitle);

    // Drawing functions
    void extendCanvasIfNeeded(const cv::Point2d &aCoord);

    void drawLine(const cv::Point2d &aStart, const cv::Point2d &aEnd,
                  const cv::Scalar &aColor);
    void drawLine(const int aStartX, const int aStartY, const int aEndX,
                  const int aEndY, const cv::Scalar &aColor);

    void drawPoint(const cv::Point2d &aCoord, const cv::Scalar &aColor);
    void drawPoint(const int ax, const int ay, const cv::Scalar &aColor);

    void drawPose(const cv::Point2d &aPoint, const double aYaw,
                  const cv::Scalar &aColor);
    void drawPose(const cv::Point3d &aPose, const cv::Scalar &aColor);
    void drawPose(const int ax, const int ay, const double aYaw,
                  const cv::Scalar &aColor);

    void writeText(const std::string &aText, const int ax, const int ay,
                   const cv::Scalar &aColor, const double aFontScale = 0.5);
    void writeText(const std::string &aText, const cv::Point2d &aCoord,
                   const cv::Scalar &aColor, const double aFontScale = 0.5);

    bool displayMessage(const std::string &aText,
                        const int aDisplayTime = 3000);

    // Display and Controls
    void display();
    void resize(const unsigned int aWidth, const unsigned int aHeight);
    void pad(const unsigned int aPadX, const unsigned int aPadY);
    void padNegative(const unsigned int aPadX, const unsigned int aPadY);
    void clear();

    unsigned char delay(unsigned int aTimeMS = 100);
    unsigned char waitForKeypress(unsigned char aKeyCode = 0);
    unsigned char waitForKeypress(unsigned char aKeyCodes[], size_t n);
};

/** @brief Typedef to OdometryVisualiser for convenience */
typedef OdometryVisualiser OdoVis;

/** @brief Typedef to OdometryVisualiser for convenience */
typedef OdometryVisualiser OdomVis;

#endif