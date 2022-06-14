/**
 * @file OdometryVisualiser.cpp
 *
 * @brief OdometryVisualiser class that helps visualise odometry path. Contains
 * all the necessary functions for displaying, waiting for key, drawing
 * points/lines, and more.
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#include "OdometryVisualiser.hpp"

/*******************************************
 * @section OdomVis-InitConst Constructors & Init
 *******************************************************/

/**
 * @brief Defafult empty constructor for Odometry Visualiser
 */
OdometryVisualiser::OdometryVisualiser() {}

/**
 * @brief Constructor for Odometry Visualiser
 * @see init()
 *
 * @param[in] aCanvasWidth Width of canvas
 * @param[in] aCanvasHeight Height of canvas
 * @param[in] aDisplay Choose whether or not to display on initialisation
 * @param[in] aDisplayTitle Title of visualiser display window
 */
OdometryVisualiser::OdometryVisualiser(const unsigned int aCanvasWidth,
                                       const unsigned int aCanvasHeight,
                                       const bool aDisplay,
                                       const std::string &aDisplayTitle) {
    init(aCanvasWidth, aCanvasHeight, aDisplay, aDisplayTitle);
}

/**
 * @brief Initialises the visualiser by setting title and canvas
 * @param[in] aCanvasWidth Width of canvas
 * @param[in] aCanvasHeight Height of canvas
 * @param[in] aDisplay Choose whether or not to display on initialisation
 * @param[in] aDisplayTitle Title of visualiser display window
 */
void OdometryVisualiser::init(const unsigned int aCanvasWidth,
                              const unsigned int aCanvasHeight,
                              const bool aDisplay,
                              const std::string &aDisplayTitle) {
    mTitle = aDisplayTitle;

    // Clear canvas
    mCanvasWidth = aCanvasWidth;
    mCanvasHeight = aCanvasHeight;
    clear();

    if (aDisplay) display();
}

/*******************************************************
 * @section OdomVis-GetterSetters Get/Set params
 *******************************************************/
/**
 * @brief Get line color
 * @param[in] aPredType Prediction type (enum)
 * @return Line color of specific prediction type in BGR
 */
const cv::Scalar &OdometryVisualiser::getLineColor(PredictionType aPredType) {
    return mLineColors[static_cast<size_t>(aPredType)];
}

/**
 * @brief Set line color
 * @param[in] aPredType Prediction type (enum)
 * @param[in] aLineColor Scalar of line color in BGR
 */
void OdometryVisualiser::setLineColor(PredictionType aPredType,
                                      cv::Scalar &aLineColor) {
    mLineColors[static_cast<size_t>(aPredType)] = aLineColor;
}

/**
 * @brief Get ppint color
 * @param[in] aPredType Prediction type (enum)
 * @note For now same as line color
 * @return Line color of specific prediction type in BGR
 */
const cv::Scalar &OdometryVisualiser::getPointColor(PredictionType aPredType) {
    return mLineColors[static_cast<size_t>(aPredType)];
}

/**
 * @brief Set point color
 * @param[in] aPredType Prediction type (enum)
 * @param[in] aPointColor Scalar of point color in BGR
 * @note For now same as line color
 */
void OdometryVisualiser::setPointColor(PredictionType aPredType,
                                       cv::Scalar &aPointColor) {
    mLineColors[static_cast<size_t>(aPredType)] = aPointColor;
}

/**
 * @brief Get background color
 * @return Background color
 */
const cv::Scalar &OdometryVisualiser::getBackgroundColor() {
    return mBackgroundColor;
}

/**
 * @brief Set background color
 * @param[in] aBackgroundColor Background color to set
 */
void OdometryVisualiser::setBackgroundColor(
    const cv::Scalar &aBackgroundColor) {
    mBackgroundColor = aBackgroundColor;
}

/**
 * @brief Get line thickness
 * @return Line thickness
 */
const int OdometryVisualiser::getLineThickness() {
    return mLineThickness;
}

/**
 * @brief Set line thickness
 * @param[in] aLineThickness Scalar of line color in RGB
 */
void OdometryVisualiser::setLineColor(int aLineThickness) {
    mLineThickness = aLineThickness;
}

/**
 * @brief Get canvas object
 * @return Reference to cv::Mat canvas
 */
const cv::Mat &OdometryVisualiser::getCanvas() {
    return mCanvas;
}

/**
 * @brief Get canvas width
 * @note Excludes negative padding
 * @see getFullCanvasWidth()
 * @return Canvas width
 */
const unsigned int OdometryVisualiser::getCanvasWidth() {
    return mCanvasWidth;
}

/**
 * @brief Get canvas height
 * @note Excludes negative padding
 * @see getFullCanvasHeight()
 * @return Canvas height
 */
const unsigned int OdometryVisualiser::getCanvasHeight() {
    return mCanvasHeight;
}

/**
 * @brief Get actual canvas width
 * @note Includes negative padding
 * @see getCanvasWidth()
 * @return Canvas width
 */
const unsigned int OdometryVisualiser::getFullCanvasWidth() {
    return mCanvasWidth + mNegativeDelta.x;
}

/**
 * @brief Get canvas height
 * @note Includes negative padding
 * @see getCanvasHeight()
 *
 * @return Canvas height with negative padding
 */
const unsigned int OdometryVisualiser::getFullCanvasHeight() {
    return mCanvasHeight + mNegativeDelta.y;
}

/**
 * @brief Get canvas title
 * @return Title of visualiser/canvas
 */
const std::string &OdometryVisualiser::getTitle() {
    return mTitle;
}

/**
 * @brief Set canvas title
 * @param[out] aTitle Title of visualiser/canvas to set
 */
void OdometryVisualiser::setTitle(const std::string &aTitle) {
    mTitle = aTitle;
}

/**
 * @brief Get how much canvas has extended in the negative direction
 * @note Used for negative coordinates support
 * @return Delta in negative direction
 */
const cv::Point2d &OdometryVisualiser::getNegativeDelta() {
    return mNegativeDelta;
}

/*******************************************************
 * @section OdomVis-Drawing Drawing functions
 *******************************************************/
/**
 * @brief Checks whether point is within canvas bounds
 * @note Private helper function
 * @param[in] ax x-coordinate of point to check
 * @param[in] ay y-coordinate of point to check
 * @param[in] aBorder Size of border from canvas edge that we also want to count
 *                    as out of bounds
 * @return Whether point is within canvas bounds
 */
bool OdometryVisualiser::pointWithinBounds(const int ax, const int ay,
                                           const int aBorder) {
    return (aBorder >= 0 && aBorder <= ax &&
            (unsigned)ax <= mCanvasWidth - aBorder && aBorder <= ay &&
            (unsigned)ay <= mCanvasHeight - aBorder);
}

/**
 * @brief Checks whether point is within canvas bounds
 * @note Private helper function
 * @param[in] aPoint Point to check
 * @param[in] aBorder Size of border from canvas edge that we also want to count
 *                    as out of bounds
 * @return Whether point is within canvas bounds
 */
bool OdometryVisualiser::pointWithinBounds(const cv::Point2d &aPoint,
                                           const int aBorder) {
    return pointWithinBounds(aPoint.x, aPoint.y, aBorder);
}

/**
 * @brief Given a point, extends the canvas in either the positive or negative
 * direction, if not within bounds. Updates the canvas width/height accordingly,
 * as well as updates the @a mNegativeDelta member variable if necessary to
 * indicate how much points need to shift.
 *
 * @param[in] aCoord Coordinate of point (to draw etc.)
 *  */
void OdometryVisualiser::extendCanvasIfNeeded(const cv::Point2d &aCoord) {
    // Default delta = no change
    cv::Point2d padNegDelta(0.0, 0.0);

    // Pad in negative direction if we have to
    if (aCoord.x + mNegativeDelta.x < 0) {
        padNegDelta.x = -(aCoord.x + mNegativeDelta.x) + mPadSpace;
    }

    if (aCoord.y + mNegativeDelta.y < 0) {
        padNegDelta.y = -(aCoord.y + mNegativeDelta.y) + mPadSpace;
    }

    // Extend canvas if need be. Updates canvas height/width also
    padNegative(padNegDelta.x, padNegDelta.y);

    int newWidth = mCanvasWidth, newHeight = mCanvasHeight;

    // Extend canvas if not within bounds
    if (!pointWithinBounds(aCoord, mBoundsBorder)) {
        newWidth = MAX(aCoord.x + mPadSpace, newWidth);
        newHeight = MAX(aCoord.y + mPadSpace, newHeight);
    }

    if (newWidth != mCanvasWidth || newHeight != mCanvasHeight) {
        resize(newWidth, newHeight);
    }
}

/**
 * @brief Draws a line on the canvas, resizing canvas if necessary.
 *
 * @param[in] aStart Starting coordinate
 * @param[in] aEnd Ending coordinate
 * @param[in] aColor Color of line
 *  */
void OdometryVisualiser::drawLine(const cv::Point2d &aStart,
                                  const cv::Point2d &aEnd,
                                  const cv::Scalar &aColor) {
    extendCanvasIfNeeded(aStart);
    extendCanvasIfNeeded(aEnd);

    cv::line(mCanvas, aStart + mNegativeDelta, aEnd + mNegativeDelta, aColor,
             mLineThickness, mLineType);
}

/**
 * @brief Draws a line on the canvas, resizing canvas if necessary.
 *
 * @param[in] aStartX Starting X coordinate
 * @param[in] aStartY Starting Y coordinate
 * @param[in] aEndX Ending x coordinate
 * @param[in] aEndY Ending y coordinate
 * @param[in] aColor Color of line
 *  */
void OdometryVisualiser::drawLine(const int aStartX, const int aStartY,
                                  const int aEndX, const int aEndY,
                                  const cv::Scalar &aColor) {
    drawLine(cv::Point2d(aStartX, aStartY), cv::Point2d(aEndX, aEndY), aColor);
}

/**
 * @brief Draws point on canvas at specified coordinate. Will extend canvas if
 * necessary. If extension is done in the negative direction, will also updates
 * the member variable @a mNegativeDelta
 * @note Actually a small circle
 *
 * @param[in] aCoord Coorindate of point
 * @param[in] aColor Color of point
 *  */
void OdometryVisualiser::drawPoint(const cv::Point2d &aCoord,
                                   const cv::Scalar &aColor) {
    extendCanvasIfNeeded(aCoord);

    cv::circle(mCanvas, aCoord + mNegativeDelta, mPointSize, aColor, cv::FILLED,
               mLineType);
    return;
}

/**
 * @brief Draws point on canvas at specified coordinate. Will extend canvas if
 * necessary. If extension is done in the negative direction, will also update
 * the shift in the member variable @a mNegativeDelta
 * @note Actually a small circle
 *
 * @param[in] ax x-coordinate of point
 * @param[in] ay y-coorindate of point
 * @param[in] aColor Color of point
 *  */
void OdometryVisualiser::drawPoint(const int ax, const int ay,
                                   const cv::Scalar &aColor) {
    drawPoint(cv::Point2d(ax, ay), aColor);
}

/**
 * @brief Rotate 2D vector/point about the (0,0) origin without using @c Eigen
 * rotation matrix
 *
 * @param[in,out] aPoint Point to rotate
 * @param[in] aAngleRad Angle to rotate
 */
void rotateVector2D(cv::Point2d &aPoint, const double aAngleRad) {
    // TODO: this
    const double x = aPoint.x;
    const double y = aPoint.y;
    const double cosA = cos(aAngleRad);
    const double sinA = sin(aAngleRad);

    aPoint.x = x * cosA - y * sinA;
    aPoint.y = x * sinA + y * cosA;
}

/**
 * @brief Draws a rotated triangle on canvas at specified center coordinate, and
 * appropriate orientation.
 *
 * Will extend canvas if necessary. If extension is done in the negative
 * direction, will also report back the shift in all the points in the member
 * variable @a mNegativeDelta
 *
 * @param[in] aPoint Point in the form @a <x,y>
 * @param[in] aColor Color of point
 *  */
void OdometryVisualiser::drawPose(const cv::Point2d &aPoint, const double aYaw,
                                  const cv::Scalar &aColor) {
    // Now build base triangle pointing in positive x-direction
    const int triWidth = 10;
    const int triHeight = 5;

    const size_t N_VERTICES = 3;
    cv::Point2d vertices[N_VERTICES];

    vertices[0] = aPoint + cv::Point2d(triWidth / 2, 0); // tip
    vertices[1] =
        aPoint - cv::Point2d(triWidth / 2, triHeight / 2); // top-left corner
    vertices[2] = aPoint - cv::Point2d(triWidth / 2,
                                       -triHeight / 2); // bottom-left corner

    // Rotate all vertices using yaw direction, with centerPoint as origin
    for (int i = 0; i < N_VERTICES; i++) {
        cv::Point2d delta = vertices[i] - aPoint;
        rotateVector2D(delta, aYaw);

        vertices[i] = aPoint + delta;
    }

    // Draw triangle
    for (size_t i = 0; i < N_VERTICES; i++) {
        drawLine(vertices[i], vertices[(i + 1) % N_VERTICES], aColor);
    }
}

/**
 * @brief Draws a rotated triangle on canvas at specified center coordinate, and
 * appropriate orientation.
 *
 * Will extend canvas if necessary. If extension is done in the negative
 * direction, will also report back the shift in all the points in the member
 * variable @a mNegativeDelta
 *
 * @param[in] ax x-coordinate of pose
 * @param[in] ay y-coorindate of pose
 * @param[in] aYaw yaw of pose
 * @param[in] aColor Color of rectangle line
 *  */
void OdometryVisualiser::drawPose(const int ax, const int ay, const double aYaw,
                                  const cv::Scalar &aColor) {
    drawPose(cv::Point2d(ax, ay), aYaw, aColor);
}

/**
 * @brief Draws a rotated triangle on canvas at specified center coordinate, and
 * appropriate orientation.
 *
 * Will extend canvas if necessary. If extension is done in the negative
 * direction, will also report back the shift in all the points in the member
 * variable @a mNegativeDelta
 *
 * @param[in] aPose Pose in the form @a <x,y,yaw>
 * @param[in] aColor Color of point
 *  */
void OdometryVisualiser::drawPose(const cv::Point3d &aPose,
                                  const cv::Scalar &aColor) {
    drawPose(aPose.x, aPose.y, aPose.z, aColor);
}

/**
 * @brief Displays text on screen using QT overlay
 *
 * @note Different from writeText(), which writes text on canvas
 *
 * @param[in] aText Text to display
 * @param[in] aDisplayTime Display time in ms before disappearing
 *
 * @return True if success, false if no QT support
 */
bool OdometryVisualiser::displayMessage(const std::string &aText,
                                        const int aDisplayTime) {
    try {
        cv::displayOverlay(mTitle, aText, aDisplayTime);
        return true;
    } catch (const std::exception &) {
        return false;
    }

    return false;
}

/**
 * @brief Writes text on canvas at specified coordinate and color. Handles
 * multiline strings.
 *
 * @param[in] aText Text to write
 * @param[in] aCoord Coordinate of text (top left corner)
 * @param[in] aColor Color of text
 * @param[in] aFontScale Font scale
 */
void OdometryVisualiser::writeText(const std::string &aText,
                                   const cv::Point2d &aCoord,
                                   const cv::Scalar &aColor,
                                   const double aFontScale) {
    writeText(aText, aCoord.x, aCoord.y, aColor);
}

/**
 * @brief Writes text on canvas at specified coordinate and color. Handles
 * multiline strings.
 * @note Defaults to (0,0) origin if out of bounds coordinate
 *
 * @param[in] aText Text to write
 * @param[in] ax x-coordinate of text (top left corner)
 * @param[in] ay y-coorindate of text (top left corner)
 * @param[in] aColor Color of text
 * @param[in] aFontScale Font scale
 */
void OdometryVisualiser::writeText(const std::string &aText, const int ax,
                                   const int ay, const cv::Scalar &aColor,
                                   const double aFontScale) {
    int x, y;

    if (pointWithinBounds(ax, ay)) {
        x = ax;
        y = ay;
    }
    else {
        x = 0;
        y = 0;
    }

    const int newlineSpace = 50 * aFontScale;

    std::stringstream ss(aText);
    std::string token;
    while (std::getline(ss, token)) {
        cv::putText(mCanvas, token, cv::Point2d(x, y), cv::FONT_HERSHEY_SIMPLEX,
                    aFontScale, aColor, 1);
        y += newlineSpace;
    }
}

/*******************************************************
 * @section OdomVis-Display
 *******************************************************/

/**
 * @brief Display the canvas in an cv::imshow window
 * @note The title is the unique ID of the window
 * @todo Add to some list if adding to title
 */
void OdometryVisualiser::display() {
    // Having GUI normal clears away all extra options like save, zoom etc
    int flags = cv::WINDOW_AUTOSIZE; // + cv::WINDOW_GUI_NORMAL;

    cv::namedWindow(mTitle, flags);
    cv::imshow(mTitle, mCanvas);
    return;
}

/**
 * @brief Resize the canvas
 * @note May truncate the image
 * @note Excludes negative padding
 * @param[in] aWidth Width to resize to
 * @param[in] aHeight Height to resize to
 */
void OdometryVisualiser::resize(const unsigned int aWidth,
                                const unsigned int aHeight) {
    int padX = aWidth - getCanvasWidth();
    int padY = aHeight - getCanvasHeight();

    // Truncation (rows first cos easier)
    if (padY < 0) {
        mCanvas.pop_back(ABS(padY));
        mCanvasHeight += padY;
        padY = 0;
    }

    if (padX < 0) {
        mCanvas = mCanvas.colRange(0, aWidth);
        mCanvasWidth += padX;
        padX = 0;
    }

    pad(padX, padY);
}

/**
 * @brief Pad the canvas with rows and cols
 * @param[in] aPadX Number of cols to pad by
 * @param[in] aPadY Number of rows to pad by
 */
void OdometryVisualiser::pad(const unsigned int aPadX,
                             const unsigned int aPadY) {
    // Add columns first
    if (aPadX) {
        cv::Mat col(getFullCanvasHeight(), aPadX, CV_8UC3, mBackgroundColor);
        cv::hconcat(mCanvas, col, mCanvas);
        mCanvasWidth += aPadX;
    }

    // Then add rows (easier operation)
    if (aPadY) {
        cv::Mat row(aPadY, getFullCanvasWidth(), CV_8UC3, mBackgroundColor);
        mCanvas.push_back(row);
        mCanvasHeight += aPadY;
    }

    return;
}

/**
 * @brief Pad the canvas with rows and cols. but in the "negative direction", ie
 * upwards and leftwards, using cv::copyMakeBorder()
 * @note Does NOT update canvas width and height, only @a mNegativeDelta
 * @param[in] aPadX Number of cols to pad by
 * @param[in] aPadY Number of rows to pad by
 */
void OdometryVisualiser::padNegative(const unsigned int aPadX,
                                     const unsigned int aPadY) {
    // Zeros: Do nothing
    if (!aPadX && !aPadY) return;

    // Copy make border to extend canvas in "negative direction"
    cv::copyMakeBorder(mCanvas, mCanvas, aPadY, 0, aPadX, 0,
                       cv::BORDER_CONSTANT, mBackgroundColor);

    // Remember to update negative delta
    mNegativeDelta.x += aPadX;
    mNegativeDelta.y += aPadY;
    return;
}

/**
 * @brief Clear everything on the canvas
 * @note Currently white background, perhaps add background image support
 */
void OdometryVisualiser::clear() {
    mCanvas = cv::Mat(getFullCanvasHeight(), getFullCanvasWidth(), CV_8UC3,
                      mBackgroundColor);

    return;
}

/***************************************************
 * @section OdomVis-Controls Controls
 ***************************************************/

/**
 * @brief Delay display
 * @param[in] aTimeMS Delay time in ms
 * @note If aTimeMS is 0, will wait indefinitely until keypresss
 * @return Key code of keypress, if any
 */
unsigned char OdometryVisualiser::delay(unsigned int aTimeMS) {
    return (unsigned char)(unsigned int)cv::waitKey(aTimeMS);
}

/**
 * @brief Simple helper function used to display keycode in a human-readable
 * format. For example "Space", "Esc"
 * @param[in] aKeyCode Keycode
 * @param[out] aDisplayString Display string (eg "Space")
 */
void OdometryVisualiser::keyCodeToDisplayString(const unsigned char aKeyCode,
                                                std::string &aDisplayString) {
    switch (aKeyCode) {
        case KEYCODE_BACKSPACE:
            aDisplayString = "Backspace";
            break;
        case KEYCODE_TAB:
            aDisplayString = "Tab";
            break;
        case KEYCODE_ESC:
            aDisplayString = "Esc";
            break;
        case KEYCODE_SPACE:
            aDisplayString = "Space";
            break;
        case KEYCODE_DELETE:
            aDisplayString = "Delete";
            break;
        case KEYCODE_ENTER:
            aDisplayString = "Enter";
            break;
        case KEYCODE_UP_ARROW:
            aDisplayString = "Up Arrow";
            break;
        case KEYCODE_DOWN_ARROW:
            aDisplayString = "Down Arrow";
            break;
        case KEYCODE_LEFT_ARROW:
            aDisplayString = "Left Arrow";
            break;
        case KEYCODE_RIGHT_ARROW:
            aDisplayString = "Right Arrow";
            break;
        default:
            aDisplayString = std::string(1, aKeyCode);
            break;
    }

    return;
}

/**
 * @brief Wait for a keypress before continue
 * @param[in] aKeyCode Key code of the key that we are waiting for.
 *                     0 for any key
 * @note You can use the actual character (eg. 'q'), but it MUST be lower case
 * @note Currently the visualiser tells the user which particular keycode it is
 * waiting for @todo verbosity level
 * @return Key that was pressed
 */
unsigned char OdometryVisualiser::waitForKeypress(unsigned char aKeyCode) {
    // Convert to lower case, just in case
    aKeyCode = tolower(aKeyCode);

    if (aKeyCode) {
        std::string displayString;
        keyCodeToDisplayString(aKeyCode, displayString);
        printf("Awaiting key %s (keycode %u) to be pressed...\n",
               displayString.c_str(), aKeyCode);

        char key;
        while (key = cv::waitKey(0)) {
            if (key == aKeyCode) {
                return key;
            }
        }
    }
    else {
        return cv::waitKey(0);
    }

    return 0;
}

/**
 * @brief Wait for a keypress before continue
 * @param[in] aKeyCodes Array of key codes of any one of keys that we are
 * waiting for
 * @param[in] n Length of char array
 * @note You can use the actual character (eg. 'q'), but it MUST be lower case
 * @note Currently the visualiser tells the user which particular keycode it is
 * waiting for @todo verbosity level
 * @return Key that was pressed
 */
unsigned char OdometryVisualiser::waitForKeypress(unsigned char aKeyCodes[],
                                                  size_t n) {
    size_t i;

    for (i = 0; i < n; i++) {
        unsigned char keyCode = tolower(aKeyCodes[i]);

        std::string displayString;
        keyCodeToDisplayString(keyCode, displayString);
        printf("Awaiting key %s (keycode %u) to be pressed...\n",
               displayString.c_str(), keyCode);
    }

    char key;
    while (key = cv::waitKey(0)) {
        for (i = 0; i < n; i++) {
            unsigned char keyCode = tolower(aKeyCodes[i]);
            if (!keyCode || key == keyCode) {
                return key;
            }
        }
    }

    return 0;
}