/**
 * @file PoseGraph.cpp
 * @brief Basic pose graph class that contains the nodes and edges structs. Also
 * allows for optimisations to be performed on it.
 *
 * @note Meant to be used with RadarFeed class
 * @see RadarFeed.cpp
 * @see PoseGraphHandler.cpp
 *
 * @note The (local) pose graph contains N+1 nodes, where N is the number of
 * elements in the sliding window. Every element of the sliding window is a node
 * in the pose graph, with the extra node being the previous keyframe. The
 * previous keyframe node is considered the "origin node", and has its
 * information/value held constant at (0,0). The origin node is implicitly in
 * the pose graph, and cannot be explicitly found in the PoseGraph data
 * structure.
 *
 * Each node has (x,y) point information, that is optimised at each iteration.
 * The outputs of the final graph may eventually be outputted to the visualiser
 * for visualisation purposes
 *
 * Each node has exactly 2 edges, corresponding to the following factors
 * 1) Odometry Factor: Translation data between current node and previous node,
 *                     where the nodes are consecutive in time. It may be
 *                     possible that there are missing "nodes" in between due
 *                     to the exclusion of nodes from the valid set.
 * 2) Heading Factor: Rotation data between current node and origin node
 *                    (the previous keyframe).
 *
 * @note The SlidingWindow data structure basically contains the edge
 * information of each node
 * @see SlidingWindow, SlidingWindowData
 * @see PoseGraphEdges, PoseGraphNodes, PoseGraphNodeData
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#include "PoseGraph.hpp"

/**
 ************************************************************************
 * @section PoseGraph-GetterSetter Constructors, Getters/Setters
 ************************************************************************
 */

/**
 * @brief Constructor for pose graph class. Uses C++ list initialisers to
 * initialise const reference to edges (i.e. the sliding window) and the max
 * size of the graph
 * @note Since multiple residual blocks need to be added to the graph, aOptions
 * should have enable_fast_removal set to true.
 * @todo Possible to ignore this because of small graph size?
 *
 * @param[in] aInitialKeyframe Initial Keyframe (very first origin node)
 * @param[in] aMaxSize The maximum size of the graph (excluding the implicit
 * origin.
 * @param[in] aFilterSize Filter size used for odometry. @see
 * DEFAULT_FILTER_SIZE
 * @param[in] aOptions Options for Ceres problem. Needed for initialisation.
 * Best to set enable_fast_removal to true. node/previous keyframe). Same as the
 * size of the sliding window.
 */
PoseGraph::PoseGraph(const KeyframeNode &aInitialKeyframe,
                     const unsigned int aMaxSize, const int aFilterSize,
                     const ceres::Problem::Options &aOptions)
    : mMaxSize(aMaxSize), mSlidingWindowSize(aMaxSize),
      mFilterSize(aFilterSize), mPrevKeyframe(aInitialKeyframe),
      mCeresProblem(aOptions), mKeyframeCandidates(aMaxSize), mNodes(aMaxSize),
      mLockedInNodes(aMaxSize), mResidualIds(aMaxSize) {
    // Init Ceres problem options
    mCeresOptions.linear_solver_type = ceres::DENSE_QR;
    mCeresOptions.max_num_iterations = 100;
    mCeresOptions.minimizer_progress_to_stdout = true;
    mCeresOptions.num_threads = 1; // force no multithreading
    // mCeresOptions.min_line_search_step_size = 0.1;

    // Local angle parametisation function
    mAngleParamFn = AngleLocalParameterization::Create();
}

/**
 * @brief Get max size of the graph
 * @note Excludes the implicit origin node/previous keyframe
 *
 * @return Graph max size
 */
const unsigned int PoseGraph::getMaxSize() {
    return mMaxSize;
}

/**
 * @brief Push a node to the graph
 * @param[in] aData Node data to push to vector
 */
void PoseGraph::pushNode(const PoseGraphNodeData &aData) {
    mNodes.push_back(aData);
}

/**
 * @brief Pop a node from the graph
 */
void PoseGraph::popNode() {
    mNodes.pop_front();
}

/**
 * @brief Pop a node from the graph, and push it to the locked in nodes buffer
 */
void PoseGraph::popNodeAndLockIn() {
    mLockedInNodes.push_back(mNodes.front());
    popNode();
}

/**
 * @brief Add a node to the graph
 * @param[in] aData Node data to push to vector
 *
 * @note Exactly the same as pushNode
 */
void PoseGraph::addNode(const PoseGraphNodeData &aData) {
    pushNode(aData);
}

/**
 * @brief Get nodes of the graph
 *
 * @return Reference to pose graph nodes
 */
const PoseGraphNodes &PoseGraph::getNodes() {
    return mNodes;
}

/**
 * @brief Get nodes of the graph that have been "locked in" after PGO was
 * performed and a new keyframe was found.
 *
 * @return Reference to pose graph nodes
 */
const PoseGraphNodes &PoseGraph::getLockedInNodes() {
    return mLockedInNodes;
}

/**
 * @brief Get edge information of the graph
 * @note Reference to the sliding window
 *
 * @return Reference to pose graph edge info
 */
const PoseGraphEdges &PoseGraph::getEdges() {
    return mEdges;
}

/**
 * @brief Get sliding window of the graph
 * @note Reference to the sliding window
 *
 * @return Reference to pose graph edge info
 */
const SlidingWindow &PoseGraph::getSlidingWindow() {
    return mKeyframeCandidates;
}

/**
 * @brief Push a node to the graph
 * @param[in] aData Node data to push to vector
 */
void PoseGraph::pushEdge(const PoseGraphEdgeData &aData) {
    mEdges.push_back(aData);
}

/**
 * @brief Pop an edge from the graph
 */
void PoseGraph::popEdge() {
    mEdges.pop_front();
}

/**
 * @brief Add a node to the graph
 * @param[in] aData Node data to push to vector
 *
 * @note Exactly the same as pushNode
 */
void PoseGraph::addEdge(const PoseGraphEdgeData &aData) {
    pushEdge(aData);
}

/**
 * @brief Get previous keyframe (or origin node) stored inside the pose graph
 * @return Reference to previous keyframe node in pose graph
 */
KeyframeNode &PoseGraph::getPrevKeyframe() {
    return mPrevKeyframe;
}

/**
 * @brief Set previous keyframe (or origin node) stored inside the pose graph.
 * Actually copies over the references.
 * @param[in] aPrevKeyframe Previous keyframe node in pose graph to set
 */
void PoseGraph::setPrevKeyframe(const KeyframeNode &aPrevKeyframe) {
    setPrevKeyframe(aPrevKeyframe.rImage, aPrevKeyframe.pose);
}

/**
 * @brief Set previous keyframe (or origin node) stored inside the pose graph.
 * Actually copies over the references.
 * @param[in] aRImage RadarImage of previous keyframe
 * @param[in] aPose Pose data of previous keyframe
 */
void PoseGraph::setPrevKeyframe(const RadarImage &aRImage,
                                const PoseGraphNodeData &aPose) {
    mPrevKeyframe.rImage = aRImage;
    mPrevKeyframe.pose = aPose;
}

/**
 * @brief Set output text file for debugging
 * @param[in] aOutputTextFile Output text file to specify for debugging
 */
void PoseGraph::setOutputTextFile(FILE *aOutputTextFile) {
    mOutputTextFile = aOutputTextFile;
    return;
}

/**
 ************************************************************************
 * @section PoseGraph-KeyframeHandling Keyframe handling
 * @brief Handling of keyframes, including odometry calcuation, keyframe
 * extracting, sliding window
 ************************************************************************
 */

/**
 * @brief Odometry calculation between 2 radar images. Calculates rotation and
 * translation difference, and returns confidence level.
 *
 * Note that now, odometry is calculated from prev image to current image.
 *
 * Also used for keyframe odometry
 * @see findKeyframe()
 *
 * @param[in] aPrevImage Previous radar image (before "curr" in time)
 * @param[in] aCurrImage Current radar image
 * @param[out] aPredData Rotation and translation data output
 *
 * @return Confidence level. 0 if error.
 *         @see RadarImageHandler::computeConfidenceLevel()
 */
double PoseGraph::calculateOdometry(RadarImage &aPrevImage,
                                    RadarImage &aCurrImage,
                                    RotTransData &aPredData) {
    double confidence = 0;

    aPrevImage.getRotationTranslationDifference(aCurrImage, aPredData,
                                                mFilterSize);
    confidence = computeConfidenceLevel(aPredData);

    return confidence;
}

/**
 * @brief Extracts the best keyframe from all candidates according to
 * algorithm in the paper:
 *
 * First we get a feasible set K by only considering the top 90% of
 * keyframes using confidence level
 *
 * The new keyframe then becomes the node with the largest change in
 * rotation when compared to the previous keyframe
 *
 * @return Index of best keyframe
 */
size_t PoseGraph::extractKeyframeFromCandidates() {
    size_t sz = mKeyframeCandidates.size();
    if (sz == 0) {
        printf_err("Attempt to extract keyframe from empty candidates list!\n");
        return 0;
    }
    else if (sz == 1) {
        return 0;
    }

    // Get highest confidence
    // Note: Caching iterator for best result
    double highestConfidence = 0;
    for (SlidingWindow::const_iterator it = mKeyframeCandidates.begin(),
                                       end = mKeyframeCandidates.end();
         it != end; ++it) {
        highestConfidence = MAX(highestConfidence, (*it).confidenceKeyframe);
    }

    // Considering only valid data above threshold, take the one with the
    // largest rotational difference with respect to prev KEYframe
    const double confidenceThreshold = 0.9 * highestConfidence;
    double largestRotDiff = 0;
    size_t index = 0;
    size_t index_best = 0;

    for (SlidingWindow::const_iterator it = mKeyframeCandidates.begin(),
                                       end = mKeyframeCandidates.end();
         it != end; ++it) {
        const SlidingWindowData &swdata = *it;

        const double conf = swdata.confidenceKeyframe;
        const double dRot = ABS(swdata.data_wrt_keyframe.dRotRad);

        // Must be "feasible"
        if (conf > confidenceThreshold) {
            // Update best based on largest rotation difference
            if (dRot > largestRotDiff) {
                index_best = index;
                largestRotDiff = dRot;
            }
        }

        index++;
    }

    return index_best;
}

/**
 * @brief Updates pose graph from new keyframe index, retrieved from
 * extractKeyframeFromCandidates()
 *
 * Pops away useless keyframe candidates, retrieves information from index, and
 * re-computes phase correlation to update the existing keyframe candidates data
 * to be relative to the new keyframe.
 *
 * Updates pose graphs keyframe data with the new keyframe data
 *
 * Also pops away old nodes, and pushes them to the @c mLockedInNodes buffer;
 * since these nodes contain the optimised point data, they will be later drawn
 * on the visualiser. @see mLockedInNodes, popNodeAndLockIn()
 *
 * @see extractKeyFrameFromCandidates(), findKeyframe()
 *
 * @param[in] aNewKeyframeIndex Index of new keyframe to update from
 */
void PoseGraph::updatePoseGraphFromIndex(const size_t aNewKeyframeIndex) {
    // Check for invalid indexes
    const size_t sz = mKeyframeCandidates.size();
    if (sz == 0) {
        printf_err("Attempt to update empty sliding window");
        return;
    }
    else if (aNewKeyframeIndex >= sz) {
        printf_err(
            "Index %zu out of bounds in sliding window of %zu elements\n",
            aNewKeyframeIndex, sz);
        return;
    }

    // Remove the nodes, edges, residuals before new keyframe
    mLockedInNodes.clear();
    // TODO: Remove multiple quicker
    // #pragma omp parallel for
    for (size_t i = 0; i < aNewKeyframeIndex; i++) {
        popEdge();
        popNodeAndLockIn();
        popResidual();
    }

    // By right, this should be the new keyframe
    SlidingWindowData newKeyframeData = mKeyframeCandidates.front();
    RadarImage &keyframeImage = newKeyframeData.rImage;

    // Loop through to the end to update data
    SlidingWindow::iterator it = mKeyframeCandidates.begin(),
                            end = mKeyframeCandidates.end();

    // We were originally at the front, so move one forward
    ++it;
    // Loop as desired till end
    for (; it != end; ++it) {
        // NOTE: Cannot be constant because swdata.rImage requires changes
        SlidingWindowData &swdata = *it;

        swdata.confidenceKeyframe = calculateOdometry(
            keyframeImage, swdata.rImage, swdata.data_wrt_keyframe);
    }

    // We're now done, so update pose graph with new keyframe
    // and remove the new (now old) keyframe from the sliding window
    setPrevKeyframe(keyframeImage, mNodes.front());

    // Remove edge, node, residual associated with keyframe
    popEdge();
    popNodeAndLockIn();
    popResidual();
}

/**
 * @brief Given a current radar image, tries to check if it is keyframe.
 * Return true if it is a keyframe, and updates the keyframe, as well as the
 * sliding window.
 *
 * Follows the algorithm in the paper for determining when a new keyframe is
 * needed:
 * 1) Sliding window is full
 * 2) <i>Conf(currImage, prevKeyframe) < Conf(prevImage, prevKeyframe)</i>
 * 3) Traversed distance exceeds radar's range [almost impossible!]
 *
 * After which, the sliding window needs to be updated, and all cached data
 * recomputed relative to the new keyframe
 *
 * @see extractKeyframeFromCandidates(), updatePoseGraphFromIndex()
 *
 * @param[in] aCurrImage Current radar image to check if its a keyframe
 *
 * @note After this, the optimised pose data can be found in @c mLockedInNodes.
 * @see getLockedInNodes()
 *
 * @return Whether a new keyframe was found
 */
bool PoseGraph::findKeyframe(RadarImage &aCurrImage) {
    // First obtain information relative to previous keyframe
    RadarImage keyframeImage = mPrevKeyframe.rImage;
    RotTransData predDataKeyframe;
    double keyframeConf =
        calculateOdometry(keyframeImage, aCurrImage, predDataKeyframe);

    // Then obtain information relative to previous frame if it exists,
    // otherwise previous keyframe
    RadarImage prevFrameImage = (mKeyframeCandidates.empty())
                                    ? keyframeImage
                                    : mKeyframeCandidates.back().rImage;

    RotTransData predDataPrevFrame;
    double frameToFrameConf =
        calculateOdometry(prevFrameImage, aCurrImage, predDataPrevFrame);

    // Non-vaild frame, exclude from graph/sliding window
    // TODO: May want to do something if too many frames invalid
    if (frameToFrameConf < VALID_SET_THRESHOLD) {
        printf("Excluded an invalid frame!\n");
        return false;
    }

    /********************************************
     * Update pose graph
     *******************************************/
    PoseGraphEdgeData edgeData;
    edgeData.rImage = aCurrImage;
    edgeData.confidenceKeyframe = keyframeConf;
    edgeData.data_wrt_keyframe = predDataKeyframe;

    convertRotTransToTransData(edgeData.trans_wrt_prev_frame,
                               predDataPrevFrame);
    edgeData.confidenceFrame = frameToFrameConf;

    // Update edges/sliding window
    /// @note Here, updating edge is equivalent to updating the sliding window,
    /// because they are aliases to the same thing
    pushEdge(edgeData);

    // Update nodes using an initial estimate based on the previous node's pose
    // and the phase correlation estimates
    PoseGraphNodeData &prevFramePose =
        (mNodes.empty()) ? mPrevKeyframe.pose : mNodes.back();

    // Estimate x, y, yaw using frame-to-frame estimate to give starter value
    // NOTE: Need to change frame of reference for dx and dy

    // Pose to take translation reference from
    // TODO: Which is better?
    // TODO: Use confidence level?
    const bool guessTransFromKeyframe =
        (keyframeConf >= frameToFrameConf); // true;
    const PoseGraphNodeData &transRefPose =
        (guessTransFromKeyframe) ? mPrevKeyframe.pose : prevFramePose;
    const RotTransData &transRefData =
        (guessTransFromKeyframe) ? predDataKeyframe : predDataPrevFrame;
    Eigen::Vector2d transVec(transRefData.dx, transRefData.dy);

    transVec = Eigen::Rotation2Dd(transRefPose.yaw) * transVec;

    // TODO: Which is better?
    // TODO: Use confidence level?
    const bool guessYawFromKeyframe =
        (keyframeConf >= frameToFrameConf); // true;
    const double guessedYaw =
        (guessYawFromKeyframe)
            ? (mPrevKeyframe.pose.yaw + predDataKeyframe.dRotRad)
            : (prevFramePose.yaw + predDataPrevFrame.dRotRad);

    PoseGraphNodeData framePoseGuess(
        transRefPose.x + transVec[0], transRefPose.y + transVec[1],
        PoseGraphError::normaliseAngle(guessedYaw));

    // Remember to push node to graph
    pushNode(framePoseGuess);

    // Update residual AFTER node and edge added
    pushResidual();

    // Now optimise!
    // TODO: Make non-verbose, once done debugging
    // TODO: if optimisation fails don't update/revert update?
    optimise(true);

    /******************************************************
     * Check if new keyframe needed according to algorithm
     ******************************************************/
    size_t nCandidates = mKeyframeCandidates.size();

    // 1) Sliding window is full
    // NOTE: should only be == but just in case
    if (nCandidates >= mSlidingWindowSize) {
        assert(nCandidates == mSlidingWindowSize);
    }
    // 2) Conf(currImage, prevKeyframe) < Conf(prevImage, prevKeyframe)
    else if (nCandidates > 1 &&
             keyframeConf <
                 mKeyframeCandidates.at(nCandidates - 2).confidenceKeyframe) {
    }
    // 3) Traversed distance exceeds radar's range
    // For optimisation note that it is impossible for x^2 + y^2 >= d^2 if x
    // and y < d / sqrt(2)
    else if (predDataKeyframe.dx >= RADAR_MAX_RANGE_M_2 ||
             predDataKeyframe.dy >= RADAR_MAX_RANGE_M_2) {
        double dist = predDataKeyframe.dx * predDataKeyframe.dx +
                      predDataKeyframe.dy * predDataKeyframe.dy;
        if (dist < RADAR_MAX_RANGE_M_SQUARED) return false;
    }
    else {
        return false;
    }

    // Keyframe is needed: extract it
    size_t newKeyframeIndex = extractKeyframeFromCandidates();
    fprintf(mOutputTextFile, "[KF,%zu]: ", newKeyframeIndex);

    // Also update the sliding window and origin node
    updatePoseGraphFromIndex(newKeyframeIndex);

    // TODO: testing only, remove later
    // optimise(true);

    return true;
}

/**
 ************************************************************************
 * @section PoseGraph-PGOProcessing Pose Graph Optimiser Processing
 ************************************************************************
 */

/**
 * @brief Push a residual block to the Ceres optimiser. Will get the latest
 * pushed node and edge from the graph, and push the ID of the residual block to
 * `mResidualIds`.
 * @see popResidual()
 *
 * @pre Non-empty pose graph edge and node
 * @pre Equal number of pose graph and nodes
 * @pre @note Residuals must be pushed AFTER a node and edge has been added to
 * the sliding window of nodes/edges.
 * @see pushEdge(), pushNode()
 */
void PoseGraph::pushResidual() {
    if (mEdges.empty() || mNodes.empty()) {
        printf_err("Push residual called on empty graph\n");
        return;
    }

    size_t sz = mNodes.size();
    if (mEdges.size() != sz) {
        printf_err("Push residual called on ill-formed graph: Different number "
                   "of edges and nodes\n");
        return;
    }

    // Build cost function according to edge data
    PoseGraphNodeData &keyframePose = mPrevKeyframe.pose;
    const PoseGraphEdgeData edgeData = mEdges.back();

    // TODO: Get an actual information matrix
    /// @see
    /// https://www.visiondummy.com/2014/04/draw-error-ellipse-representing-covariance-matrix/
    const Eigen::Matrix3d informationMatrix =
        Eigen::DiagonalMatrix<double, 3, 3>(
            edgeData.confidenceFrame * 1, edgeData.confidenceFrame * 1,
            edgeData.confidenceKeyframe * HEADING_WEIGHT_FACTOR);
    // TODO: Confidence should be based on the factor it is associated with

    ceres::CostFunction *costFunc =
        PoseGraphError::Create(edgeData, keyframePose, informationMatrix);

    // Add residual according to node data
    PoseGraphNodeData &nodeData = mNodes.back(); // FIXME: Dont use a reference
    PoseGraphNodeData &prevNodeData =
        (sz == 1) ? keyframePose : mNodes.at(sz - 2);

    ceres::ResidualBlockId id = mCeresProblem.AddResidualBlock(
        costFunc, NULL, &prevNodeData.x, &prevNodeData.y, &prevNodeData.yaw,
        &nodeData.x, &nodeData.y, &nodeData.yaw);
    mResidualIds.push_back(id);

    // Also set the parameterisation for constraining the angles
    mCeresProblem.SetParameterization(&nodeData.yaw, mAngleParamFn);

    // Set keyframe blocks constant so that optimiser cannot change it
    if (sz == 1) {
        mCeresProblem.SetParameterBlockConstant(&keyframePose.x);
        mCeresProblem.SetParameterBlockConstant(&keyframePose.y);
        mCeresProblem.SetParameterBlockConstant(&keyframePose.yaw);

        printf("Keyframe Node Data: %lf %lf %lf\n", keyframePose.x,
               keyframePose.y, keyframePose.yaw);
    }

    // FIXME: Check node data pointer change?
    printf("Prev Node Data: %lf %lf %lf\n", *(&prevNodeData.x),
           *(&prevNodeData.y), prevNodeData.yaw);
    printf("Target Node Data: %lf %lf %lf\n", *(&nodeData.x), *(&nodeData.y),
           nodeData.yaw);
}

/**
 * @brief Pop a residual block from the Ceres optimiser. Also remove it from the
 * mResidualId queue
 * @see pushResiduals()
 *
 * @pre Non-empty residual ID queue
 */
void PoseGraph::popResidual() {
    if (mResidualIds.empty()) return;

    mCeresProblem.RemoveResidualBlock(mResidualIds.front());
    mResidualIds.pop_front();
}

/**
 * @brief Print pose information from graph's existing nodes
 */
void PoseGraph::printPoses() {
    printf("Keyframe pose: (x: %lf, y: %lf, yaw: %lf)\n", mPrevKeyframe.pose.x,
           mPrevKeyframe.pose.y, mPrevKeyframe.pose.yaw);

    size_t i = 0;
    for (PoseGraphNodes::const_iterator it = mNodes.begin(), end = mNodes.end();
         it != end; it++) {
        const PoseGraphNodeData &data = *it;
        printf("Frame %zu pose: (x: %lf, y: %lf, yaw: %lf)\n", i, data.x,
               data.y, data.yaw);
        i++;
    }

    printf("\n");
}

/**
 * @brief Print edge information from graph's existing nodes
 */
void PoseGraph::printEdges() {
    size_t i = 0;
    for (PoseGraphEdges::const_iterator it = mEdges.begin(), end = mEdges.end();
         it != end; it++) {
        const PoseGraphEdgeData &data = *it;

        if (i)
            printf("Edge between %zu and %zu: ", i - 1, i);
        else
            printf("Edge between keyframe and 0: ");

        printf("wrt frame: (dx: %lf, dy: %lf), wrt keyframe: (dRot: %lf)\n",
               data.trans_wrt_prev_frame.dx, data.trans_wrt_prev_frame.dy,
               data.data_wrt_keyframe.dRotRad);
        i++;
    }

    printf("\n");
}

/**
 * @brief Use Ceres optimiser to optimise the current graph. Returns success
 * status (i.e. whether graph converged). Will not optimise if only one point
 * available.
 *
 * @param[in] aVerbose Flag for whether to print in verbose info
 *
 * @return Success status
 */
bool PoseGraph::optimise(bool aVerbose) {
    ceres::Solver::Summary summary;

    if (aVerbose) {
        printf("Before optimisation...\n");

        printPoses();
        printEdges();
    }

    ceres::Solve(mCeresOptions, &mCeresProblem, &summary);

    if (aVerbose) {
        std::cout << summary.BriefReport() << "\n";

        printPoses();
        printEdges();
    }

    return summary.IsSolutionUsable();
}

/**
 * @brief Use Ceres optimiser to optimise the current graph. Returns success
 * status (i.e. whether graph converged).
 * @note Exactly the same as optimise(), but American spelling
 * @see optimise()
 *
 * @param[in] aVerbose Flag for whether to print in verbose info
 *
 * @return Success status
 */
bool PoseGraph::optimize(bool aVerbose) {
    return optimise(aVerbose);
}