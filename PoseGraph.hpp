/**
 * @file PoseGraph.hpp
 * @brief Basic pose graph class that contains the nodes and edges structs. Also
 * allows for optimisations to be performed on it.
 *
 * @note Meant to be used with RadarFeed class
 * @see RadarFeed.cpp
 * @see PoseGraphHandler.cpp
 *
 * @see SlidingWindow, SlidingWindowData
 * @see PoseGraphEdges, PoseGraphNodes, PoseGraphNodeData
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#ifndef __POSE_GRAPH_H__
#define __POSE_GRAPH_H__

#include "PoseGraphHandler.hpp"

/**
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
 */
class PoseGraph {
  private:
    /** @brief Keyframe candidates */
    SlidingWindow mKeyframeCandidates;
    /**
     * @brief Pose graph edges ( @c SlidingWindow )
     * @see PoseGraphEdgeData
     * @note Edges of pose graphs is a reference to the sliding window
     */
    PoseGraphEdges &mEdges = mKeyframeCandidates;

    /**
     * @brief Keyframe candidates
     * @see PoseGraphNodeData
     * @note Edges of pose graphs is a reference to the sliding window
     */
    PoseGraphNodes mNodes;

    /**
     * @brief Buffer of nodes that have just been "locked in" after local PGO is
     * performed, and a keyframe is found
     * @see extractKeyframeFromCandidates(), updatePoseGraphFromIndex()
     */
    PoseGraphNodes mLockedInNodes;

    /**
     * @brief Max size of graph
     * @note Equal to sliding window size
     */
    const unsigned int mMaxSize;

    /**
     * @brief Max size of graph
     * @note Equal to max graph size
     */
    const unsigned int mSlidingWindowSize;

    /** @brief Filter size for phase corr estimation */
    const int mFilterSize;

    /** @brief Previous keyframe data aka. origin node */
    KeyframeNode mPrevKeyframe;

    /** @brief Output file for debugging */
    FILE *mOutputTextFile = stdout;

    /** @brief Ceres problem for pose graph optimisation */
    ceres::Problem mCeresProblem;

    /** @brief Ceres options for problem */
    ceres::Solver::Options mCeresOptions;

    /** @brief Queue (circular buffer) of residual IDs for quick removal */
    ResidualIdQueue mResidualIds;

    /** @brief Angle normalising/constraining parameterisation function */
    ceres::LocalParameterization *mAngleParamFn;

  public:
    // Constructors
    PoseGraph(
        const KeyframeNode &aInitialKeyframe,
        const unsigned int aMaxSize = DEFAULT_SLIDING_WINDOW_SIZE,
        const int aFilterSize = DEFAULT_FILTER_SIZE,
        const ceres::Problem::Options &aOptions = ceres::Problem::Options());

    // Getters Setters
    const unsigned int getMaxSize();
    const PoseGraphNodes &getNodes();
    const PoseGraphNodes &getLockedInNodes();
    void pushNode(const PoseGraphNodeData &aData);
    void popNode();
    void addNode(const PoseGraphNodeData &aData);
    void popNodeAndLockIn();

    const PoseGraphEdges &getEdges();
    void pushEdge(const PoseGraphEdgeData &aData);
    void popEdge();
    void addEdge(const PoseGraphEdgeData &aData);

    void setOutputTextFile(FILE *aOutputTextFile);

    KeyframeNode &getPrevKeyframe();

    void setPrevKeyframe(const RadarImage &aRImage,
                         const PoseGraphNodeData &aPos);
    void setPrevKeyframe(const KeyframeNode &aPrevKeyframe);

    const SlidingWindow &getSlidingWindow();

    // Sliding Window and Keyframe Handling
    double calculateOdometry(RadarImage &aPrevImage, RadarImage &aCurrImage,
                             RotTransData &aPredData);

    void updatePoseGraphFromIndex(const size_t aNewKeyframeIndex);

    size_t extractKeyframeFromCandidates();

    bool findKeyframe(RadarImage &aCurrImage);

    // Pose graph optimisation handling
    void pushResidual();
    void popResidual();
    bool optimise(bool aVerbose = false);
    bool optimize(bool aVerbose = false);

    void printPoses();
    void printEdges();

    /** @brief Weight of heading factor in information matrix */
    const double HEADING_WEIGHT_FACTOR = 10.0;
};

#endif