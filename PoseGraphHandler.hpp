/**
 * @file PoseGraphHandler.hpp
 * @brief Handlers for pose graph optimsation portion of algorithm. Also handles
 * keyframes. Includes elements of sliding window.
 *
 * Notably includes error functor for pose graph: PoseGraphError.
 *
 * @note Meant to be used with RadarFeed class
 * @see RadarFeed.cpp
 *
 * @author Samuel Leong <samleocw@gmail.com>
 */

#ifndef __POSE_GRAPH_HANDLER_H__
#define __POSE_GRAPH_HANDLER_H__

#include <Eigen/Geometry>
#include <boost/circular_buffer.hpp>
#include <ceres/ceres.h>
#include <ceres/local_parameterization.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <vector>

#define _USE_MATH_DEFINES
#include <math.h>

#include "RadarImage.hpp"

/**
 *******************************************************************************
 * @section PoseGraphHandler-SlidingWindow Sliding Window Data Structure
 *
 * @brief The sliding window is used to keep track of the potential keyframe
 * candidates for algorithm
 *
 * The sliding window will contain a struct of @c SlidingWindowData which
 *contains:
 * 1. @c RadarImage of current frame
 * 2. (cached) Rotation/translation data that is always relative to the previous
 * keyframe. @todo Change this to only rotation; now used for vis purposes.
 * 3. Translation data (no rotation) relative to the previous (consecutive)
 * frame
 * 4. Confidence of observation relative to previous keyframe
 *
 * @see RadarFeed.cpp
 * @see RadarFeed::run()
 * @see SlidingWindowData, SlidingWindow
 ******************************************************************************
 */

/**
 * @brief Struct containing all the data that the sliding window needs.
 * Technically, only the RadarImage is necessary, but to save processing time,
 * old RotTransData and confidences are saved.
 *
 * The sliding window will contain a struct of @c SlidingWindowData which
 * contains:
 * 1. @c RadarImage of current frame
 * 2. (cached) Rotation/translation data that is always relative to the previous
 * keyframe. @todo Change this to only rotation; now used for vis purposes.
 * 3. Translation data (no rotation) relative to the previous (consecutive)
 * frame
 * 4. Confidence of observation relative to previous keyframe
 */
typedef struct {
    /** @brief Radar image of current frame */
    RadarImage rImage;

    /**
     * @brief Cached rotation/translation data that is always relative to the
     * previous keyframe
     */
    RotTransData data_wrt_keyframe;

    /** @brief Confidence of observation relative to previous keyframe */
    double confidenceKeyframe;

    /**
     * @brief Translation data (no rotation) relative to the previous
     * (consecutive) frame
     */
    TransData trans_wrt_prev_frame;

    /** @brief Confidence of observation relative to previous frame */
    double confidenceFrame;

} SlidingWindowData;

/**
 * @brief Sliding window data structure
 * @see SlidingWindowData (struct)
 */
typedef boost::circular_buffer<SlidingWindowData> SlidingWindow;

/**
 * @brief Default sliding window size
 */
const int DEFAULT_SLIDING_WINDOW_SIZE = 5;

/**
 * @brief Threshhold for valid set. If confidence falls below this
 * threshhold, it is not counted as part of the valid frame set, and
 * excluded from the sliding window entirely.
 * @todo Exclude entirely, or still have some threshold of frames?
 */
const double VALID_SET_THRESHOLD = 0.1;

/**
 *******************************************************************************
 * @section PoseGraphHandler-PoseGraph Pose graph
 * @brief The (local) pose graph contains N+1 nodes, where N is the number
 *of elements in the sliding window. Every element of the sliding window is
 *a node in the pose graph, with the extra node being the previous keyframe.
 *The previous keyframe node is considered the "origin node", and has its
 * information/value held constant at (0,0). The origin node is implicitly
 *in the pose graph, and cannot be explicitly found in the PoseGraph data
 * structure.
 *
 * Each node has @a (x,y,yaw) pose information, that is optimised at each
 * iteration. The outputs of the final graph may eventually be outputted to
 *the visualiser for visualisation purposes. In this case, the origin @a
 *(0,0,0) is defined where the robot first starts, with the x-axis being
 *"forward", and turning to the right (anticlockwise) constituting positive
 *yaw. Note that the z-axis is pointing into the screen.
 *
 * Each node has exactly 2 edges, corresponding to the following factors
 * 1) Odometry Factor: Translation data between current node and previous
 *node, where the nodes are consecutive in time. It may be possible that
 *there are missing "nodes" in between due to the exclusion of nodes from
 *the valid set. 2) Heading Factor: Rotation data between current node and
 *origin node (the previous keyframe).
 *
 * @note The SlidingWindow data structure basically contains the edge
 * information of each node
 * @see SlidingWindow, SlidingWindowData
 * @see PoseGraphEdges, PoseGraphNodes, PoseGraphNodeData
 ******************************************************************************
 */

/**
 * @brief Edges for the pose graph structure. The i-th element in the vector
 * corresponds to the factor information between that node and the previous
 * node, or the origin nodes, accordingly
 * @note Alias to the SlidingWindow data structure.
 * @see SlidingWindow, SlidingWindowData
 */
typedef SlidingWindow PoseGraphEdges;

/**
 * @brief Edges data for the pose graph structure.
 * @class PoseGraphEdgeData
 * @note Alias to SlidingWindowData
 * @see SlidingWindowData
 */
typedef SlidingWindowData PoseGraphEdgeData;

/**
 * @brief Node data for the pose graph structure. Basically pose
 * information.
 * @see PoseGraphNodes
 */
typedef struct PoseStruct {
    double x = 0;   ///< x-coordinate
    double y = 0;   ///< y-coordinate
    double yaw = 0; ///< yaw value

    /**
     * @brief Set pose data
     *
     * @param[in] aPoint x and y-coordinates to set
     * @param[in] aYaw yaw value to set
     */
    void set(const cv::Point2d &aPoint, const double aYaw) {
        x = aPoint.x;
        y = aPoint.y;
        yaw = aYaw;
    }

    /**
     * @brief Set pose data
     *
     * @param[in] aPoint x and y-coordinates to set
     * @param[in] aYaw yaw value to set
     */
    void set(const Eigen::Vector2d &aPoint, const double aYaw) {
        x = aPoint[0];
        y = aPoint[1];
        yaw = aYaw;
    }

    /**
     * @brief Set pose data
     *
     * @param[in] _x x-coordinate to set
     * @param[in] _y y-coordinate to set
     * @param[in] _yaw yaw value to set
     */
    void set(const double _x, const double _y, const double _yaw) {
        x = _x;
        y = _y;
        yaw = _yaw;
    }

    /**
     * @brief Constructor for PoseStruct
     *
     * @param[in] aPoint x and y coordinates of pose in OpenCV Point format
     * @param[in] aYaw yaw value of pose
     */
    PoseStruct(const cv::Point2d &aPoint, const double aYaw) {
        set(aPoint, aYaw);
    }

    /**
     * @brief Constructor for PoseStruct
     *
     * @param[in] aPoint 2d vector of pose
     * @param[in] aYaw yaw value of pose
     */
    PoseStruct(const Eigen::Vector2d &aPoint, const double aYaw) {
        set(aPoint, aYaw);
    }

    /**
     * @brief Constructor for PoseStruct
     *
     * @param[in] _x x-coordinate of pose
     * @param[in] _y y-coordinate of pose
     * @param[in] _yaw yaw value of pose
     */
    PoseStruct(const double _x, const double _y, const double _yaw) {
        set(_x, _y, _yaw);
    }
} PoseGraphNodeData;

/**
 * @brief Buffer of nodes for the pose graph structure.
 * @see PoseGraphNodes
 */
typedef boost::circular_buffer<PoseGraphNodeData> PoseGraphNodes;

/**
 * @brief KeyframeNode struct to contain necessary keyframe info
 * @note Cannot use reference to @a rImage and @a pose as they will be destroyed
 * after pop from nodes and edges
 * @todo Is @a rImage really necessary?
 */
typedef struct KeyframeNode {
    /** @brief Radar image of keyframe */
    RadarImage rImage;

    /** @brief Pose of keyframe */
    PoseGraphNodeData pose;

    /**
     * @brief KeyframeNode constructor
     *
     * @param aRImage Radar Image of keyframe
     * @param aPose Pose data of keyframe
     */
    KeyframeNode(const RadarImage &aRImage, const PoseGraphNodeData &aPose)
        : rImage(aRImage), pose(aPose) {}
} KeyframeNode;

/**
 *******************************************************************************
 * @section PoseGraphHandler-PoseGraphError Pose graph error functor
 *
 * @brief This class/struct defines the error functor for the pose graph as
 * described in the paper. To be passed into @c ceres::Problem solver.
 *
 * @note Private variables of the error function will contain the constant,
 * absolute measurements.
 *******************************************************************************/
class PoseGraphError {
  public:
    /**
     * @brief Constructor for PoseGraphError class.
     * @note The parameters are "constants" which the PGO uses for error
     * calcuation. They are usually measurements, but in this case, we also take
     * in the keyframe pose data and information matrix, which is are constants
     * for this node.
     *
     * @param[in] aRot Rotation data, relative to the (previous) keyframe.
     * @param[in] aTrans Translation data, relative to the previous frame.
     * @param[in] aKeyframePose Previous keyframe pose. Held constant.
     * @param[in] aInformationMatrix Information matrix is used to weight the
     * errors accordingly.
     */
    PoseGraphError(const double aRot, const TransData &aTrans,
                   const PoseGraphNodeData &aKeyframePose,
                   const Eigen::Matrix3d &aInformationMatrix)
        : mRot(aRot), mTrans(aTrans), mKeyframePose(aKeyframePose),
          mInformationMatrix(aInformationMatrix) {}

    /**
     * @brief Constructor for PoseGraphError class.
     * @note The parameters are "constants" which the PGO uses for error
     * calcuation. They are usually measurements, but in this case, we also take
     * in the keyframe pose data and information matrix, which is are constants
     * for this node.
     * @param[in] aEdgeData Pose graph edge data.
     * @param[in] aKeyframePose Previous keyframe pose <i><x, y, yaw></i>. Held
     * constant.
     * @param[in] aInformationMatrix Information matrix is used to weight the
     * errors accordingly.
     */
    PoseGraphError(const PoseGraphEdgeData &aEdgeData,
                   const PoseGraphNodeData &aKeyframePose,
                   const Eigen::Matrix3d &aInformationMatrix)
        : mRot(aEdgeData.data_wrt_keyframe.dRotRad),
          mTrans(aEdgeData.trans_wrt_prev_frame), mKeyframePose(aKeyframePose),
          mInformationMatrix(aInformationMatrix) {
        printf("PoseGraphError created with measurements:\n\twrt frame: (dx: "
               "%lf dy: %lf)\n\twrt keyframe: (dRot: %lf)\n",
               mTrans.dx, mTrans.dy, mRot);
        printf("\tKeyframe Pose: (x: %lf, y: %lf, yaw: %lf)\n", mKeyframePose.x,
               mKeyframePose.y, mKeyframePose.yaw);
    }

    // TODO: Separate into 2 kinds of constraints
    /// @see
    /// https://ceres-solver.googlesource.com/ceres-solver/+/refs/tags/1.12.0/examples/robot_pose_mle.cc

    /**
     * @brief Normalizes the angle in radians between <tt>[-pi, pi)</tt>,
     * supporting templated format.
     *
     * @tparam T Templated either @c float, @c double or @c Jet (Jacobian matrix
     * used by @c Ceres)
     * @param[in] aAngleRad Angle to normalise, in radians
     * @return Normalised angle, in radians
     */
    template <typename T>
    static inline T normaliseAngle(const T &aAngleRad) {
        // Use ceres::floor because it is specialized for double and Jet types.
        T two_pi(M_TAU);
        return aAngleRad -
               two_pi *
                   ceres::floor((aAngleRad + static_cast<T>(M_PI)) / two_pi);
    }

    /**
     * @brief Templated @c Eigen -based 2D rotation matrix. For use in changing
     * the frame of reference in error functor @see operator()()
     *
     * Copied from @see
     * https://ceres-solver.googlesource.com/ceres-solver/+/refs/tags/1.12.0/examples/slam/pose_graph_2d/pose_graph_2d_error_term.h
     *
     * @tparam T Templated either @c float, @c double or @c Jet (Jacobian matrix
     * used by @c Ceres)
     * @param[in] aYawRad Yaw angle in radians
     * @return Templated @c Eigen rotation matrix
     */
    template <typename T>
    static inline Eigen::Matrix<T, 2, 2> RotationMatrix2D(const T &aYawRad) {
        const T cosYaw = ceres::cos(aYawRad);
        const T sinYaw = ceres::sin(aYawRad);
        Eigen::Matrix<T, 2, 2> rotation;
        rotation << cosYaw, -sinYaw, sinYaw, cosYaw;
        return rotation;
    }

    /**
     * @brief Actual error functor. Each function is associated with 1 node, but
     * only one error/residual is associated per node.
     *
     * Total number of residuals for the whole graph = <i>3 * (N-1)</i>, where
     * @a N-1 is the total number of nodes excluding the keyframe. Each residual
     * corresponds to one of 3 measurements associated with each node: <em><x,
     * y></em> (against frame), @a rot (against keyframe), in accordance with
     * the paper's formula.
     * @todo Check if we should be having a single residual or 3 separate ones
     *
     * @note Parameters are the things to change, in this case, the <em>X, Y,
     * yaw</em> pose components of the nodes. The target node is the node whose
     * error we are trying to calcuate. Odometry factors are measured relative
     * from the target node to the previous node, and the heading factors are
     * measured relative to the keyframe.
     *
     * @note The @a x and @a y -coordinates of the nodes are global x and y
     * positions, i.e. they are in the @b global frame of reference. However,
     * the dx and dy measurements are in the @b relative frame of reference. It
     * is especially important to note that this frame of reference is affected
     * by the <b>heading/yaw of the previous frame</b>, since the coordinate
     * axes are rotated to match the heading.
     *
     * @see
     * https://ceres-solver.googlesource.com/ceres-solver/+/refs/tags/1.12.0/examples/slam/pose_graph_2d/pose_graph_2d_error_term.h
     *
     * @tparam T Templated either @c float, @c double or @c Jet (Jacobian matrix
     * used by @c Ceres)
     * @param[in] aPrevNodeXPtr Pointer to x-coord (of pose) of the previous
     * node
     * @param[in] aPrevNodeYPtr Pointer to y-coord (of pose) of the previous
     * node
     * @param[in] aPrevNodeYawPtr Pointer to yaw (of pose) of the previous node
     * @param[in] aTargetNodeXPtr Pointer to x-coord (of pose) of the target
     * node
     * @param[in] aTargetNodeYPtr Pointer to y-coord (of pose) of the target
     * node
     * @param[in] aTargetNodeYawPtr Pointer to yaw (of pose) of the target node
     * @param[out] aResidualsPtr Pointer to error residual output
     */
    template <typename T>
    bool operator()(const T *const aPrevNodeXPtr, const T *const aPrevNodeYPtr,
                    const T *const aPrevNodeYawPtr,
                    const T *const aTargetNodeXPtr,
                    const T *const aTargetNodeYPtr,
                    const T *const aTargetNodeYawPtr, T *aResidualsPtr) const {
        // Dereference Pointers to get their values
        const T aPrevNodeX = *aPrevNodeXPtr;
        const T aPrevNodeY = *aPrevNodeYPtr;
        const T aPrevNodeYaw = *aPrevNodeYawPtr;

        const T aTargetNodeX = *aTargetNodeXPtr;
        const T aTargetNodeY = *aTargetNodeYPtr;
        const T aTargetNodeYaw = *aTargetNodeYawPtr;

        // Estimated Rotation (from target node pose and keyframe pose)
        const T estRot = aTargetNodeYaw - static_cast<T>(mKeyframePose.yaw);

        // Estimated Positions in GLOBAL frame of reference
        const Eigen::Matrix<T, 2, 1> targetNodePos(aTargetNodeX, aTargetNodeY);
        const Eigen::Matrix<T, 2, 1> prevNodePos(aPrevNodeX, aPrevNodeY);

        // Estimated Translation in RELATIVE frame of reference
        // NOTE: Need to take transpose (i.e. inverse of rotation matrix)
        const Eigen::Matrix<T, 2, 1> estDeltaTrans =
            RotationMatrix2D(aPrevNodeYaw).transpose() *
            (targetNodePos - prevNodePos);

        // Error vector
        const Eigen::Matrix<T, 3, 1> errorVector(
            estDeltaTrans[0] - static_cast<T>(mTrans.dx),
            estDeltaTrans[1] - static_cast<T>(mTrans.dy),
            normaliseAngle(estRot - static_cast<T>(mRot)));

        // aResidualsPtr[0] = errorVector.transpose() *
        //                    mInformationMatrix.template cast<T>() *
        //                    errorVector;

        // aResidualsPtr[0] = errorVector.transpose() * errorVector;

        // Map Eigen 3D vector to residual
        Eigen::Map<Eigen::Matrix<T, 3, 1>> residualsMap(aResidualsPtr);

        // Weight by information matrix
        residualsMap = mInformationMatrix.template cast<T>() * errorVector;

        return true;
    }

    /**
     * @brief Create pose graph error functor
     *
     * @see PoseGraphError() constructor
     *
     * @param[in] aEdgeData Pose graph edge data, notably containing information
     * about @a dx, @a dy (relative to prev frame) and @a dRot (relative to prev
     * keyframe).
     * @param[in] aKeyframePose Previous keyframe pose data.
     * @param[in] aInformationMatrix Information matrix to weight the errors
     *
     * @return Pose graph error/cost function
     */
    static ceres::CostFunction *
    Create(const PoseGraphEdgeData &aEdgeData,
           const PoseGraphNodeData &aKeyframePose,
           const Eigen::Matrix3d &aInformationMatrix) {
        return new ceres::AutoDiffCostFunction<PoseGraphError, nResiduals, 1, 1,
                                               1, 1, 1, 1>(
            new PoseGraphError(aEdgeData, aKeyframePose, aInformationMatrix));
    }

    /**
     * @brief Create pose graph error functor
     *
     * @see PoseGraphError() constructor
     *
     * @param[in] aRot Rotation data, relative to the (previous) keyframe.
     * @param[in] aTrans Translation data, relative to the previous frame.
     * @param[in] aKeyframePose Previous keyframe pose data.
     * @param[in] aInformationMatrix Information matrix to weight the errors
     *
     * @return Pose graph error/cost function
     */
    static ceres::CostFunction *
    Create(const double aRot, const TransData &aTrans,
           const PoseGraphNodeData &aKeyframePose,
           const Eigen::Matrix3d &aInformationMatrix) {
        return new ceres::AutoDiffCostFunction<PoseGraphError, nResiduals, 1, 1,
                                               1, 1, 1, 1>(new PoseGraphError(
            aRot, aTrans, aKeyframePose, aInformationMatrix));
    }

  private:
    /** @brief Rotation relative to (prev) keyframe */
    const double mRot;

    /** @brief Translation relative to previous frame */
    const TransData mTrans;

    /** @brief Keyframe pose data */
    const PoseGraphNodeData mKeyframePose;

    /** @brief Information matrix. Diagonal matrix @a (x,y,theta) */
    const Eigen::Matrix3d mInformationMatrix;

    /** @brief Number of residuals */
    static const int nResiduals = 3;
};

/**
 * @brief Defines a local parameterisation functor for updating angle to be
 * constrained in [-pi, pi). Used in PoseGraphError functor.
 *
 * Copied from
 * https://ceres-solver.googlesource.com/ceres-solver/+/refs/tags/1.12.0/examples/slam/pose_graph_2d/angle_local_parameterization.h
 *
 * @see PoseGraphError functor
 */
class AngleLocalParameterization {
  public:
    /**
     * @brief Functor defintion for normalising of angle
     * @see PoseGraphError::normaliseAngle()
     *
     * @tparam T Templated either @c float, @c double or @c Jet (Jacobian matrix
     * used by @c Ceres)
     * @param[in] theta_radians Original angle [rad]
     * @param[in] delta_theta_radians Proposed change in angle by Ceres
     * optimiser [rad]
     * @param[out] theta_radians_plus_delta Outputted angle [rad]
     * @return true always
     */
    template <typename T>
    bool operator()(const T *theta_radians, const T *delta_theta_radians,
                    T *theta_radians_plus_delta) const {
        *theta_radians_plus_delta = PoseGraphError::normaliseAngle(
            *theta_radians + *delta_theta_radians);
        return true;
    }

    /**
     * @brief Create the LocalParametrization functor
     *
     * @return ceres::LocalParameterization pointer
     */
    static ceres::LocalParameterization *Create() {
        return (
            new ceres::AutoDiffLocalParameterization<AngleLocalParameterization,
                                                     1, 1>);
    }
};

/**
 * @brief Queue for Ceres residual block IDs, for removal purposes. With every
 * node and edge being added, corresponding residuals will be pushed into the
 * queue/sliding window.
 * @note Residuals must be pushed AFTER a node and edge has been added to the
 * sliding window of nodes/edges.
 */
typedef boost::circular_buffer<ceres::ResidualBlockId> ResidualIdQueue;

#endif