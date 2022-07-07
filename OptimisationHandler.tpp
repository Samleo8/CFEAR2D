/**
 * @file OptimisationHandler.tpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Implementation file for templated functions related to optimization
 * and cost functions for bundle adjustment
 * @version 0.1
 * @date 2022-06-28
 *
 * @copyright Copyright (c) 2022
 *
 */

#ifndef __OPTIMISATION_HANDLER_TPP__
#define __OPTIMISATION_HANDLER_TPP__

/**
 * @brief Constrain angle in radians between [-pi and pi)
 * @ref
 * https://github.com/ceres-solver/ceres-solver/blob/master/examples/slam/pose_graph_2d/normalize_angle.h
 *
 * @tparam T Scalar type, used by ceres
 * @param[in] aAngleRad
 * @return T
 */
template <typename T> T constrainAngle(const T &aAngleRad) {
    // Use ceres::floor because it is specialized for double and Jet types.
    T two_pi(2.0 * M_PI);
    return aAngleRad - two_pi * ceres::floor((aAngleRad + T(M_PI)) / two_pi);
}

/**
 * @brief Obtain the angle between two vectors. Used for comparing angle
 * tolerance between normals of associated points
 *
 * @param[in] aVec1 Vector 1 of arbitrary dimenion k
 * @param[in] aVec2 Vector 2 of arbitrary dimenion k
 * @return Constrained Angle between the two vectors in radians
 */
template <typename T, int Dimension>
const T angleBetweenVectors(const VectorDimT<T, Dimension> &aVec1,
                            const VectorDimT<T, Dimension> &aVec2) {
    T unnormalizedAngle =
        ceres::acos(aVec1.dot(aVec2) / (aVec1.norm() * aVec2.norm()));

    return constrainAngle<T>(unnormalizedAngle);
}

/**
 * @brief Huber loss according to formula
 * @see https://en.wikipedia.org/wiki/Huber_loss
 *
 * @param[in] a Value
 * @param[in] delta threshold for Huber loss
 * @return Huber loss
 */
template <typename T> const T HuberLoss(const T &a, const T &delta) {
    if (ceres::abs(a) < delta) {
        return 0.5 * a * a;
    }
    else {
        return delta * (ceres::abs(a) - 0.5 * delta);
    }
}

/**
 * @brief Cost between point to line given a radar image and specific keyframe,
 and
 * optimization parameters (in this case, a pose)
 *
 * @tparam T Type of data to use for optimization, used by Ceres
 * @param[in] aRImage Radar image to register against
 * @param[in] aKeyframe Keyframe to register against
 * @param[in] aPose Optimization parameters (in this case a pose)
 * @param[out] aOutputCost Pointer to output cost between point to line as
 indicated by cost function
 *
 * @return Successfully found cost between point to line
 */
const void buildPoint2LineProblem(ceres::Problem &aProblem,
                                  ceres::LossFunction *aLossFn,
                                  const RadarImage &aRImage,
                                  const Keyframe &aKeyframe,
                                  double *positionArr, double *orientationArr) {
    // Loop through each point from ORSP point in RImage and get the cost from
    // formula
    const ORSPVec<double> rImgFeaturePts = aRImage.getORSPFeaturePoints();
    for (const ORSP<double> &featurePt : rImgFeaturePts) {
        ceres::CostFunction *regCostFn =
            RegistrationCostFunctor::Create(aKeyframe, featurePt);

        problem.AddResidualBlock(regCostFn, regLossFn, positionArr,
                                 orientationArr);
    }
}

const bool buildAndSolveRegistrationProblem(const RadarImage &aRImage,
                                            const KeyframeBuffer &aKFBuffer,
                                            Pose2D<double> &aPose) {
    // Create array pointers from params to feed into problem residual solver
    double positionArr[2] = { aPose.position[0], aPose.position[1] };
    double orientationArr[1] = { aPose.orientation };

    // Create Ceres problem
    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;
    // as specified by paper, can potentially use L-BFGS
    options.line_search_direction_type = ceres::BFGS;
    // options.max_num_iterations = 100;

    ceres::LossFunction *regLossFn = new ceres::HuberLoss(HUBER_DELTA_DEFAULT);

    problem->SetManifold(&aPose.orientation, angleManifold);

    for (size_t i = 0, sz = aKFBuffer.size(); i < sz; i++) {
        const Keyframe &kf = aKFBuffer[i];

        buildPoint2LineProblem(problem, regLossFn, aRImage, kf, positionArr,
                               orientationArr);
    }

    ceres::Solve(options, &problem, &summary);

    bool success = summary.IsSolutionUsable();

    if (success) {
        std::cout << "Success!";
        std::cout << "New frame pose: " << positionArr[0] << " "
                  << positionArr[1] << " " << orientationArr[0] << std::endl;

        // Save the parameters
        aPose.position = Eigen::Vector2d(positionArr[0], positionArr[1]);
        aPose.orientation = orientationArr[0];
    }
    else {
        std::cout << "==================" << std::endl;
        std::cout << "No solution found!" << std::endl;
        std::cout << summary.FullReport() << std::endl;
        std::cout << "==================" << std::endl;
    }
}

#endif // __OPTIMISATION_HANDLER_TPP__