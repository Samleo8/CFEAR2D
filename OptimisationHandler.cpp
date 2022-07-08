/**
 * @file OptimisationHandler.cpp
 * @author Samuel Leong (scleong@andrew.cmu.edu)
 * @brief Handler for optimisation functions, including the building and solving
 * of registration and motion distortion optimisation problems. 
 * Notably, @see buildAndSolveRegistrationProblem()
 * @version 0.1
 * @date 2022-07-08
 *
 * @copyright Copyright (c) 2022
 */

#include "OptimisationHandler.hpp"

/**
 * @brief Given optimization parameters (pose in this case), add residual blocks
 * to the existing Ceres problem based on point2line cost for EACH feature point
 * in the radar image and specific keyframe. Cost function is found in @see
 * RegistrationCostFunctor
 * @note One residual block is added for each point association, so that the
 * loss function equation is correct. Otherwise, because Ceres squares the
 * residuals internally, we would need to square the sums then sqrt etc. to
 * match Ceres' style of residual computation
 *
 * @param[in] aProblem Reference to Ceres problem @see
 * buildAndSolveRegistrationProblem()
 * @param[in] aLossFnPtr Pointer to loss function to use for optimization
 * @param[in] aRImage Reference to radar image, contains feature points to
 * associate against
 * @param[in] aKeyframe Reference to keyframe to associate against
 * @param[in] positionArr Pointers to position params for optimization
 * @param[in] orientationArr Pointers to orientation params for optimization
 */
const void buildPoint2LineProblem(ceres::Problem &aProblem,
                                  ceres::LossFunction *aLossFnPtr,
                                  const RadarImage &aRImage,
                                  const Keyframe &aKeyframe,
                                  double *positionArr, double *orientationArr) {
    // For each ORSP point in RImage, add a residual block related to the cost
    // of a single rImg feature point association with its closest keyframe
    // counterpart
    const ORSPVec<double> rImgFeaturePts = aRImage.getORSPFeaturePoints();
    for (const ORSP<double> &featurePt : rImgFeaturePts) {
        ceres::CostFunction *regCostFn =
            RegistrationCostFunctor::Create(featurePt, aKeyframe);

        aProblem.AddResidualBlock(regCostFn, aLossFnPtr, positionArr,
                                  orientationArr);
    }
}

/**
 * @brief Build and solve registration problem using Ceres. This function
 * creates a Ceres problem where it aims to minimise the cost of registration
 * between a radar image and a set of keyframes, using @see
 * buildPoint2LineProblem(), RegistrationCostFunctor
 * @note Residuals are associated with EACH point association (i.e. one residual
 * block is created for each valid point association)
 *
 * @param[in] aRImage Radar image to register against
 * @param[in] aKFBuffer Circular buffer of keyframes
 * @param[in, out] aPose Pose of radar image in world frame. Input is a seed
 * pose based on some velocity model, and output is pose found from optimization
 * @return success
 */
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

    // TODO: When 3D set to EigenQuaternion manifold
    // Angle Manifold
    ceres::Manifold *angleManifold = AngleManifold::Create();
    problem.SetManifold(&aPose.orientation, angleManifold);

    // Build the point to line problem for each keyframe, adding residual blocks
    // for each associated point
    for (size_t i = 0, sz = aKFBuffer.size(); i < sz; i++) {
        const Keyframe &kf = aKFBuffer[i];

        buildPoint2LineProblem(problem, regLossFn, aRImage, kf, positionArr,
                               orientationArr);
    }

    // Solve after building problem
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

    return success;
}