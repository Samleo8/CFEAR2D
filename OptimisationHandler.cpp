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
 * @brief Build and solve registration problem using Ceres. This function
 * creates a Ceres problem where it aims to minimise the cost of registration
 * between a radar image and a set of keyframes
 * @see RegistrationCostFunctor
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
    // Create array (pointers) from params to feed into problem residual solver
    double positionArr[2] = { aPose.position[0], aPose.position[1] };
    double orientationArr[1] = { aPose.orientation };

    // Create Ceres problem with appropriate options
    ceres::Problem problem;
    ceres::Solver::Summary summary;
    ceres::Solver::Options options;

    // as specified by paper, can potentially use L-BFGS
    options.minimizer_type = ceres::LINE_SEARCH;
    options.line_search_direction_type = ceres::BFGS;
    options.max_num_iterations = 100;

    ceres::LossFunction *regLossFn = new ceres::HuberLoss(HUBER_DELTA_DEFAULT);

    // For each feature point, and keyframe, associate said feature point with a
    // keyframe and add residual block to problem if association succeeds
    // For each ORSP point in RImage, add a residual block related to the cost
    // of a single rImg feature point association with its closest keyframe
    // counterpart

    // Create pose transform and get radar image feature points in local
    // coordinates
    const PoseTransform2D<double> rImgTransform =
        paramsToTransform<double>(positionArr, orientationArr);

    // For each feature point in RImage, associate it with feature point in
    // keyframes
    const ORSPVec<double> rImgFeaturePts = aRImage.getORSPFeaturePoints();
    for (const ORSP<double> &featurePt : rImgFeaturePts) {
        // Transform radar image feature point into world coordinates
        ORSP<double> featurePtWorld;
        convertORSPCoordinates<double>(featurePt, featurePtWorld,
                                       rImgTransform);

        // For each keyframe, associate feature point
        for (size_t i = 0, sz = aKFBuffer.size(); i < sz; i++) {
            const ORSPVec<double> kfFeaturePoints =
                aKFBuffer[i].getORSPFeaturePoints();

            // Find closest keyframe ORSP feature point to image feature point
            // transformed into world coordinates from seeding transform
            ORSP<double> closestORSPPoint;
            const bool found = findClosestORSPInSet<double>(
                featurePtWorld, kfFeaturePoints, closestORSPPoint);

            // Only add residual block if association succeeds
            if (found) {
                ceres::CostFunction *regCostFn =
                    RegistrationCostFunctor::Create(featurePtWorld,
                                                    closestORSPPoint);

                problem.AddResidualBlock(regCostFn, regLossFn, positionArr,
                                         orientationArr);
            }
        }
    }

    // NOTE: Manifold must be set after residual blocks are added
    // TODO: When 3D set to EigenQuaternion manifold
    // Angle Manifold
    ceres::Manifold *angleManifold = AngleManifold::Create();
    problem.SetManifold(orientationArr, angleManifold);

    // For debugging of evaluation values
#ifdef __DEBUG_OPTIMISATION__

    double cost = 0.0;
    std::vector<double> residuals;
    std::vector<double> gradients;
    ceres::CRSMatrix jacobian;

    problem.Evaluate(ceres::Problem::EvaluateOptions(), &cost, &residuals,
                     &gradients, &jacobian);

    std::cout << "================Eval Debug================" << std::endl;
    std::cout << "Evaluated Cost: " << cost << std::endl;

    std::cout << "Residuals: ";
    for (double residual : residuals)
        std::cout << residual << " ";
    std::cout << std::endl;

    std::cout << "Gradients: ";
    for (double gradient : gradients)
        std::cout << gradient << " ";
    std::cout << std::endl;

// TODO: Print jacobian via Eigen map
// std::cout << "Jacobian: " << std::endl;
// Eigen::Map<Eigen::MatrixXd> jacobianEigen(jacobian, jacobian.num_rows,
//                                           jacobian.num_cols);
#endif

    // Solve after building problem
    ceres::Solve(options, &problem, &summary);

    // Check if solution is usable
    bool success = summary.IsSolutionUsable();
    if (success) {
        // Save the parameters
        aPose.position = Eigen::Vector2d(positionArr[0], positionArr[1]);
        aPose.orientation = orientationArr[0];

        std::cout << "New frame pose: " << aPose << std::endl;
    }
    else {
        std::cout << "==================" << std::endl;
        std::cout << "No solution found!" << std::endl;
        std::cout << summary.FullReport() << std::endl;
        std::cout << "==================" << std::endl;
    }

    return success;
}