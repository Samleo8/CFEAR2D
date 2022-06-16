#include <stdio.h>
#include <string>
#include <vector>

#include "ceres/ceres.h"
#include "glog/logging.h"

/**
 * @section Parabola
 * @brief Solve for the minimum point of a parabola of (10 - x)^2
 */
struct CostFunctorParabola {
    template <typename T>
    bool operator()(const T *const x, T *residual) const {
        residual[0] = T(10.0) - x[0];

        if (std::is_same_v<T, double>)
            printf("x: %lf | residual: %lf\n", x[0], residual[0]);

        return true;
    }
};

void solveParabola() {
    // Init Vars
    const double initialX = 5;
    double x = initialX;

    // Build problem
    ceres::Problem problem;

    // NOTE: Cost function also known as residual
    // Use auto differential and jacobian to obtain derivative
    ceres::CostFunction *cost_fn =
        new ceres::AutoDiffCostFunction<CostFunctorParabola, 1, 1>(
            new CostFunctorParabola);
    problem.AddResidualBlock(cost_fn, NULL, &x);

    // Run the solver!
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "x : " << initialX << " -> " << x << "\n\n";
}

/**
 * @section Powell's Function
 * @brief Solve for a more complicated Powell's function involving 4 cost
 * functions
 */
// f1
struct f1 {
    template <typename T>
    bool operator()(const T *const x1, const T *const x2, T *residual) const {
        residual[0] = x1[0] + 10.0 * x2[0];
        return true;
    }
};

// f2
struct f2 {
    template <typename T>
    bool operator()(const T *const x3, const T *const x4, T *residual) const {
        residual[0] = sqrt(5.0) * (x3[0] - x4[0]);
        return true;
    }
};

// f3
struct f3 {
    template <typename T>
    bool operator()(const T *const x2, const T *const x3, T *residual) const {
        residual[0] = (x2[0] - 2.0 * x3[0]);
        residual[0] *= residual[0];
        return true;
    }
};

// f4
struct f4 {
    template <typename T>
    bool operator()(const T *const x1, const T *const x4, T *residual) const {
        residual[0] = sqrt(10.0) * (x1[0] - x4[0]) * (x1[0] - x4[0]);
        return true;
    }
};

void solvePowell() {
    double x1 = 1;
    double x2 = -1;
    double x3 = 1;
    double x4 = 5;

    // Problem 2: Powells function
    ceres::Problem problem_powell;

    // Note: 1 residual, but each cost function only has 2 input params (1, 1)
    problem_powell.AddResidualBlock(
        new ceres::AutoDiffCostFunction<f1, 1, 1, 1>(new f1), NULL, &x1, &x2);
    problem_powell.AddResidualBlock(
        new ceres::AutoDiffCostFunction<f2, 1, 1, 1>(new f2), NULL, &x3, &x4);
    problem_powell.AddResidualBlock(
        new ceres::AutoDiffCostFunction<f3, 1, 1, 1>(new f3), NULL, &x2, &x3);
    problem_powell.AddResidualBlock(
        new ceres::AutoDiffCostFunction<f4, 1, 1, 1>(new f4), NULL, &x1, &x4);

    // Init solver
    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 100;

    ceres::Solver::Summary summary_powell;
    ceres::Solve(options, &problem_powell, &summary_powell);
    std::cout << summary_powell.FullReport() << "\n";
    printf("Final: x1 = %lf x2 = %lf x3 = %lf x4 = %lf\n", x1, x2, x3, x4);
}

/**
 * @section Exponential curve fitting
 * @brief Solve for the best-fit exponential curve given both a cost function
 * and a loss function
 */
static const double data[] = {
    0.000000e+00, 1.133898e+00, 7.500000e-02, 1.334902e+00, 1.500000e-01,
    1.213546e+00, 2.250000e-01, 1.252016e+00, 3.000000e-01, 1.392265e+00,
    3.750000e-01, 1.314458e+00, 4.500000e-01, 1.472541e+00, 5.250000e-01,
    1.536218e+00, 6.000000e-01, 1.355679e+00, 6.750000e-01, 1.463566e+00,
    7.500000e-01, 1.490201e+00, 8.250000e-01, 1.658699e+00, 9.000000e-01,
    1.067574e+00, 9.750000e-01, 1.464629e+00, 1.050000e+00, 1.402653e+00,
    1.125000e+00, 1.713141e+00, 1.200000e+00, 1.527021e+00, 1.275000e+00,
    1.702632e+00, 1.350000e+00, 1.423899e+00, 1.425000e+00, 1.543078e+00,
    1.500000e+00, 1.664015e+00, 1.575000e+00, 1.732484e+00, 1.650000e+00,
    1.543296e+00, 1.725000e+00, 1.959523e+00, 1.800000e+00, 1.685132e+00,
    1.875000e+00, 1.951791e+00, 1.950000e+00, 2.095346e+00, 2.025000e+00,
    2.361460e+00, 2.100000e+00, 2.169119e+00, 2.175000e+00, 2.061745e+00,
    2.250000e+00, 2.178641e+00, 2.325000e+00, 2.104346e+00, 2.400000e+00,
    2.584470e+00, 2.475000e+00, 1.914158e+00, 2.550000e+00, 2.368375e+00,
    2.625000e+00, 2.686125e+00, 2.700000e+00, 2.712395e+00, 2.775000e+00,
    2.499511e+00, 2.850000e+00, 2.558897e+00, 2.925000e+00, 2.309154e+00,
    3.000000e+00, 2.869503e+00, 3.075000e+00, 3.116645e+00, 3.150000e+00,
    3.094907e+00, 3.225000e+00, 2.471759e+00, 3.300000e+00, 3.017131e+00,
    3.375000e+00, 3.232381e+00, 3.450000e+00, 2.944596e+00, 3.525000e+00,
    3.385343e+00, 3.600000e+00, 3.199826e+00, 3.675000e+00, 3.423039e+00,
    3.750000e+00, 3.621552e+00, 3.825000e+00, 3.559255e+00, 3.900000e+00,
    3.530713e+00, 3.975000e+00, 3.561766e+00, 4.050000e+00, 3.544574e+00,
    4.125000e+00, 3.867945e+00, 4.200000e+00, 4.049776e+00, 4.275000e+00,
    3.885601e+00, 4.350000e+00, 4.110505e+00, 4.425000e+00, 4.345320e+00,
    4.500000e+00, 4.161241e+00, 4.575000e+00, 4.363407e+00, 4.650000e+00,
    4.161576e+00, 4.725000e+00, 4.619728e+00, 4.800000e+00, 4.737410e+00,
    4.875000e+00, 4.727863e+00, 4.950000e+00, 4.669206e+00, 1.425000e+00,
    5.543078e+00,               // Outlier point
    1.500000e+00, 5.664015e+00, // Outlier point
};
static const int numObservations = 0.5 * sizeof(data) / sizeof(data[0]);

struct ExponentResidual {
  public:
    ExponentResidual(double x, double y) : _x(x), _y(y) {}
    template <typename T>
    bool operator()(const T *const m, const T *const c, T *residual) const {
        residual[0] = _y - exp(m[0] * _x + c[0]);
        return true;
    }

  private:
    const double _x;
    const double _y;
};

void solveCurveFit() {
    ceres::Problem problem;

    // Construct problem
    double m = 0.0;
    double c = 0.0;
    for (int i = 0; i < numObservations; i++) {
        problem.AddResidualBlock(
            new ceres::AutoDiffCostFunction<ExponentResidual, 1, 1, 1>(
                new ExponentResidual(data[2 * i], data[2 * i + 1])),
            new ceres::CauchyLoss(0.5), &m, &c);
    }

    ceres::Solver::Options options;
    options.linear_solver_type = ceres::DENSE_QR;
    options.minimizer_progress_to_stdout = true;
    options.max_num_iterations = 25;

    ceres::Solver::Summary summary;
    ceres::Solve(options, &problem, &summary);

    std::cout << summary.BriefReport() << "\n";
    std::cout << "Initial m: " << 0.0 << " c: " << 0.0 << "\n";
    std::cout << "Final   m: " << m << " c: " << c << "\n";

    return;
}

int main(int argc, char **argv) {
    // Setup logging
    // google::InitGoogleLogging(argv[0]);

    solveParabola();
    // solvePowell();
    // solveCurveFit();

    return 0;
}