#ifndef LINEAR_SOLVER_H
#define LINEAR_SOLVER_H

#include <functional>
#include <vector>

#ifndef FLOW_NO_EIGEN3
#include <Eigen/SparseCore>
#include <Eigen/SparseLU>
#endif

namespace LinearSolver
{

#ifndef FLOW_NO_EIGEN3
    // Sparse matrix type definitions for efficient storage and solving
    using SpMat = Eigen::SparseMatrix<double, Eigen::ColMajor>;
    using Triplet = Eigen::Triplet<double>;
#else
    struct Triplet
    {
        int row;
        int col;
        double value;

        Triplet(int r, int c, double v) : row(r), col(c), value(v) {}
    };
#endif

    enum class Method
    {
        GaussianElimination = 0,
        GaussSeidel = 1,
        ParallelGaussian = 2,
        JacobiParallel = 3,
        ConjugateGradient = 4,
        CudaDense = 5,
        SparseLU = 6 // New sparse matrix method
    };

    // Controls how GPU-accelerated methods behave:
    // - CpuOnly: never use CUDA (even if available).
    // - Hybrid: try CUDA, but allow CPU fallback.
    // - GpuOnly: require CUDA; no CPU fallback.
    enum class ComputeBackend
    {
        CpuOnly = 0,
        Hybrid = 1,
        GpuOnly = 2
    };

    struct IterationHistory
    {
        std::vector<double> residuals;
        int totalIterations;
        bool converged;

        IterationHistory() : totalIterations(0), converged(false) {}
    };

    void setComputeBackend(ComputeBackend backend);
    ComputeBackend getComputeBackend();

    // Sets the maximum OpenMP threads used by CPU-parallel methods.
    // Value <= 0 keeps the runtime default.
    void setMaxThreads(int threads);
    int getMaxThreads();
    int getRuntimeMaxThreads();

    // Dense matrix solver (original interface)
    bool solve(std::vector<std::vector<double>> &a,
               std::vector<double> &b,
               std::vector<double> &x,
               Method method,
               double tol = 1e-10,
               int maxIter = 10000,
               double relaxation = 1.0,
               std::function<void(int)> iterCallback = nullptr);

    // Sparse matrix solver (new interface using Eigen triplet format)
    bool solveSparse(std::vector<Triplet> &triplets,
                     std::size_t matrixSize,
                     std::vector<double> &b,
                     std::vector<double> &x,
                     double tol = 1e-10);

    const IterationHistory &getLastIterationHistory();
    void resetIterationHistory();

} // namespace LinearSolver

#endif // LINEAR_SOLVER_H
