#include "linear_solver.h"

#include "cuda_linear_solver.h"
#include "sparse_matrix.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <string>
#include <utility>

#ifdef _OPENMP
#include <omp.h>
#endif

namespace
{

    thread_local LinearSolver::IterationHistory g_lastHistory;
    LinearSolver::ComputeBackend g_computeBackend = LinearSolver::ComputeBackend::Hybrid;
    int g_maxThreads = 0;

    void applyOpenMpSettings()
    {
#ifdef _OPENMP
        if (g_maxThreads > 0)
        {
            omp_set_dynamic(0);
            omp_set_num_threads(g_maxThreads);
        }
#endif
    }

    void startHistory()
    {
        g_lastHistory.residuals.clear();
        g_lastHistory.totalIterations = 0;
        g_lastHistory.converged = false;
    }

    double computeResidualNorm(const std::vector<std::vector<double>> &a,
                               const std::vector<double> &b,
                               const std::vector<double> &x)
    {
        const int n = static_cast<int>(a.size());
        if (n == 0 || static_cast<int>(b.size()) != n || static_cast<int>(x.size()) != n)
        {
            return 1e300;
        }

        double sumSq = 0.0;
        for (int i = 0; i < n; ++i)
        {
            if (static_cast<int>(a[static_cast<std::size_t>(i)].size()) != n)
            {
                return 1e300;
            }
            double ax = 0.0;
            for (int j = 0; j < n; ++j)
            {
                ax += a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                      x[static_cast<std::size_t>(j)];
            }
            const double ri = b[static_cast<std::size_t>(i)] - ax;
            sumSq += ri * ri;
        }
        return std::sqrt(sumSq);
    }

    bool gaussianElimination(std::vector<std::vector<double>> &a,
                             std::vector<double> &b,
                             std::vector<double> &x)
    {
        const int n = static_cast<int>(a.size());
        if (n == 0 || static_cast<int>(b.size()) != n)
        {
            return false;
        }
        for (int i = 0; i < n; ++i)
        {
            if (static_cast<int>(a[static_cast<std::size_t>(i)].size()) != n)
            {
                return false;
            }
        }

        std::vector<std::vector<double>> m(static_cast<std::size_t>(n),
                                           std::vector<double>(static_cast<std::size_t>(n + 1), 0.0));
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                m[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                    a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            }
            m[static_cast<std::size_t>(i)][static_cast<std::size_t>(n)] =
                b[static_cast<std::size_t>(i)];
        }

        const double eps = 1e-14;
        for (int col = 0, row = 0; col < n && row < n; ++col)
        {
            int sel = row;
            for (int i = row; i < n; ++i)
            {
                if (std::abs(m[static_cast<std::size_t>(i)][static_cast<std::size_t>(col)]) >
                    std::abs(m[static_cast<std::size_t>(sel)][static_cast<std::size_t>(col)]))
                {
                    sel = i;
                }
            }
            if (std::abs(m[static_cast<std::size_t>(sel)][static_cast<std::size_t>(col)]) < eps)
            {
                continue;
            }
            if (sel != row)
            {
                std::swap(m[static_cast<std::size_t>(sel)], m[static_cast<std::size_t>(row)]);
            }
            const double inv = 1.0 / m[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
            for (int j = col; j <= n; ++j)
            {
                m[static_cast<std::size_t>(row)][static_cast<std::size_t>(j)] *= inv;
            }
            for (int i = 0; i < n; ++i)
            {
                if (i == row)
                {
                    continue;
                }
                const double factor =
                    m[static_cast<std::size_t>(i)][static_cast<std::size_t>(col)];
                for (int j = col; j <= n; ++j)
                {
                    m[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] -=
                        factor * m[static_cast<std::size_t>(row)][static_cast<std::size_t>(j)];
                }
            }
            ++row;
        }

        x.assign(static_cast<std::size_t>(n), 0.0);
        for (int i = 0; i < n; ++i)
        {
            int pivot = -1;
            for (int j = 0; j < n; ++j)
            {
                if (std::abs(m[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]) > 0.5)
                {
                    pivot = j;
                    break;
                }
            }
            if (pivot == -1)
            {
                if (std::abs(m[static_cast<std::size_t>(i)][static_cast<std::size_t>(n)]) > eps)
                {
                    return false;
                }
                continue;
            }
            x[static_cast<std::size_t>(pivot)] =
                m[static_cast<std::size_t>(i)][static_cast<std::size_t>(n)];
        }
        return true;
    }

    bool parallelGaussianElimination(std::vector<std::vector<double>> &a,
                                     std::vector<double> &b,
                                     std::vector<double> &x)
    {
        const int n = static_cast<int>(a.size());
        if (n == 0 || static_cast<int>(b.size()) != n)
        {
            return false;
        }
        for (int i = 0; i < n; ++i)
        {
            if (static_cast<int>(a[static_cast<std::size_t>(i)].size()) != n)
            {
                return false;
            }
        }

        std::vector<std::vector<double>> m(static_cast<std::size_t>(n),
                                           std::vector<double>(static_cast<std::size_t>(n + 1), 0.0));
        for (int i = 0; i < n; ++i)
        {
            for (int j = 0; j < n; ++j)
            {
                m[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] =
                    a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
            }
            m[static_cast<std::size_t>(i)][static_cast<std::size_t>(n)] =
                b[static_cast<std::size_t>(i)];
        }

        const double eps = 1e-14;
        for (int col = 0, row = 0; col < n && row < n; ++col)
        {
            int sel = row;
            for (int i = row; i < n; ++i)
            {
                if (std::abs(m[static_cast<std::size_t>(i)][static_cast<std::size_t>(col)]) >
                    std::abs(m[static_cast<std::size_t>(sel)][static_cast<std::size_t>(col)]))
                {
                    sel = i;
                }
            }
            if (std::abs(m[static_cast<std::size_t>(sel)][static_cast<std::size_t>(col)]) < eps)
            {
                continue;
            }
            if (sel != row)
            {
                std::swap(m[static_cast<std::size_t>(sel)], m[static_cast<std::size_t>(row)]);
            }
            const double inv = 1.0 / m[static_cast<std::size_t>(row)][static_cast<std::size_t>(col)];
            for (int j = col; j <= n; ++j)
            {
                m[static_cast<std::size_t>(row)][static_cast<std::size_t>(j)] *= inv;
            }

#ifdef _OPENMP
#pragma omp parallel for schedule(static)
#endif
            for (int i = 0; i < n; ++i)
            {
                if (i == row)
                {
                    continue;
                }
                const double factor =
                    m[static_cast<std::size_t>(i)][static_cast<std::size_t>(col)];
                for (int j = col; j <= n; ++j)
                {
                    m[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] -=
                        factor * m[static_cast<std::size_t>(row)][static_cast<std::size_t>(j)];
                }
            }
            ++row;
        }

        x.assign(static_cast<std::size_t>(n), 0.0);
        for (int i = 0; i < n; ++i)
        {
            int pivot = -1;
            for (int j = 0; j < n; ++j)
            {
                if (std::abs(m[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)]) > 0.5)
                {
                    pivot = j;
                    break;
                }
            }
            if (pivot == -1)
            {
                if (std::abs(m[static_cast<std::size_t>(i)][static_cast<std::size_t>(n)]) > eps)
                {
                    return false;
                }
                continue;
            }
            x[static_cast<std::size_t>(pivot)] =
                m[static_cast<std::size_t>(i)][static_cast<std::size_t>(n)];
        }
        return true;
    }

    bool gaussSeidel(std::vector<std::vector<double>> &a,
                     std::vector<double> &b,
                     std::vector<double> &x,
                     double tol,
                     int maxIter,
                     double relaxation,
                     std::function<void(int)> iterCallback)
    {
        const int n = static_cast<int>(a.size());
        if (n == 0 || static_cast<int>(b.size()) != n)
        {
            return false;
        }
        for (int i = 0; i < n; ++i)
        {
            if (static_cast<int>(a[static_cast<std::size_t>(i)].size()) != n)
            {
                return false;
            }
        }

        startHistory();
        x.assign(static_cast<std::size_t>(n), 0.0);
        std::vector<double> xOld(static_cast<std::size_t>(n), 0.0);

        double omega = relaxation;
        if (omega <= 0.0)
        {
            omega = 1.0;
        }
        if (omega > 1.0)
        {
            omega = 1.0;
        }

        for (int iter = 0; iter < maxIter; ++iter)
        {
            double maxDiff = 0.0;
            for (int i = 0; i < n; ++i)
            {
                const double diag = a[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)];
                if (std::abs(diag) < 1e-15)
                {
                    return false;
                }
                double s = b[static_cast<std::size_t>(i)];
                for (int j = 0; j < i; ++j)
                {
                    s -= a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                         x[static_cast<std::size_t>(j)];
                }
                for (int j = i + 1; j < n; ++j)
                {
                    s -= a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                         xOld[static_cast<std::size_t>(j)];
                }
                const double xPrev = x[static_cast<std::size_t>(i)];
                double xi = s / diag;
                if (omega != 1.0)
                {
                    xi = xPrev + omega * (xi - xPrev);
                }
                maxDiff = std::max(maxDiff, std::abs(xi - xPrev));
                x[static_cast<std::size_t>(i)] = xi;
            }
            xOld = x;
            const double residualNorm = computeResidualNorm(a, b, x);
            g_lastHistory.residuals.push_back(residualNorm);
            g_lastHistory.totalIterations = iter + 1;
            if (iterCallback)
            {
                iterCallback(iter);
            }
            if (residualNorm < tol || maxDiff < tol)
            {
                g_lastHistory.converged = true;
                return true;
            }
        }
        return false;
    }

    bool jacobiParallel(std::vector<std::vector<double>> &a,
                        std::vector<double> &b,
                        std::vector<double> &x,
                        double tol,
                        int maxIter,
                        double relaxation,
                        std::function<void(int)> iterCallback)
    {
        const int n = static_cast<int>(a.size());
        if (n == 0 || static_cast<int>(b.size()) != n)
        {
            return false;
        }
        for (int i = 0; i < n; ++i)
        {
            if (static_cast<int>(a[static_cast<std::size_t>(i)].size()) != n)
            {
                return false;
            }
        }

        startHistory();
        std::vector<double> xOld(static_cast<std::size_t>(n), 0.0);
        std::vector<double> xNew(static_cast<std::size_t>(n), 0.0);
        x.assign(static_cast<std::size_t>(n), 0.0);

        double omega = relaxation;
        if (omega <= 0.0)
        {
            omega = 1.0;
        }
        if (omega > 1.0)
        {
            omega = 1.0;
        }

        for (int iter = 0; iter < maxIter; ++iter)
        {
            double maxDiff = 0.0;
#ifdef _OPENMP
#pragma omp parallel
            {
                double threadMaxDiff = 0.0;
#pragma omp for nowait
                for (int i = 0; i < n; ++i)
                {
                    const double diag = a[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)];
                    if (std::abs(diag) < 1e-15)
                    {
                        threadMaxDiff = std::max(threadMaxDiff, 1e100);
                        continue;
                    }
                    double s = b[static_cast<std::size_t>(i)];
                    for (int j = 0; j < i; ++j)
                    {
                        s -= a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                             xOld[static_cast<std::size_t>(j)];
                    }
                    for (int j = i + 1; j < n; ++j)
                    {
                        s -= a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                             xOld[static_cast<std::size_t>(j)];
                    }
                    const double xi = s / diag;
                    const double relaxed =
                        xOld[static_cast<std::size_t>(i)] +
                        omega * (xi - xOld[static_cast<std::size_t>(i)]);
                    xNew[static_cast<std::size_t>(i)] = relaxed;
                    threadMaxDiff = std::max(threadMaxDiff,
                                             std::abs(relaxed - xOld[static_cast<std::size_t>(i)]));
                }
#pragma omp critical
                {
                    maxDiff = std::max(maxDiff, threadMaxDiff);
                }
            }
#else
            for (int i = 0; i < n; ++i)
            {
                const double diag = a[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)];
                if (std::abs(diag) < 1e-15)
                {
                    maxDiff = 1e100;
                    continue;
                }
                double s = b[static_cast<std::size_t>(i)];
                for (int j = 0; j < i; ++j)
                {
                    s -= a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                         xOld[static_cast<std::size_t>(j)];
                }
                for (int j = i + 1; j < n; ++j)
                {
                    s -= a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] *
                         xOld[static_cast<std::size_t>(j)];
                }
                const double xi = s / diag;
                const double relaxed =
                    xOld[static_cast<std::size_t>(i)] +
                    omega * (xi - xOld[static_cast<std::size_t>(i)]);
                xNew[static_cast<std::size_t>(i)] = relaxed;
                maxDiff = std::max(maxDiff,
                                   std::abs(relaxed - xOld[static_cast<std::size_t>(i)]));
            }
#endif
            xOld = xNew;
            x = xNew;
            const double residualNorm = computeResidualNorm(a, b, x);
            g_lastHistory.residuals.push_back(residualNorm);
            g_lastHistory.totalIterations = iter + 1;
            if (iterCallback)
            {
                iterCallback(iter);
            }
            if (residualNorm < tol || maxDiff < tol)
            {
                g_lastHistory.converged = true;
                return true;
            }
        }
        return false;
    }

    bool conjugateGradient(std::vector<std::vector<double>> &a,
                           std::vector<double> &b,
                           std::vector<double> &x,
                           double tol,
                           int maxIter,
                           double relaxation,
                           std::function<void(int)> iterCallback)
    {
        const int n = static_cast<int>(a.size());
        if (n == 0 || static_cast<int>(b.size()) != n)
        {
            return false;
        }

        startHistory();
        if (static_cast<int>(x.size()) != n)
        {
            x.assign(static_cast<std::size_t>(n), 0.0);
        }

        const LinearSolver::SparseMatrix as = LinearSolver::SparseMatrix::fromDense(a);
        std::vector<double> r(static_cast<std::size_t>(n), 0.0);
        std::vector<double> ax;
        std::vector<double> p(static_cast<std::size_t>(n), 0.0);
        std::vector<double> ap;

        as.matvec(x, ax);
        for (int i = 0; i < n; ++i)
        {
            r[static_cast<std::size_t>(i)] =
                b[static_cast<std::size_t>(i)] - ax[static_cast<std::size_t>(i)];
        }
        p = r;
        double rr = LinearSolver::dot(r, r);
        const double rr0 = std::max(rr, 1e-30);

        double omega = relaxation;
        if (omega <= 0.0)
        {
            omega = 1.0;
        }
        if (omega > 1.0)
        {
            omega = 1.0;
        }

        for (int iter = 0; iter < maxIter; ++iter)
        {
            as.matvec(p, ap);
            const double pap = LinearSolver::dot(p, ap);
            if (std::abs(pap) < 1e-20)
            {
                return false;
            }

            const double alpha = rr / pap;
            const double step = omega * alpha;
            LinearSolver::saxpy(step, p, x);
            LinearSolver::saxpy(-step, ap, r);

            const double rrNew = LinearSolver::dot(r, r);
            const double residual = std::sqrt(rrNew);
            g_lastHistory.residuals.push_back(residual);
            g_lastHistory.totalIterations = iter + 1;
            if (iterCallback)
            {
                iterCallback(iter);
            }

            if (rrNew < tol * tol * rr0)
            {
                g_lastHistory.converged = true;
                return true;
            }

            const double beta = rrNew / rr;
            rr = rrNew;
            for (int i = 0; i < n; ++i)
            {
                p[static_cast<std::size_t>(i)] =
                    r[static_cast<std::size_t>(i)] +
                    beta * p[static_cast<std::size_t>(i)];
            }
        }
        return false;
    }

} // namespace

namespace LinearSolver
{

    void setComputeBackend(ComputeBackend backend)
    {
        g_computeBackend = backend;
    }

    ComputeBackend getComputeBackend()
    {
        return g_computeBackend;
    }

    void setMaxThreads(int threads)
    {
        g_maxThreads = (threads > 0) ? threads : 0;
        applyOpenMpSettings();
    }

    int getMaxThreads()
    {
        return g_maxThreads;
    }

    int getRuntimeMaxThreads()
    {
#ifdef _OPENMP
        return omp_get_max_threads();
#else
        return 1;
#endif
    }

    bool solve(std::vector<std::vector<double>> &a,
               std::vector<double> &b,
               std::vector<double> &x,
               Method method,
               double tol,
               int maxIter,
               double relaxation,
               std::function<void(int)> iterCallback)
    {
        applyOpenMpSettings();

        if (method == Method::GaussianElimination)
        {
            startHistory();
            const bool ok = gaussianElimination(a, b, x);
            g_lastHistory.totalIterations = ok ? 1 : 0;
            g_lastHistory.converged = ok;
            if (ok)
            {
                g_lastHistory.residuals.push_back(computeResidualNorm(a, b, x));
            }
            return ok;
        }
        if (method == Method::ParallelGaussian)
        {
            startHistory();
            const bool ok = parallelGaussianElimination(a, b, x);
            g_lastHistory.totalIterations = ok ? 1 : 0;
            g_lastHistory.converged = ok;
            if (ok)
            {
                g_lastHistory.residuals.push_back(computeResidualNorm(a, b, x));
            }
            return ok;
        }
        if (method == Method::GaussSeidel)
        {
            return gaussSeidel(a, b, x, tol, maxIter, relaxation, iterCallback);
        }
        if (method == Method::JacobiParallel)
        {
            return jacobiParallel(a, b, x, tol, maxIter, relaxation, iterCallback);
        }
        if (method == Method::ConjugateGradient)
        {
            return conjugateGradient(a, b, x, tol, maxIter, relaxation, iterCallback);
        }
        if (method == Method::CudaDense)
        {
            startHistory();
            bool ok = false;

            if (g_computeBackend == ComputeBackend::CpuOnly)
            {
                ok = gaussianElimination(a, b, x);
            }
            else
            {
                std::string errorMessage;
                ok = LinearSolver::Cuda::solveDense(a, b, x, &errorMessage);
                if (!ok)
                {
                    if (g_computeBackend == ComputeBackend::GpuOnly)
                    {
                        std::cerr << "CUDA dense solver unavailable (" << errorMessage
                                  << "). GPU-only backend selected." << std::endl;
                        g_lastHistory.totalIterations = 0;
                        g_lastHistory.converged = false;
                        return false;
                    }

                    std::cerr << "CUDA dense solver unavailable (" << errorMessage
                              << "). Falling back to GaussianElimination." << std::endl;
                    ok = gaussianElimination(a, b, x);
                }
            }
            g_lastHistory.totalIterations = ok ? 1 : 0;
            g_lastHistory.converged = ok;
            if (ok)
            {
                g_lastHistory.residuals.push_back(computeResidualNorm(a, b, x));
            }
            return ok;
        }
        return false;
    }

    const IterationHistory &getLastIterationHistory()
    {
        return g_lastHistory;
    }

    void resetIterationHistory()
    {
        startHistory();
    }

    // Sparse matrix solver using Eigen SparseLU
#ifndef FLOW_NO_EIGEN3
    bool solveSparse(std::vector<Triplet> &triplets,
                     std::size_t matrixSize,
                     std::vector<double> &b,
                     std::vector<double> &x,
                     double tol)
    {
        try
        {
            // Construct sparse matrix from triplet list
            SpMat A(matrixSize, matrixSize);
            A.setFromTriplets(triplets.begin(), triplets.end());
            A.makeCompressed();

            // Convert vector b to Eigen vector
            Eigen::Map<Eigen::VectorXd> b_eigen(b.data(), static_cast<int>(b.size()));

            // Solve using SparseLU
            Eigen::SparseLU<SpMat> solver;
            solver.analyzePattern(A);
            if (solver.info() != Eigen::Success)
            {
                std::cerr << "Sparse matrix analysis failed." << std::endl;
                return false;
            }

            solver.factorize(A);
            if (solver.info() != Eigen::Success)
            {
                std::cerr << "Sparse matrix factorization failed." << std::endl;
                return false;
            }

            // Allocate solution vector if needed
            if (x.size() != matrixSize)
            {
                x.resize(matrixSize);
            }
            Eigen::Map<Eigen::VectorXd> x_eigen(x.data(), static_cast<int>(x.size()));

            // Solve the system
            x_eigen = solver.solve(b_eigen);
            if (solver.info() != Eigen::Success)
            {
                std::cerr << "Sparse matrix solve failed." << std::endl;
                return false;
            }

            startHistory();
            g_lastHistory.totalIterations = 1;
            g_lastHistory.converged = true;
            g_lastHistory.residuals.push_back(0.0);
            return true;
        }
        catch (const std::exception &ex)
        {
            std::cerr << "Exception in sparse solver: " << ex.what() << std::endl;
            return false;
        }
    }
#else
    bool solveSparse(std::vector<Triplet> & /*triplets*/,
                     std::size_t /*matrixSize*/,
                     std::vector<double> & /*b*/,
                     std::vector<double> & /*x*/,
                     double /*tol*/)
    {
        std::cerr << "Sparse solver is disabled because Eigen3 is not available." << std::endl;
        return false;
    }
#endif

} // namespace LinearSolver
