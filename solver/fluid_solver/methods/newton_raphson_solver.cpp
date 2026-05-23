#include "newton_raphson_solver.h"

#include "utils/coupling/coupling_io.h"
#include "../solver_context_utils.h"

#include <cmath>
#include <iostream>
#include <stdexcept>
#include <vector>
#include <algorithm>

#ifdef _OPENMP
#include <omp.h>
#endif

SteadySolverType NewtonRaphsonSolverMethod::kind() const
{
    return SteadySolverType::NewtonRaphson;
}

const char *NewtonRaphsonSolverMethod::name() const
{
    return "NewtonRaphson";
}

bool NewtonRaphsonSolverMethod::solve(SolverContext &context)
{
    try
    {
        context.iterationHistory.residuals.clear();
        context.iterationHistory.totalIterations = 0;
        context.iterationHistory.converged = false;

        std::vector<int> internalNodeIds = getInternalNodeIds(context);

        if (internalNodeIds.empty())
        {
            applyBoundaryConditions(context);
            context.iterationHistory.totalIterations = 0;
            context.iterationHistory.converged = true;
            return true;
        }

        Vector variables(internalNodeIds.size());
        for (std::size_t i = 0; i < internalNodeIds.size(); ++i)
        {
            const Node *node = findNodeById(context, internalNodeIds[i]);
            variables[i] = (node != nullptr) ? node->pressure : 101325.0;
        }

        // Enhanced convergence parameters
        double tolAbs = context.convergenceTolerance;
        double tolRel = 1e-6; // Relative residual tolerance
        double tolDx = 1e-6;  // Solution increment tolerance
        double dampingFactor = 1.0;
        int noImprovementCount = 0;

        Vector previousVariables = variables;
        for (int iter = 0; iter < context.maxIterations; ++iter)
        {
            if (context.couplingConfig && context.couplingConfig->enabled)
            {
                if (context.couplingConfig->readEveryIteration || iter == 0)
                {
                    CouplingIO::applyCouplingInputs(context,
                                                    *context.couplingConfig,
                                                    context.currentTime,
                                                    iter + 1);
                }
            }
            // Evaluate residuals
            const Vector residuals = evaluateResiduals(context, internalNodeIds, variables);
            const double residualNorm = l2Norm(residuals);
            const double relativeResidual = (iter == 0) ? 1.0 : residualNorm / std::max(context.convergenceTolerance, 1e-12);
            context.iterationHistory.residuals.push_back(residualNorm);
            context.iterationHistory.totalIterations = iter + 1;
            recordIterationComponentResults(context, iter + 1);

            if (context.couplingConfig && context.couplingConfig->enabled &&
                context.couplingConfig->writeEveryIteration)
            {
                CouplingIO::writeCouplingOutputs(context,
                                                 *context.couplingConfig,
                                                 context.currentTime,
                                                 iter + 1);
            }

            // Check convergence with enhanced criteria
            Vector solutionIncrement = variables - previousVariables;
            double incrementNorm = l2Norm(solutionIncrement);
            double relativeIncrement = (l2Norm(variables) > 1e-12) ? (incrementNorm / l2Norm(variables)) : incrementNorm;

            bool absConverged = residualNorm < tolAbs;
            bool relConverged = relativeResidual < tolRel;
            bool incConverged = incrementNorm > 1e-15 ? (relativeIncrement < tolDx) : true;

            if (absConverged && relConverged && incConverged)
            {
                for (std::size_t i = 0; i < internalNodeIds.size(); ++i)
                {
                    Node *node = findNodeById(context, internalNodeIds[i]);
                    if (node != nullptr)
                    {
                        node->pressure = variables[i];
                    }
                }

                context.iterationHistory.converged = true;
                computeAllConnectionFlows(context);
                std::cout << "Steady solution converged after " << (iter + 1)
                          << " iterations using " << name() << "."
                          << " (Abs residual: " << residualNorm << ")" << std::endl;
                return true;
            }

            // Use sparse Jacobian for better performance
            const std::vector<LinearSolver::Triplet> jacobianTriplets =
                calculateJacobianSparse(context, internalNodeIds, variables);
            const Vector rhs = residuals * (-1.0);
            Vector delta = solveLinearSystemSparse(jacobianTriplets, internalNodeIds.size(), rhs);

            if (delta.size() != variables.size())
            {
                std::cerr << "Linear solver returned incorrect solution size!" << std::endl;
                return false;
            }

            // Adaptive damping strategy
            dampingFactor = std::min(1.0, dampingFactor + 0.1);

            // Armijo line search for global convergence
            double actualDampingFactor = armijoLineSearch(context, internalNodeIds, variables, delta, residualNorm);

            // Check for convergence quality
            if (actualDampingFactor < 1e-12)
            {
                std::cerr << "Warning: Line search failed to find acceptable step." << std::endl;
                noImprovementCount++;
                if (noImprovementCount >= 3)
                {
                    std::cerr << "Newton-Raphson stalled with no improvement for 3 iterations." << std::endl;
                    return false;
                }
            }
            else
            {
                noImprovementCount = 0;
            }

            previousVariables = variables;
        }
    }
    catch (const std::exception &ex)
    {
        std::cerr << "Newton-Raphson solver error: " << ex.what() << std::endl;
        return false;
    }

    std::cerr << "Steady solution failed to converge after " << context.maxIterations
              << " iterations using " << name() << "." << std::endl;
    return false;
}

Vector NewtonRaphsonSolverMethod::evaluateResiduals(SolverContext &context,
                                                    const std::vector<int> &internalNodeIds,
                                                    const Vector &variables) const
{
    for (std::size_t i = 0; i < internalNodeIds.size(); ++i)
    {
        Node *node = findNodeById(context, internalNodeIds[i]);
        if (node != nullptr)
        {
            node->pressure = variables[i];
        }
    }

    applyBoundaryConditions(context);

    // Parallel residual calculation for independent components
    Vector residuals(internalNodeIds.size());
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < static_cast<int>(internalNodeIds.size()); ++i)
    {
        residuals[i] = computeNetFlowAtNode(context, internalNodeIds[i]);
    }

    return residuals;
}

std::vector<LinearSolver::Triplet> NewtonRaphsonSolverMethod::calculateJacobianSparse(
    SolverContext &context,
    const std::vector<int> &internalNodeIds,
    const Vector &variables) const
{
    const std::size_t n = internalNodeIds.size();
    std::vector<LinearSolver::Triplet> triplets;

    // Estimate required triplets based on typical network sparsity (~5% density)
    triplets.reserve(static_cast<std::size_t>(n * n * 0.05) + n);

    const double h = 1e-6;

    // Compute Jacobian using finite differences
    for (std::size_t j = 0; j < n; ++j)
    {
        Vector xPlus = variables;
        xPlus[j] += h;
        const Vector fPlus = evaluateResiduals(context, internalNodeIds, xPlus);

        Vector xMinus = variables;
        xMinus[j] -= h;
        const Vector fMinus = evaluateResiduals(context, internalNodeIds, xMinus);

        for (std::size_t i = 0; i < n; ++i)
        {
            double jac_val = (fPlus[i] - fMinus[i]) / (2.0 * h);
            if (std::abs(jac_val) > 1e-15)
            { // Only store non-zero elements
                triplets.emplace_back(static_cast<int>(i), static_cast<int>(j), jac_val);
            }
        }
    }

    return triplets;
}

Matrix NewtonRaphsonSolverMethod::calculateJacobian(SolverContext &context,
                                                    const std::vector<int> &internalNodeIds,
                                                    const Vector &variables) const
{
    const std::size_t n = internalNodeIds.size();
    Matrix jacobian(n, n);

    const double h = 1e-6;
    for (std::size_t j = 0; j < n; ++j)
    {
        Vector xPlus = variables;
        xPlus[j] += h;
        const Vector fPlus = evaluateResiduals(context, internalNodeIds, xPlus);

        Vector xMinus = variables;
        xMinus[j] -= h;
        const Vector fMinus = evaluateResiduals(context, internalNodeIds, xMinus);

        for (std::size_t i = 0; i < n; ++i)
        {
            jacobian[i][j] = (fPlus[i] - fMinus[i]) / (2.0 * h);
        }
    }

    return jacobian;
}

Vector NewtonRaphsonSolverMethod::solveLinearSystemSparse(
    const std::vector<LinearSolver::Triplet> &triplets,
    std::size_t matrixSize,
    const Vector &b) const
{
    std::vector<double> b_vec(b.size());
    for (std::size_t i = 0; i < b.size(); ++i)
    {
        b_vec[i] = b[i];
    }

    std::vector<double> x(matrixSize);
    const_cast<std::vector<LinearSolver::Triplet> &>(triplets); // Cast away const for solve

    bool success = LinearSolver::solveSparse(
        const_cast<std::vector<LinearSolver::Triplet> &>(triplets),
        matrixSize,
        b_vec,
        x);

    if (!success)
    {
        // Fallback to dense solver if sparse fails
        std::cerr << "Sparse solver failed, attempting fallback..." << std::endl;

        // Convert sparse matrix to dense for fallback
        std::vector<std::vector<double>> A_dense(matrixSize, std::vector<double>(matrixSize, 0.0));
        for (const auto &triplet : triplets)
        {
#ifdef FLOW_NO_EIGEN3
            A_dense[triplet.row][triplet.col] = triplet.value;
#else
            A_dense[triplet.row()][triplet.col()] = triplet.value();
#endif
        }

        const bool fallback_ok = LinearSolver::solve(A_dense, b_vec, x, LinearSolver::Method::GaussianElimination);
        if (!fallback_ok)
        {
            throw std::runtime_error("Both sparse and dense linear solvers failed.");
        }
    }

    Vector result(x.size());
    for (std::size_t i = 0; i < x.size(); ++i)
    {
        result[i] = x[i];
    }
    return result;
}

double NewtonRaphsonSolverMethod::armijoLineSearch(
    SolverContext &context,
    const std::vector<int> &internalNodeIds,
    Vector &variables,
    const Vector &searchDirection,
    double initialResidualNorm,
    double c,
    double rho) const
{

    double alpha = 1.0;
    const double minAlpha = 1e-12;
    int iterations = 0;
    const int maxIterations = 50;

    Vector testVariables = variables;

    while (alpha > minAlpha && iterations < maxIterations)
    {
        // Compute test point
        for (std::size_t i = 0; i < testVariables.size(); ++i)
        {
            testVariables[i] = variables[i] + alpha * searchDirection[i];
        }

        // Evaluate residuals at test point
        const Vector testResiduals = evaluateResiduals(context, internalNodeIds, testVariables);
        const double testResidualNorm = l2Norm(testResiduals);

        // Check Armijo condition: f(x + alpha*d) <= f(x) * (1 - c*alpha)
        if (testResidualNorm <= initialResidualNorm * (1.0 - c * alpha))
        {
            variables = testVariables;
            return alpha;
        }

        // Backtrack
        alpha *= rho;
        iterations++;
    }

    // If line search fails, accept the step with final alpha value
    if (alpha <= minAlpha)
    {
        std::cerr << "Warning: Armijo line search converged to minimum step size." << std::endl;
        return minAlpha;
    }

    return alpha;
}

Vector NewtonRaphsonSolverMethod::solveLinearSystem(const Matrix &a, const Vector &b) const
{
    const std::size_t n = a.getRows();
    if (n == 0 || a.getCols() != n || b.size() != n)
    {
        throw std::invalid_argument("Invalid linear system dimensions.");
    }

    std::vector<std::vector<double>> augmented(
        n, std::vector<double>(n + 1, 0.0));

    for (std::size_t i = 0; i < n; ++i)
    {
        for (std::size_t j = 0; j < n; ++j)
        {
            augmented[i][j] = a[i][j];
        }
        augmented[i][n] = b[i];
    }

    for (std::size_t i = 0; i < n; ++i)
    {
        std::size_t pivot = i;
        for (std::size_t r = i + 1; r < n; ++r)
        {
            if (std::abs(augmented[r][i]) > std::abs(augmented[pivot][i]))
            {
                pivot = r;
            }
        }

        if (std::abs(augmented[pivot][i]) < 1e-14)
        {
            throw std::runtime_error("Singular Jacobian encountered.");
        }

        if (pivot != i)
        {
            std::swap(augmented[i], augmented[pivot]);
        }

        for (std::size_t r = i + 1; r < n; ++r)
        {
            const double factor = augmented[r][i] / augmented[i][i];
            for (std::size_t c = i; c <= n; ++c)
            {
                augmented[r][c] -= factor * augmented[i][c];
            }
        }
    }

    Vector x(n);
    for (int i = static_cast<int>(n) - 1; i >= 0; --i)
    {
        double value = augmented[static_cast<std::size_t>(i)][n];
        for (std::size_t c = static_cast<std::size_t>(i) + 1; c < n; ++c)
        {
            value -= augmented[static_cast<std::size_t>(i)][c] * x[c];
        }
        x[static_cast<std::size_t>(i)] = value / augmented[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)];
    }

    return x;
}

double NewtonRaphsonSolverMethod::l2Norm(const Vector &v) const
{
    double sum = 0.0;
    for (std::size_t i = 0; i < v.size(); ++i)
    {
        sum += v[i] * v[i];
    }
    return std::sqrt(sum);
}

double NewtonRaphsonSolverMethod::computeRelativeResidual(
    const Vector &current, const Vector &previous) const
{
    Vector diff = current - previous;
    double diffNorm = l2Norm(diff);
    double prevNorm = l2Norm(previous);

    if (prevNorm < 1e-15)
    {
        return diffNorm;
    }
    return diffNorm / prevNorm;
}
