#ifndef NEWTON_RAPHSON_SOLVER_H
#define NEWTON_RAPHSON_SOLVER_H

#include "../solver_method.h"
#include "../linear_solver.h"
#include "utils/matrix_vector.h"

#include <vector>

class NewtonRaphsonSolverMethod : public ISteadySolverMethod
{
public:
    SteadySolverType kind() const override;
    const char *name() const override;
    bool solve(SolverContext &context) override;

private:
    // Evaluate residuals for the current state
    Vector evaluateResiduals(SolverContext &context,
                             const std::vector<int> &internalNodeIds,
                             const Vector &variables) const;

    // Calculate Jacobian using finite differences and store as sparse triplets
    std::vector<LinearSolver::Triplet> calculateJacobianSparse(
        SolverContext &context,
        const std::vector<int> &internalNodeIds,
        const Vector &variables) const;

    // Legacy dense Jacobian calculation (kept for backward compatibility)
    Matrix calculateJacobian(SolverContext &context,
                             const std::vector<int> &internalNodeIds,
                             const Vector &variables) const;

    // Solve linear system with dense matrix
    Vector solveLinearSystem(const Matrix &a, const Vector &b) const;

    // Solve linear system with sparse matrix
    Vector solveLinearSystemSparse(const std::vector<LinearSolver::Triplet> &triplets,
                                   std::size_t matrixSize,
                                   const Vector &b) const;

    // Armijo backtracking line search for global convergence
    double armijoLineSearch(SolverContext &context,
                            const std::vector<int> &internalNodeIds,
                            Vector &variables,
                            const Vector &searchDirection,
                            double initialResidualNorm,
                            double c = 1e-4,
                            double rho = 0.5) const;

    // Compute L2 norm of vector
    double l2Norm(const Vector &v) const;

    // Compute relative residual norm
    double computeRelativeResidual(const Vector &current, const Vector &previous) const;
};

#endif // NEWTON_RAPHSON_SOLVER_H
