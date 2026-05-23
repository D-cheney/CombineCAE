#ifndef SOLVER_METHOD_H
#define SOLVER_METHOD_H

#include "solver_context.h"

enum class SteadySolverType {
    SimpleIteration,
    NewtonRaphson,
    LinearGaussianElimination,
    LinearGaussSeidel,
    LinearParallelGaussian,
    LinearJacobiParallel,
    LinearConjugateGradient,
    LinearCudaDense
};

class ISteadySolverMethod {
public:
    virtual ~ISteadySolverMethod() = default;

    virtual SteadySolverType kind() const = 0;
    virtual const char* name() const = 0;
    virtual bool solve(SolverContext& context) = 0;
};

#endif // SOLVER_METHOD_H
