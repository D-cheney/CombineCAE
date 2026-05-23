#ifndef SIMPLE_ITERATION_SOLVER_H
#define SIMPLE_ITERATION_SOLVER_H

#include "../solver_method.h"

class SimpleIterationSolverMethod : public ISteadySolverMethod {
public:
    SteadySolverType kind() const override;
    const char* name() const override;
    bool solve(SolverContext& context) override;
};

#endif // SIMPLE_ITERATION_SOLVER_H