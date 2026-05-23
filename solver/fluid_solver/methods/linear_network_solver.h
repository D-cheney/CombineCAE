#ifndef LINEAR_NETWORK_SOLVER_H
#define LINEAR_NETWORK_SOLVER_H

#include "../linear_solver.h"
#include "../solver_method.h"

#include <string>

class LinearNetworkSolverMethod : public ISteadySolverMethod {
private:
    SteadySolverType kind_;
    LinearSolver::Method linearMethod_;
    std::string name_;

public:
    LinearNetworkSolverMethod(SteadySolverType kind,
                              LinearSolver::Method linearMethod,
                              const std::string& name);

    SteadySolverType kind() const override;
    const char* name() const override;
    bool solve(SolverContext& context) override;

private:
    bool solveHydraulic(SolverContext& context);
    bool solveThermal(SolverContext& context);
};

#endif // LINEAR_NETWORK_SOLVER_H
