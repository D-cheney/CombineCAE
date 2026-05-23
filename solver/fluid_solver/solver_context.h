#ifndef SOLVER_CONTEXT_H
#define SOLVER_CONTEXT_H

#include "core/component.h"
#include "core/material_properties.h"
#include "core/node.h"
#include "utils/boundary_conditions.h"
#include "utils/coupling/coupling_config.h"
#include "linear_solver.h"

#include <map>
#include <memory>
#include <vector>

struct ComponentConnection
{
    int componentIndex;
    int fromNodeId;
    int toNodeId;
    int fromPortIndex;
    int toPortIndex;

    ComponentConnection(int compIndex = -1,
                        int from = -1,
                        int to = -1,
                        int fromPort = 0,
                        int toPort = 1)
        : componentIndex(compIndex),
          fromNodeId(from),
          toNodeId(to),
          fromPortIndex(fromPort),
          toPortIndex(toPort) {}
};

struct ComponentIterationResult
{
    int componentIndex;
    std::string type;
    std::string name;
    int fromNodeId;
    int toNodeId;
    int fromPortIndex;
    int toPortIndex;
    double flow;
    double pressureDrop;
    double resistance;
    double heatTransfer;
};

struct IterationComponentSnapshot
{
    int iteration;
    std::vector<ComponentIterationResult> components;
};

struct SolverContext
{
    std::vector<std::unique_ptr<Node>> &nodes;
    std::vector<std::unique_ptr<Component>> &components;
    std::vector<BoundaryCondition> &boundaryConditions;
    std::map<int, std::vector<int>> &connectivity;
    std::vector<ComponentConnection> &componentConnections;
    MaterialProperties &workingFluid;
    std::vector<double> &latestConnectionFlows;
    LinearSolver::IterationHistory &iterationHistory;

    double convergenceTolerance;
    int maxIterations;
    double relaxationFactor;

    // Enhanced convergence parameters for Newton-Raphson
    double relativeConvergenceTolerance; // Relative residual tolerance
    double solutionIncrementTolerance;   // Solution increment tolerance

    bool debugEnabled;
    std::vector<IterationComponentSnapshot> *debugIterations;

    const CouplingConfig *couplingConfig;
    double currentTime;

    // Constructor
    SolverContext(
        std::vector<std::unique_ptr<Node>> &n,
        std::vector<std::unique_ptr<Component>> &c,
        std::vector<BoundaryCondition> &bc,
        std::map<int, std::vector<int>> &conn,
        std::vector<ComponentConnection> &cc,
        MaterialProperties &wf,
        std::vector<double> &lf,
        LinearSolver::IterationHistory &ih,
        double convTol = 1e-6,
        int maxIter = 100,
        double relax = 1.0,
        double relConvTol = 1e-8,
        double solIncrTol = 1e-8,
        bool dbg = false,
        std::vector<IterationComponentSnapshot> *dbgIter = nullptr,
        const CouplingConfig *coupCfg = nullptr,
        double curTime = 0.0)
        : nodes(n), components(c), boundaryConditions(bc), connectivity(conn),
          componentConnections(cc), workingFluid(wf), latestConnectionFlows(lf),
          iterationHistory(ih), convergenceTolerance(convTol), maxIterations(maxIter),
          relaxationFactor(relax), relativeConvergenceTolerance(relConvTol),
          solutionIncrementTolerance(solIncrTol), debugEnabled(dbg),
          debugIterations(dbgIter), couplingConfig(coupCfg), currentTime(curTime) {}
};

#endif // SOLVER_CONTEXT_H
