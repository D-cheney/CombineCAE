#ifndef SOLVER_BASE_H
#define SOLVER_BASE_H

#include "solver_context.h"
#include "solver_method.h"
#include "transient_solver.h"
#include "coupling_solver.h"
#include "utils/coupling/coupling_config.h"
#include "utils/error_handler.h"
#include "utils/logger.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

enum class CalculationType
{
    SteadyState,
    Transient
};

enum class ComputeBackend
{
    CpuOnly,
    CpuPlusGpu,
    GpuOnly
};

struct TimeStepSnapshot
{
    double time;
    std::vector<double> nodePressures;
    std::vector<double> nodeTemperatures;
    std::vector<double> connectionFlows;
    LinearSolver::IterationHistory iterationHistory;
    std::vector<IterationComponentSnapshot> iterationComponentResults;
};

class FluidNetworkSolver
{
private:
    std::vector<std::unique_ptr<Node>> nodes;
    std::vector<std::unique_ptr<Component>> components;
    std::vector<BoundaryCondition> boundaryConditions;
    std::map<int, std::vector<int>> connectivity;
    std::vector<ComponentConnection> componentConnections;
    std::vector<double> latestConnectionFlows;
    LinearSolver::IterationHistory lastIterationHistory;
    std::vector<IterationComponentSnapshot> lastIterationComponentResults;
    std::vector<TimeStepSnapshot> timeStepSnapshots;

    MaterialProperties workingFluid;
    CalculationType calcType;
    SteadySolverType steadySolverType;

    double timeStep;
    double convergenceTolerance;
    int maxIterations;
    double relaxationFactor;

    // Enhanced convergence parameters for Newton-Raphson
    double relativeConvergenceTolerance;
    double solutionIncrementTolerance;

    double currentTime;
    bool debugEnabled;
    std::vector<IterationComponentSnapshot> *activeDebugTarget;
    CouplingConfig couplingConfig;
    int cpuThreads;
    ComputeBackend computeBackend;

    std::map<SteadySolverType, std::unique_ptr<ISteadySolverMethod>> steadySolvers;

    // 瞬态求解器
    std::unique_ptr<TransientSolver> transientSolver;
    TransientConfig transientConfig;

    // 耦合求解器
    std::unique_ptr<CouplingSolver> couplingSolver;

    // 错误处理器
    std::unique_ptr<ErrorHandler> errorHandler;

public:
    FluidNetworkSolver(MaterialProperties fluid = MaterialProperties(),
                       CalculationType ct = CalculationType::SteadyState);

    void addNode(std::unique_ptr<Node> node);
    void addComponent(std::unique_ptr<Component> comp);
    void addBoundaryCondition(BoundaryCondition bc);

    void connectNodes(int node1Id, int node2Id);
    bool connectComponent(int compIndex,
                          int fromNodeId,
                          int toNodeId,
                          int fromPortIndex = 0,
                          int toPortIndex = 1,
                          bool allowMultiple = false);

    void setCalculationParameters(double dt, double tolerance, int maxIter, double relaxation = 1.0);

    // Enhanced convergence parameters for Newton-Raphson solver
    void setConvergenceParameters(double absTolerance, double relTolerance, double dxTolerance)
    {
        convergenceTolerance = absTolerance;
        relativeConvergenceTolerance = relTolerance;
        solutionIncrementTolerance = dxTolerance;
    }

    double getAbsoluteConvergenceTolerance() const { return convergenceTolerance; }
    double getRelativeConvergenceTolerance() const { return relativeConvergenceTolerance; }
    double getSolutionIncrementTolerance() const { return solutionIncrementTolerance; }

    void setSteadySolverType(SteadySolverType solverType);
    void setDebugEnabled(bool enabled) { debugEnabled = enabled; }
    bool isDebugEnabled() const { return debugEnabled; }
    void setCouplingConfig(const CouplingConfig &config) { couplingConfig = config; }
    const CouplingConfig &getCouplingConfig() const { return couplingConfig; }

    // CPU parallel configuration (OpenMP).
    void setCpuThreads(int threads);
    int getCpuThreads() const { return cpuThreads; }

    // Compute backend selection.
    void setComputeBackend(ComputeBackend backend) { computeBackend = backend; }
    ComputeBackend getComputeBackend() const { return computeBackend; }
    bool setComputeBackendByName(const std::string &name);

    bool solve();
    bool solveTransient(double totalTime);
    bool solveNewtonRaphson();
    bool setSteadySolverTypeByName(const std::string &name);

    // 瞬态求解器配置
    void setTransientConfig(const TransientConfig &config);
    const TransientConfig &getTransientConfig() const { return transientConfig; }
    void setTimeIntegrationMethod(TimeIntegrationMethod method);
    void setAdaptiveTimeStep(bool enabled);
    void setConservationTolerance(double tolerance);

    // 耦合求解器配置
    void setCouplingMode(CouplingMode mode);
    void setMaxOuterIterations(int maxIter);
    void setOuterConvergenceTolerance(double tolerance);
    void setUnderRelaxationFactor(double factor);
    bool solveWithCoupling();

    // 错误处理和诊断
    void setErrorCallback(ErrorCallback callback);
    bool checkNetworkTopology();
    bool checkMatrixCondition();
    const std::vector<ErrorInfo>& getErrorHistory() const;

    Node *getNode(int nodeId);
    const Node *getNode(int nodeId) const;
    Component *getComponent(int compIndex);
    const Component *getComponent(int compIndex) const;

    const std::vector<std::unique_ptr<Node>> &getNodes() const { return nodes; }
    const std::vector<std::unique_ptr<Component>> &getComponents() const { return components; }
    const std::vector<ComponentConnection> &getComponentConnections() const { return componentConnections; }
    const std::vector<BoundaryCondition> &getBoundaryConditions() const { return boundaryConditions; }
    const std::vector<double> &getLatestConnectionFlows() const { return latestConnectionFlows; }
    const LinearSolver::IterationHistory &getLastIterationHistory() const { return lastIterationHistory; }
    const std::vector<IterationComponentSnapshot> &getLastIterationComponentResults() const
    {
        return lastIterationComponentResults;
    }
    const std::vector<TimeStepSnapshot> &getTimeStepSnapshots() const { return timeStepSnapshots; }
    double getCurrentTime() const { return currentTime; }
    SteadySolverType getSteadySolverType() const { return steadySolverType; }
    const MaterialProperties &getWorkingFluid() const { return workingFluid; }

    /**
     * @brief Replace the solver's working fluid properties at runtime.
     */
    void setWorkingFluid(const MaterialProperties &fluid) { workingFluid = fluid; }

    void updateComponents(double deltaTime = 0.0);
    bool validateNetwork();
    void printNetworkInfo();
    void reset();
    void clearTimeStepSnapshots();
    void clearNetwork();

private:
    SolverContext buildContext();
    bool solveSteadyState(std::vector<IterationComponentSnapshot> *debugTarget = nullptr);
    bool hasNode(int nodeId) const;
    void addConnectivityEdge(int fromNodeId, int toNodeId);
    void saveTimeStepSnapshot(double time,
                              const std::vector<IterationComponentSnapshot> *debugResults = nullptr);
};

#endif // SOLVER_BASE_H
