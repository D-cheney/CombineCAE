#include "fluid_solver.h"

#include "methods/linear_network_solver.h"
#include "methods/newton_raphson_solver.h"
#include "methods/simple_iteration_solver.h"
#include "solver_context_utils.h"

#include <algorithm>
#include <cmath>
#include <cctype>
#include <iomanip>
#include <iostream>

namespace
{

    std::string toLowerCopy(const std::string &text)
    {
        std::string out = text;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c)
                       { return static_cast<char>(std::tolower(c)); });
        return out;
    }

    void printIterationResiduals(const std::string &solverName,
                                 const LinearSolver::IterationHistory &history)
    {
        if (history.residuals.empty())
        {
            return;
        }

        std::cout << "Residual history (" << solverName << "):" << std::endl;
        std::cout << std::scientific << std::setprecision(8);
        for (std::size_t i = 0; i < history.residuals.size(); ++i)
        {
            std::cout << "  Iter " << (i + 1) << ": " << history.residuals[i] << std::endl;
        }
        std::cout << std::defaultfloat;
    }

} // namespace

FluidNetworkSolver::FluidNetworkSolver(MaterialProperties fluid, CalculationType ct)
    : workingFluid(fluid),
      calcType(ct),
      steadySolverType(SteadySolverType::LinearGaussSeidel),
      timeStep(0.1),
      convergenceTolerance(1e-6),
      maxIterations(200),
      relaxationFactor(1.0),
      relativeConvergenceTolerance(1e-6),
      solutionIncrementTolerance(1e-6),
      currentTime(0.0),
      debugEnabled(false),
      activeDebugTarget(nullptr),
      couplingConfig(),
      cpuThreads(0),
      computeBackend(ComputeBackend::CpuPlusGpu)
{
    std::unique_ptr<ISteadySolverMethod> simple(new SimpleIterationSolverMethod());
    std::unique_ptr<ISteadySolverMethod> nr(new NewtonRaphsonSolverMethod());

    std::unique_ptr<ISteadySolverMethod> lg(new LinearNetworkSolverMethod(
        SteadySolverType::LinearGaussianElimination,
        LinearSolver::Method::GaussianElimination,
        "LinearGaussianElimination"));
    std::unique_ptr<ISteadySolverMethod> lgs(new LinearNetworkSolverMethod(
        SteadySolverType::LinearGaussSeidel,
        LinearSolver::Method::GaussSeidel,
        "LinearGaussSeidel"));
    std::unique_ptr<ISteadySolverMethod> lpg(new LinearNetworkSolverMethod(
        SteadySolverType::LinearParallelGaussian,
        LinearSolver::Method::ParallelGaussian,
        "LinearParallelGaussian"));
    std::unique_ptr<ISteadySolverMethod> lj(new LinearNetworkSolverMethod(
        SteadySolverType::LinearJacobiParallel,
        LinearSolver::Method::JacobiParallel,
        "LinearJacobiParallel"));
    std::unique_ptr<ISteadySolverMethod> lcg(new LinearNetworkSolverMethod(
        SteadySolverType::LinearConjugateGradient,
        LinearSolver::Method::ConjugateGradient,
        "LinearConjugateGradient"));
    std::unique_ptr<ISteadySolverMethod> lcuda(new LinearNetworkSolverMethod(
        SteadySolverType::LinearCudaDense,
        LinearSolver::Method::CudaDense,
        "LinearCudaDense"));

    steadySolvers[simple->kind()] = std::move(simple);
    steadySolvers[nr->kind()] = std::move(nr);
    steadySolvers[lg->kind()] = std::move(lg);
    steadySolvers[lgs->kind()] = std::move(lgs);
    steadySolvers[lpg->kind()] = std::move(lpg);
    steadySolvers[lj->kind()] = std::move(lj);
    steadySolvers[lcg->kind()] = std::move(lcg);
    steadySolvers[lcuda->kind()] = std::move(lcuda);
}

void FluidNetworkSolver::addNode(std::unique_ptr<Node> node)
{
    if (!node)
    {
        return;
    }

    if (hasNode(node->id))
    {
        std::cerr << "Duplicate node id: " << node->id << std::endl;
        return;
    }

    nodes.push_back(std::move(node));
}

void FluidNetworkSolver::addComponent(std::unique_ptr<Component> comp)
{
    if (!comp)
    {
        return;
    }

    components.push_back(std::move(comp));
}

void FluidNetworkSolver::addBoundaryCondition(BoundaryCondition bc)
{
    boundaryConditions.push_back(bc);
}

void FluidNetworkSolver::connectNodes(int node1Id, int node2Id)
{
    if (!hasNode(node1Id) || !hasNode(node2Id))
    {
        std::cerr << "connectNodes ignored: invalid node ids (" << node1Id << ", "
                  << node2Id << ")." << std::endl;
        return;
    }

    addConnectivityEdge(node1Id, node2Id);
    addConnectivityEdge(node2Id, node1Id);

    for (std::size_t i = 0; i < components.size(); ++i)
    {
        if (!components[i]->hasConnection())
        {
            connectComponent(static_cast<int>(i), node1Id, node2Id);
            break;
        }
    }
}

bool FluidNetworkSolver::connectComponent(int compIndex,
                                          int fromNodeId,
                                          int toNodeId,
                                          int fromPortIndex,
                                          int toPortIndex,
                                          bool allowMultiple)
{
    if (compIndex < 0 || compIndex >= static_cast<int>(components.size()))
    {
        std::cerr << "connectComponent failed: invalid component index " << compIndex << std::endl;
        return false;
    }

    if (!hasNode(fromNodeId) || !hasNode(toNodeId))
    {
        std::cerr << "connectComponent failed: invalid node ids (" << fromNodeId << ", "
                  << toNodeId << ")." << std::endl;
        return false;
    }

    Component *comp = components[static_cast<std::size_t>(compIndex)].get();
    if (comp != nullptr && (!allowMultiple || !comp->hasConnection()))
    {
        comp->setConnection(fromNodeId, toNodeId);
    }

    if (!allowMultiple)
    {
        bool replaced = false;
        for (auto &connection : componentConnections)
        {
            if (connection.componentIndex == compIndex)
            {
                connection.fromNodeId = fromNodeId;
                connection.toNodeId = toNodeId;
                connection.fromPortIndex = fromPortIndex;
                connection.toPortIndex = toPortIndex;
                replaced = true;
                break;
            }
        }

        if (!replaced)
        {
            componentConnections.push_back(
                ComponentConnection(compIndex, fromNodeId, toNodeId, fromPortIndex, toPortIndex));
        }
    }
    else
    {
        bool exists = false;
        for (const auto &connection : componentConnections)
        {
            if (connection.componentIndex == compIndex &&
                connection.fromNodeId == fromNodeId &&
                connection.toNodeId == toNodeId &&
                connection.fromPortIndex == fromPortIndex &&
                connection.toPortIndex == toPortIndex)
            {
                exists = true;
                break;
            }
        }
        if (!exists)
        {
            componentConnections.push_back(
                ComponentConnection(compIndex, fromNodeId, toNodeId, fromPortIndex, toPortIndex));
        }
    }

    addConnectivityEdge(fromNodeId, toNodeId);
    addConnectivityEdge(toNodeId, fromNodeId);

    return true;
}

void FluidNetworkSolver::setCalculationParameters(double dt, double tolerance, int maxIter,
                                                  double relaxation)
{
    if (dt > 0.0)
    {
        timeStep = dt;
    }
    if (tolerance > 0.0)
    {
        convergenceTolerance = tolerance;
    }
    if (maxIter > 0)
    {
        maxIterations = maxIter;
    }
    if (relaxation > 0.0)
    {
        relaxationFactor = relaxation;
    }
    if (relaxationFactor <= 0.0)
    {
        relaxationFactor = 1.0;
    }
    if (relaxationFactor > 1.0)
    {
        relaxationFactor = 1.0;
    }
}

void FluidNetworkSolver::setSteadySolverType(SteadySolverType solverType)
{
    if (steadySolvers.find(solverType) == steadySolvers.end())
    {
        std::cerr << "Requested steady solver is not registered." << std::endl;
        return;
    }

    steadySolverType = solverType;
}

void FluidNetworkSolver::setCpuThreads(int threads)
{
    cpuThreads = (threads > 0) ? threads : 0;
    LinearSolver::setMaxThreads(cpuThreads);
}

bool FluidNetworkSolver::setComputeBackendByName(const std::string &name)
{
    const std::string key = toLowerCopy(name);
    if (key.empty())
    {
        return false;
    }

    if (key == "cpu" || key == "cpuonly" || key == "cpu_only" || key == "cpu-only")
    {
        computeBackend = ComputeBackend::CpuOnly;
        return true;
    }
    if (key == "hybrid" || key == "cpugpu" || key == "cpu+gpu" ||
        key == "cpu_gpu" || key == "cpu-gpu" ||
        key == "cpuplusgpu" || key == "cpu_plus_gpu")
    {
        computeBackend = ComputeBackend::CpuPlusGpu;
        return true;
    }
    if (key == "gpu" || key == "gpuonly" || key == "gpu_only" || key == "gpu-only")
    {
        computeBackend = ComputeBackend::GpuOnly;
        return true;
    }

    return false;
}

bool FluidNetworkSolver::setSteadySolverTypeByName(const std::string &name)
{
    const std::string key = toLowerCopy(name);

    if (key == "cpu" || key == "cpuonly" || key == "cpu_only" || key == "cpu-only")
    {
        computeBackend = ComputeBackend::CpuOnly;
        setSteadySolverType(SteadySolverType::LinearParallelGaussian);
        return true;
    }
    if (key == "hybrid" || key == "cpugpu" || key == "cpu+gpu" ||
        key == "cpu_gpu" || key == "cpu-gpu" ||
        key == "cpuplusgpu" || key == "cpu_plus_gpu")
    {
        computeBackend = ComputeBackend::CpuPlusGpu;
        setSteadySolverType(SteadySolverType::LinearCudaDense);
        return true;
    }
    if (key == "gpu" || key == "gpuonly" || key == "gpu_only" || key == "gpu-only")
    {
        computeBackend = ComputeBackend::GpuOnly;
        setSteadySolverType(SteadySolverType::LinearCudaDense);
        return true;
    }

    if (key == "simple" || key == "simpleiteration")
    {
        setSteadySolverType(SteadySolverType::SimpleIteration);
        return true;
    }
    if (key == "newton" || key == "newtonraphson")
    {
        setSteadySolverType(SteadySolverType::NewtonRaphson);
        return true;
    }
    if (key == "gaussian" || key == "gaussianelimination" || key == "lineargaussianelimination")
    {
        setSteadySolverType(SteadySolverType::LinearGaussianElimination);
        return true;
    }
    if (key == "gaussseidel" || key == "lineargaussseidel")
    {
        setSteadySolverType(SteadySolverType::LinearGaussSeidel);
        return true;
    }
    if (key == "parallelgaussian" || key == "linearparallelgaussian")
    {
        setSteadySolverType(SteadySolverType::LinearParallelGaussian);
        return true;
    }
    if (key == "jacobi" || key == "jacobiparallel" || key == "linearjacobiparallel")
    {
        setSteadySolverType(SteadySolverType::LinearJacobiParallel);
        return true;
    }
    if (key == "cg" || key == "conjugategradient" || key == "linearconjugategradient")
    {
        setSteadySolverType(SteadySolverType::LinearConjugateGradient);
        return true;
    }
    if (key == "cuda" || key == "cudadense" || key == "linearcudadense")
    {
        setSteadySolverType(SteadySolverType::LinearCudaDense);
        return true;
    }

    return false;
}

void FluidNetworkSolver::setTransientConfig(const TransientConfig &config)
{
    transientConfig = config;
    transientSolver = std::make_unique<TransientSolver>(config);
}

void FluidNetworkSolver::setTimeIntegrationMethod(TimeIntegrationMethod method)
{
    transientConfig.method = method;
    if (transientSolver) {
        transientSolver->setConfig(transientConfig);
    }
}

void FluidNetworkSolver::setAdaptiveTimeStep(bool enabled)
{
    transientConfig.adaptiveTimeStep = enabled;
    if (transientSolver) {
        transientSolver->setConfig(transientConfig);
    }
}

void FluidNetworkSolver::setConservationTolerance(double tolerance)
{
    transientConfig.conservationTolerance = tolerance;
    if (transientSolver) {
        transientSolver->setConfig(transientConfig);
    }
}

void FluidNetworkSolver::setCouplingMode(CouplingMode mode)
{
    couplingConfig.mode = mode;
    couplingConfig.enabled = (mode != CouplingMode::None);

    if (!couplingSolver) {
        couplingSolver = std::make_unique<CouplingSolver>(couplingConfig);
    } else {
        couplingSolver->setConfig(couplingConfig);
    }
}

void FluidNetworkSolver::setMaxOuterIterations(int maxIter)
{
    couplingConfig.maxOuterIterations = maxIter;
    if (couplingSolver) {
        couplingSolver->setConfig(couplingConfig);
    }
}

void FluidNetworkSolver::setOuterConvergenceTolerance(double tolerance)
{
    couplingConfig.outerConvergenceTol = tolerance;
    if (couplingSolver) {
        couplingSolver->setConfig(couplingConfig);
    }
}

void FluidNetworkSolver::setUnderRelaxationFactor(double factor)
{
    couplingConfig.underRelaxationFactor = factor;
    if (couplingSolver) {
        couplingSolver->setConfig(couplingConfig);
    }
}

bool FluidNetworkSolver::solveWithCoupling()
{
    if (!validateNetwork()) {
        std::cerr << "Network validation failed." << std::endl;
        return false;
    }

    if (!couplingSolver || !couplingConfig.enabled) {
        // 无耦合，直接求解
        return solve();
    }

    SolverContext context = buildContext();
    return couplingSolver->solve(context);
}

void FluidNetworkSolver::setErrorCallback(ErrorCallback callback)
{
    if (!errorHandler) {
        errorHandler = std::make_unique<ErrorHandler>();
    }
    errorHandler->setErrorCallback(callback);
}

bool FluidNetworkSolver::checkNetworkTopology()
{
    if (!errorHandler) {
        errorHandler = std::make_unique<ErrorHandler>();
    }

    std::vector<int> nodeIds;
    for (const auto& node : nodes) {
        if (node) {
            nodeIds.push_back(node->id);
        }
    }

    std::vector<std::pair<int, int>> connections;
    for (const auto& conn : componentConnections) {
        connections.push_back({conn.fromNodeId, conn.toNodeId});
    }

    std::vector<int> isolatedNodes;
    return errorHandler->checkNetworkConnectivity(nodeIds, connections, isolatedNodes);
}

bool FluidNetworkSolver::checkMatrixCondition()
{
    if (!errorHandler) {
        errorHandler = std::make_unique<ErrorHandler>();
    }

    // 构建简化的矩阵进行检查
    std::size_t n = nodes.size();
    std::vector<std::vector<double>> matrix(n, std::vector<double>(n, 0.0));

    // 填充矩阵对角线
    for (std::size_t i = 0; i < n; ++i) {
        matrix[i][i] = 1.0;
    }

    // 添加组件连接的贡献
    for (const auto& conn : componentConnections) {
        if (conn.componentIndex >= 0 &&
            conn.componentIndex < static_cast<int>(components.size())) {
            // 简化的矩阵组装
        }
    }

    double conditionNumber;
    return errorHandler->checkMatrixCondition(matrix, conditionNumber);
}

const std::vector<ErrorInfo>& FluidNetworkSolver::getErrorHistory() const
{
    static std::vector<ErrorInfo> emptyHistory;
    if (errorHandler) {
        return errorHandler->getErrorHistory();
    }
    return emptyHistory;
}

bool FluidNetworkSolver::solve()
{
    if (!validateNetwork())
    {
        std::cerr << "Network validation failed." << std::endl;
        return false;
    }

    if (calcType == CalculationType::SteadyState)
    {
        return solveSteadyState(debugEnabled ? &lastIterationComponentResults : nullptr);
    }

    std::cerr << "Transient calculation requires solveTransient(totalTime)." << std::endl;
    return false;
}

bool FluidNetworkSolver::solveTransient(double totalTime)
{
    if (!validateNetwork())
    {
        std::cerr << "Network validation failed." << std::endl;
        return false;
    }

    if (totalTime <= 0.0)
    {
        return true;
    }

    clearTimeStepSnapshots();

    // 使用新的瞬态求解器
    if (!transientSolver) {
        transientConfig.timeStep = timeStep;
        transientConfig.totalTime = totalTime;
        transientSolver = std::make_unique<TransientSolver>(transientConfig);
    }

    SolverContext context = buildContext();
    return transientSolver->solve(context, totalTime);
}

bool FluidNetworkSolver::solveNewtonRaphson()
{
    if (!validateNetwork())
    {
        std::cerr << "Network validation failed." << std::endl;
        return false;
    }

    const SteadySolverType oldType = steadySolverType;
    steadySolverType = SteadySolverType::NewtonRaphson;
    const bool ok = solveSteadyState(debugEnabled ? &lastIterationComponentResults : nullptr);
    steadySolverType = oldType;
    return ok;
}

Node *FluidNetworkSolver::getNode(int nodeId)
{
    for (auto &node : nodes)
    {
        if (node && node->id == nodeId)
        {
            return node.get();
        }
    }
    return nullptr;
}

const Node *FluidNetworkSolver::getNode(int nodeId) const
{
    for (const auto &node : nodes)
    {
        if (node && node->id == nodeId)
        {
            return node.get();
        }
    }
    return nullptr;
}

Component *FluidNetworkSolver::getComponent(int compIndex)
{
    if (compIndex < 0 || compIndex >= static_cast<int>(components.size()))
    {
        return nullptr;
    }
    return components[static_cast<std::size_t>(compIndex)].get();
}

const Component *FluidNetworkSolver::getComponent(int compIndex) const
{
    if (compIndex < 0 || compIndex >= static_cast<int>(components.size()))
    {
        return nullptr;
    }
    return components[static_cast<std::size_t>(compIndex)].get();
}

void FluidNetworkSolver::updateComponents(double deltaTime)
{
    for (auto &comp : components)
    {
        if (comp)
        {
            comp->update(deltaTime);
        }
    }
}

bool FluidNetworkSolver::validateNetwork()
{
    if (!errorHandler) {
        errorHandler = std::make_unique<ErrorHandler>();
    }

    Logger& logger = getLogger();
    logger.info("Validating network topology...");

    if (nodes.empty())
    {
        errorHandler->reportError(ErrorCode::MissingComponent, ErrorSeverity::Error,
                                  "Network has no nodes");
        return false;
    }

    if (components.empty())
    {
        errorHandler->reportError(ErrorCode::MissingComponent, ErrorSeverity::Error,
                                  "Network has no components");
        return false;
    }

    for (const auto &bc : boundaryConditions)
    {
        if (!hasNode(bc.nodeId))
        {
            errorHandler->reportError(ErrorCode::IsolatedNode, ErrorSeverity::Error,
                                      "Boundary condition references missing node",
                                      "", bc.nodeId);
            return false;
        }
    }

    for (std::size_t i = 0; i < components.size(); ++i)
    {
        if (!components[i]->hasConnection())
        {
            errorHandler->reportError(ErrorCode::ComponentNotConnected, ErrorSeverity::Error,
                                      "Component is not connected to nodes",
                                      "", -1, static_cast<int>(i));
            return false;
        }
    }

    if (componentConnections.empty())
    {
        errorHandler->reportError(ErrorCode::MissingComponent, ErrorSeverity::Error,
                                  "Component connection mapping is empty");
        return false;
    }

    std::vector<int> connectionCounts(components.size(), 0);
    for (const auto &connection : componentConnections)
    {
        if (connection.componentIndex >= 0 &&
            connection.componentIndex < static_cast<int>(components.size()))
        {
            ++connectionCounts[static_cast<std::size_t>(connection.componentIndex)];
        }
    }

    for (std::size_t i = 0; i < components.size(); ++i)
    {
        if (!components[i])
        {
            continue;
        }
        if (connectionCounts[i] <= 0)
        {
            errorHandler->reportError(ErrorCode::ComponentNotConnected, ErrorSeverity::Error,
                                      "Component has no connections",
                                      "", -1, static_cast<int>(i));
            return false;
        }
    }

    for (const auto &connection : componentConnections)
    {
        if (!hasNode(connection.fromNodeId) || !hasNode(connection.toNodeId))
        {
            errorHandler->reportError(ErrorCode::IsolatedNode, ErrorSeverity::Error,
                                      "Connection references missing node");
            return false;
        }
    }

    // 检查网络连通性
    if (!checkNetworkTopology()) {
        return false;
    }

    logger.info("Network validation passed.");
    return true;
}

void FluidNetworkSolver::printNetworkInfo()
{
    std::cout << "\n=== Fluid Network Summary ===" << std::endl;
    std::cout << "Nodes: " << nodes.size() << std::endl;
    std::cout << "Components: " << components.size() << std::endl;
    std::cout << "Boundary Conditions: " << boundaryConditions.size() << std::endl;
    std::cout << "Snapshots: " << timeStepSnapshots.size() << std::endl;

    const auto solverIt = steadySolvers.find(steadySolverType);
    if (solverIt != steadySolvers.end())
    {
        std::cout << "Steady Solver: " << solverIt->second->name() << std::endl;
    }

    std::cout << "\nNode List:" << std::endl;
    for (const auto &node : nodes)
    {
        if (!node)
        {
            continue;
        }

        std::cout << "  [" << node->id << "] " << node->name << ", P=" << node->pressure
                  << " Pa, T=" << node->temperature << " K, Qin=" << node->flowInjection
                  << std::endl;
    }

    std::cout << "\nComponent List:" << std::endl;
    for (const auto &connection : componentConnections)
    {
        if (connection.componentIndex < 0 ||
            connection.componentIndex >= static_cast<int>(components.size()))
        {
            continue;
        }

        const Component *comp = components[static_cast<std::size_t>(connection.componentIndex)].get();
        if (!comp)
        {
            continue;
        }

        std::cout << "  #" << connection.componentIndex << " " << comp->getName() << " ["
                  << comp->getType() << "] " << connection.fromNodeId << " -> "
                  << connection.toNodeId << std::endl;
    }

    std::cout << std::endl;
}

void FluidNetworkSolver::reset()
{
    currentTime = 0.0;
    latestConnectionFlows.clear();
    LinearSolver::resetIterationHistory();
    lastIterationHistory = LinearSolver::getLastIterationHistory();
    lastIterationComponentResults.clear();
    clearTimeStepSnapshots();
}

void FluidNetworkSolver::clearTimeStepSnapshots()
{
    timeStepSnapshots.clear();
}

void FluidNetworkSolver::clearNetwork()
{
    nodes.clear();
    components.clear();
    boundaryConditions.clear();
    connectivity.clear();
    componentConnections.clear();
    latestConnectionFlows.clear();
    clearTimeStepSnapshots();
    reset();
}

SolverContext FluidNetworkSolver::buildContext()
{
    SolverContext context = {nodes,
                             components,
                             boundaryConditions,
                             connectivity,
                             componentConnections,
                             workingFluid,
                             latestConnectionFlows,
                             lastIterationHistory,
                             convergenceTolerance,
                             maxIterations,
                             relaxationFactor,
                             relativeConvergenceTolerance,
                             solutionIncrementTolerance,
                             debugEnabled,
                             activeDebugTarget,
                             &couplingConfig,
                             currentTime};
    return context;
}

bool FluidNetworkSolver::solveSteadyState(std::vector<IterationComponentSnapshot> *debugTarget)
{
    const auto it = steadySolvers.find(steadySolverType);
    if (it == steadySolvers.end() || !it->second)
    {
        getLogger().error("Steady solver is not available.");
        return false;
    }

    Logger& logger = getLogger();
    logger.info("Starting steady state solve with " + std::string(it->second->name()));

    LinearSolver::setMaxThreads(cpuThreads);
    switch (computeBackend)
    {
    case ComputeBackend::CpuOnly:
        LinearSolver::setComputeBackend(LinearSolver::ComputeBackend::CpuOnly);
        break;
    case ComputeBackend::GpuOnly:
        LinearSolver::setComputeBackend(LinearSolver::ComputeBackend::GpuOnly);
        break;
    case ComputeBackend::CpuPlusGpu:
    default:
        LinearSolver::setComputeBackend(LinearSolver::ComputeBackend::Hybrid);
        break;
    }

    activeDebugTarget = debugTarget;
    if (activeDebugTarget != nullptr)
    {
        activeDebugTarget->clear();
    }

    SolverContext context = buildContext();
    applyBoundaryConditions(context);
    const bool ok = it->second->solve(context);
    lastIterationHistory = context.iterationHistory;
    if (activeDebugTarget != nullptr)
    {
        lastIterationComponentResults = *activeDebugTarget;
    }
    else
    {
        lastIterationComponentResults.clear();
    }

    // 使用日志系统输出残差历史
    if (!lastIterationHistory.residuals.empty()) {
        logger.printResidualTable(lastIterationHistory.residuals, it->second->name());
    }

    logger.logConvergence(lastIterationHistory.totalIterations,
                          lastIterationHistory.converged,
                          it->second->name());

    computeAllConnectionFlows(context);
    activeDebugTarget = nullptr;
    return ok;
}

bool FluidNetworkSolver::hasNode(int nodeId) const
{
    for (const auto &node : nodes)
    {
        if (node && node->id == nodeId)
        {
            return true;
        }
    }
    return false;
}

void FluidNetworkSolver::addConnectivityEdge(int fromNodeId, int toNodeId)
{
    std::vector<int> &neighbors = connectivity[fromNodeId];
    if (std::find(neighbors.begin(), neighbors.end(), toNodeId) == neighbors.end())
    {
        neighbors.push_back(toNodeId);
    }
}

void FluidNetworkSolver::saveTimeStepSnapshot(
    double time,
    const std::vector<IterationComponentSnapshot> *debugResults)
{
    TimeStepSnapshot snapshot;
    snapshot.time = time;

    snapshot.nodePressures.reserve(nodes.size());
    snapshot.nodeTemperatures.reserve(nodes.size());
    for (const auto &node : nodes)
    {
        if (!node)
        {
            continue;
        }
        snapshot.nodePressures.push_back(node->pressure);
        snapshot.nodeTemperatures.push_back(node->temperature);
    }

    snapshot.connectionFlows = latestConnectionFlows;
    snapshot.iterationHistory = lastIterationHistory;
    if (debugResults != nullptr)
    {
        snapshot.iterationComponentResults = *debugResults;
    }
    else
    {
        snapshot.iterationComponentResults.clear();
    }

    timeStepSnapshots.push_back(snapshot);
}
