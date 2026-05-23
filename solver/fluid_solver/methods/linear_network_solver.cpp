#include "linear_network_solver.h"

#include "core/heat_exchanger.h"
#include "utils/coupling/coupling_io.h"
#include "../solver_context_utils.h"

#include <algorithm>
#include <cmath>
#include <iostream>
#include <vector>

namespace {

double computeThermalConductance(Component* component, Node* fromNode, Node* toNode) {
    if (component == nullptr || fromNode == nullptr || toNode == nullptr) {
        return 0.0;
    }

    std::vector<Node*> compNodes;
    compNodes.push_back(fromNode);
    compNodes.push_back(toNode);

    const double deltaT = fromNode->temperature - toNode->temperature;
    if (std::abs(deltaT) < 1e-12) {
        const HeatExchanger* hex = dynamic_cast<const HeatExchanger*>(component);
        if (hex != nullptr) {
            return std::max(0.0, hex->getUA());
        }
    }

    const double heatTransfer = component->calculateHeatTransfer(compNodes);
    if (std::abs(deltaT) > 1e-12) {
        return std::max(0.0, std::abs(heatTransfer / deltaT));
    }

    const HeatExchanger* hex = dynamic_cast<const HeatExchanger*>(component);
    if (hex != nullptr) {
        return std::max(0.0, hex->getUA());
    }

    return 0.0;
}

} // namespace

LinearNetworkSolverMethod::LinearNetworkSolverMethod(SteadySolverType kind,
                                                     LinearSolver::Method linearMethod,
                                                     const std::string& name)
    : kind_(kind), linearMethod_(linearMethod), name_(name) {}

SteadySolverType LinearNetworkSolverMethod::kind() const {
    return kind_;
}

const char* LinearNetworkSolverMethod::name() const {
    return name_.c_str();
}

bool LinearNetworkSolverMethod::solve(SolverContext& context) {
    const bool hydraulicOk = solveHydraulic(context);
    const bool thermalOk = solveThermal(context);
    return hydraulicOk || thermalOk;
}

bool LinearNetworkSolverMethod::solveHydraulic(SolverContext& context) {
    if (context.couplingConfig && context.couplingConfig->enabled) {
        if (context.couplingConfig->readEveryIteration) {
            CouplingIO::applyCouplingInputs(context,
                                            *context.couplingConfig,
                                            context.currentTime,
                                            0);
        }
    }
    applyBoundaryConditions(context);

    const int n = static_cast<int>(context.nodes.size());
    if (n <= 0) {
        return false;
    }

    std::vector<std::vector<double>> a(static_cast<std::size_t>(n),
                                       std::vector<double>(static_cast<std::size_t>(n), 0.0));
    std::vector<double> b(static_cast<std::size_t>(n), 0.0);
    std::vector<double> x(static_cast<std::size_t>(n), 0.0);
    std::vector<bool> fixed(static_cast<std::size_t>(n), false);
    std::vector<double> fixedValue(static_cast<std::size_t>(n), 0.0);

    for (int i = 0; i < n; ++i) {
        Node* node = context.nodes[static_cast<std::size_t>(i)].get();
        if (node == nullptr) {
            continue;
        }
        x[static_cast<std::size_t>(i)] = node->pressure;
        b[static_cast<std::size_t>(i)] = node->flowInjection;
        if (node->isPressureFixed) {
            fixed[static_cast<std::size_t>(i)] = true;
            fixedValue[static_cast<std::size_t>(i)] = node->fixedPressure;
        }
    }

    for (const auto& bc : context.boundaryConditions) {
        if (bc.type == BoundaryConditionType::Pressure) {
            const int idx = getNodeIndexById(context, bc.nodeId);
            if (idx >= 0) {
                fixed[static_cast<std::size_t>(idx)] = true;
                fixedValue[static_cast<std::size_t>(idx)] = bc.value;
            }
        } else if (bc.type == BoundaryConditionType::FlowRate) {
            const int idx = getNodeIndexById(context, bc.nodeId);
            if (idx >= 0) {
                b[static_cast<std::size_t>(idx)] = bc.value;
            }
        }
    }

    for (const auto& connection : context.componentConnections) {
        if (connection.componentIndex < 0 ||
            connection.componentIndex >= static_cast<int>(context.components.size())) {
            continue;
        }
        const int i = getNodeIndexById(context, connection.fromNodeId);
        const int j = getNodeIndexById(context, connection.toNodeId);
        if (i < 0 || j < 0 || i == j) {
            continue;
        }

        const double resistance = estimateConnectionResistance(context, connection);
        if (resistance <= 0.0 || resistance >= 1e11) {
            continue;
        }

        const double conductance = 1.0 / resistance;
        a[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += conductance;
        a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] -= conductance;
        a[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] -= conductance;
        a[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] += conductance;
    }

    for (int i = 0; i < n; ++i) {
        if (!fixed[static_cast<std::size_t>(i)]) {
            continue;
        }
        for (int j = 0; j < n; ++j) {
            a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = (i == j) ? 1.0 : 0.0;
        }
        b[static_cast<std::size_t>(i)] = fixedValue[static_cast<std::size_t>(i)];
        x[static_cast<std::size_t>(i)] = fixedValue[static_cast<std::size_t>(i)];
    }

    const bool debugEnabled =
        context.debugEnabled && context.debugIterations != nullptr;
    std::function<void(int)> iterCallback;
    bool callbackInvoked = false;
    if (debugEnabled || (context.couplingConfig && context.couplingConfig->enabled)) {
        iterCallback = [&](int iter) {
            callbackInvoked = true;
            for (int idx = 0; idx < n; ++idx) {
                Node* node = context.nodes[static_cast<std::size_t>(idx)].get();
                if (node != nullptr && idx < static_cast<int>(x.size())) {
                    node->pressure = x[static_cast<std::size_t>(idx)];
                }
            }
            recordIterationComponentResults(context, iter + 1);
            if (context.couplingConfig && context.couplingConfig->enabled) {
                if (context.couplingConfig->readEveryIteration) {
                    CouplingIO::applyCouplingInputs(context,
                                                    *context.couplingConfig,
                                                    context.currentTime,
                                                    iter + 1);
                }
                if (context.couplingConfig->writeEveryIteration) {
                    CouplingIO::writeCouplingOutputs(context,
                                                     *context.couplingConfig,
                                                     context.currentTime,
                                                     iter + 1);
                }
            }
        };
    }

    const bool ok = LinearSolver::solve(a,
                                        b,
                                        x,
                                        linearMethod_,
                                        context.convergenceTolerance,
                                        context.maxIterations,
                                        context.relaxationFactor,
                                        iterCallback);
    context.iterationHistory = LinearSolver::getLastIterationHistory();
    if (!ok) {
        std::cerr << "Hydraulic linear solve failed in method " << name() << std::endl;
        return false;
    }

    for (int i = 0; i < n; ++i) {
        Node* node = context.nodes[static_cast<std::size_t>(i)].get();
        if (node != nullptr) {
            node->pressure = x[static_cast<std::size_t>(i)];
        }
    }

    if (debugEnabled && context.debugIterations->empty()) {
        recordIterationComponentResults(context, 1);
    }
    if (context.couplingConfig && context.couplingConfig->enabled &&
        context.couplingConfig->writeEveryIteration && !callbackInvoked) {
        CouplingIO::writeCouplingOutputs(context,
                                         *context.couplingConfig,
                                         context.currentTime,
                                         1);
    }

    computeAllConnectionFlows(context);
    return true;
}

bool LinearNetworkSolverMethod::solveThermal(SolverContext& context) {
    const int n = static_cast<int>(context.nodes.size());
    if (n <= 0) {
        return false;
    }

    std::vector<std::vector<double>> a(static_cast<std::size_t>(n),
                                       std::vector<double>(static_cast<std::size_t>(n), 0.0));
    std::vector<double> b(static_cast<std::size_t>(n), 0.0);
    std::vector<double> t(static_cast<std::size_t>(n), 0.0);
    std::vector<bool> fixed(static_cast<std::size_t>(n), false);
    std::vector<double> fixedValue(static_cast<std::size_t>(n), 0.0);

    for (int i = 0; i < n; ++i) {
        Node* node = context.nodes[static_cast<std::size_t>(i)].get();
        if (node == nullptr) {
            continue;
        }
        t[static_cast<std::size_t>(i)] = node->temperature;
        b[static_cast<std::size_t>(i)] = node->thermalInjectionPower;
    }

    bool hasTemperatureBC = false;
    for (const auto& bc : context.boundaryConditions) {
        if (bc.type == BoundaryConditionType::Temperature) {
            const int idx = getNodeIndexById(context, bc.nodeId);
            if (idx >= 0) {
                fixed[static_cast<std::size_t>(idx)] = true;
                fixedValue[static_cast<std::size_t>(idx)] = bc.value;
                hasTemperatureBC = true;
            }
        }
    }

    bool hasThermalTerm = false;
    for (const auto& connection : context.componentConnections) {
        if (connection.componentIndex < 0 ||
            connection.componentIndex >= static_cast<int>(context.components.size())) {
            continue;
        }
        const int i = getNodeIndexById(context, connection.fromNodeId);
        const int j = getNodeIndexById(context, connection.toNodeId);
        if (i < 0 || j < 0 || i == j) {
            continue;
        }

        Component* component =
            context.components[static_cast<std::size_t>(connection.componentIndex)].get();
        Node* fromNode = context.nodes[static_cast<std::size_t>(i)].get();
        Node* toNode = context.nodes[static_cast<std::size_t>(j)].get();
        const double gT = computeThermalConductance(component, fromNode, toNode);
        if (gT <= 0.0) {
            continue;
        }

        hasThermalTerm = true;
        a[static_cast<std::size_t>(i)][static_cast<std::size_t>(i)] += gT;
        a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] -= gT;
        a[static_cast<std::size_t>(j)][static_cast<std::size_t>(i)] -= gT;
        a[static_cast<std::size_t>(j)][static_cast<std::size_t>(j)] += gT;
    }

    if (!hasThermalTerm) {
        return true;
    }

    if (!hasTemperatureBC) {
        for (int i = 0; i < n; ++i) {
            if (context.nodes[static_cast<std::size_t>(i)] != nullptr) {
                fixed[static_cast<std::size_t>(i)] = true;
                fixedValue[static_cast<std::size_t>(i)] = t[static_cast<std::size_t>(i)];
                break;
            }
        }
    }

    for (int i = 0; i < n; ++i) {
        if (!fixed[static_cast<std::size_t>(i)]) {
            continue;
        }
        for (int j = 0; j < n; ++j) {
            a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)] = (i == j) ? 1.0 : 0.0;
        }
        b[static_cast<std::size_t>(i)] = fixedValue[static_cast<std::size_t>(i)];
        t[static_cast<std::size_t>(i)] = fixedValue[static_cast<std::size_t>(i)];
    }

    LinearSolver::Method thermalMethod = LinearSolver::Method::GaussianElimination;
    if (linearMethod_ == LinearSolver::Method::CudaDense &&
        LinearSolver::getComputeBackend() == LinearSolver::ComputeBackend::GpuOnly) {
        thermalMethod = LinearSolver::Method::CudaDense;
    }

    bool ok = LinearSolver::solve(a,
                                  b,
                                  t,
                                  thermalMethod,
                                  context.convergenceTolerance,
                                  context.maxIterations,
                                  context.relaxationFactor,
                                  nullptr);
    if (!ok && n > 1) {
        const int ref = n - 1;
        const int rn = n - 1;
        double refValue = t[static_cast<std::size_t>(ref)];
        if (fixed[static_cast<std::size_t>(ref)]) {
            refValue = fixedValue[static_cast<std::size_t>(ref)];
        }
        std::vector<std::vector<double>> ar(static_cast<std::size_t>(rn),
                                            std::vector<double>(static_cast<std::size_t>(rn), 0.0));
        std::vector<double> br(static_cast<std::size_t>(rn), 0.0);
        std::vector<double> xr(static_cast<std::size_t>(rn), 0.0);

        for (int i = 0, ii = 0; i < n; ++i) {
            if (i == ref) {
                continue;
            }
            br[static_cast<std::size_t>(ii)] = b[static_cast<std::size_t>(i)];
            for (int j = 0, jj = 0; j < n; ++j) {
                if (j == ref) {
                    continue;
                }
                ar[static_cast<std::size_t>(ii)][static_cast<std::size_t>(jj)] =
                    a[static_cast<std::size_t>(i)][static_cast<std::size_t>(j)];
                ++jj;
            }
            ++ii;
        }

        ok = LinearSolver::solve(ar,
                                 br,
                                 xr,
                                 thermalMethod,
                                 context.convergenceTolerance,
                                 context.maxIterations,
                                 context.relaxationFactor,
                                 nullptr);
        if (ok) {
            for (int i = 0, ii = 0; i < n; ++i) {
                if (i == ref) {
                    t[static_cast<std::size_t>(i)] = refValue;
                    continue;
                }
                t[static_cast<std::size_t>(i)] = xr[static_cast<std::size_t>(ii)];
                ++ii;
            }
        }
    }

    if (!ok) {
        std::cerr << "Thermal linear solve failed in method " << name() << std::endl;
        return false;
    }

    for (int i = 0; i < n; ++i) {
        Node* node = context.nodes[static_cast<std::size_t>(i)].get();
        if (node != nullptr) {
            node->temperature = t[static_cast<std::size_t>(i)];
        }
    }
    return true;
}
