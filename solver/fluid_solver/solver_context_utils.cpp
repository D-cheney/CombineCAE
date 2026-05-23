#include "solver_context_utils.h"

#include <algorithm>
#include <cmath>

#ifdef _OPENMP
#include <omp.h>
#endif

Node *findNodeById(SolverContext &context, int nodeId)
{
    for (auto &node : context.nodes)
    {
        if (node && node->id == nodeId)
        {
            return node.get();
        }
    }
    return nullptr;
}

const Node *findNodeById(const SolverContext &context, int nodeId)
{
    for (const auto &node : context.nodes)
    {
        if (node && node->id == nodeId)
        {
            return node.get();
        }
    }
    return nullptr;
}

bool hasNodeId(const SolverContext &context, int nodeId)
{
    return findNodeById(context, nodeId) != nullptr;
}

bool isBoundaryNode(const SolverContext &context, int nodeId)
{
    return std::any_of(context.boundaryConditions.begin(), context.boundaryConditions.end(),
                       [nodeId](const BoundaryCondition &bc)
                       { return bc.nodeId == nodeId; });
}

void applyBoundaryConditions(SolverContext &context)
{
    for (const auto &bc : context.boundaryConditions)
    {
        Node *node = findNodeById(context, bc.nodeId);
        if (node == nullptr)
        {
            continue;
        }

        switch (bc.type)
        {
        case BoundaryConditionType::Pressure:
            node->pressure = bc.value;
            node->isPressureFixed = true;
            node->fixedPressure = bc.value;
            break;
        case BoundaryConditionType::Temperature:
            node->temperature = bc.value;
            break;
        case BoundaryConditionType::FlowRate:
            node->massFlowRate = bc.value;
            node->flowInjection = bc.value;
            break;
        default:
            break;
        }
    }
}

std::vector<int> getInternalNodeIds(const SolverContext &context)
{
    std::vector<int> internalNodeIds;
    internalNodeIds.reserve(context.nodes.size());

    for (const auto &node : context.nodes)
    {
        if (node && !isBoundaryNode(context, node->id))
        {
            internalNodeIds.push_back(node->id);
        }
    }

    return internalNodeIds;
}

int getNodeIndexById(const SolverContext &context, int nodeId)
{
    for (std::size_t i = 0; i < context.nodes.size(); ++i)
    {
        const auto &node = context.nodes[i];
        if (node && node->id == nodeId)
        {
            return static_cast<int>(i);
        }
    }
    return -1;
}

double estimateConnectionResistance(SolverContext &context, const ComponentConnection &connection)
{
    if (connection.componentIndex < 0 ||
        connection.componentIndex >= static_cast<int>(context.components.size()))
    {
        return 1e12;
    }

    Component *component = context.components[static_cast<std::size_t>(connection.componentIndex)].get();
    Node *fromNode = findNodeById(context, connection.fromNodeId);
    Node *toNode = findNodeById(context, connection.toNodeId);

    if (component == nullptr || fromNode == nullptr || toNode == nullptr)
    {
        return 1e12;
    }

    const double pressureDiff = fromNode->pressure - toNode->pressure;
    const double probeFlow = (std::abs(pressureDiff) < 1e-12 || pressureDiff >= 0.0) ? 1.0 : -1.0;
    std::vector<double> flowRates(1, probeFlow);
    std::vector<Node *> compNodes;
    compNodes.push_back(fromNode);
    compNodes.push_back(toNode);

    double pressureDrop =
        component->calculatePressureDropWithPorts(flowRates,
                                                  compNodes,
                                                  connection.fromPortIndex,
                                                  connection.toPortIndex);
    double resistance = std::abs(pressureDrop / probeFlow);
    if (resistance < 1e-6)
    {
        resistance = 1e-6;
    }
    return resistance;
}

double estimateConnectionFlow(SolverContext &context, const ComponentConnection &connection)
{
    const double resistance = estimateConnectionResistance(context, connection);
    if (resistance >= 1e11)
    {
        return 0.0;
    }

    Node *fromNode = findNodeById(context, connection.fromNodeId);
    Node *toNode = findNodeById(context, connection.toNodeId);
    if (fromNode == nullptr || toNode == nullptr)
    {
        return 0.0;
    }

    const double pressureDiff = fromNode->pressure - toNode->pressure;
    return pressureDiff / resistance;
}

double computeNetFlowAtNode(SolverContext &context, int nodeId)
{
    double netFlow = 0.0;

    for (const auto &connection : context.componentConnections)
    {
        const double flow = estimateConnectionFlow(context, connection);

        if (connection.fromNodeId == nodeId)
        {
            netFlow -= flow;
        }
        else if (connection.toNodeId == nodeId)
        {
            netFlow += flow;
        }
    }

    const Node *node = findNodeById(context, nodeId);
    if (node != nullptr)
    {
        netFlow += node->flowInjection;
    }

    return netFlow;
}

void computeAllConnectionFlows(SolverContext &context)
{
    context.latestConnectionFlows.assign(context.componentConnections.size(), 0.0);

    // Parallel calculation of connection flows - independent components
#ifdef _OPENMP
#pragma omp parallel for schedule(dynamic)
#endif
    for (int i = 0; i < static_cast<int>(context.componentConnections.size()); ++i)
    {
        context.latestConnectionFlows[i] =
            estimateConnectionFlow(context, context.componentConnections[i]);
    }
}

void recordIterationComponentResults(SolverContext &context, int iteration)
{
    if (!context.debugEnabled || context.debugIterations == nullptr)
    {
        return;
    }

    IterationComponentSnapshot snapshot;
    snapshot.iteration = iteration;
    snapshot.components.reserve(context.componentConnections.size());

    for (const auto &connection : context.componentConnections)
    {
        if (connection.componentIndex < 0 ||
            connection.componentIndex >= static_cast<int>(context.components.size()))
        {
            continue;
        }

        Component *component =
            context.components[static_cast<std::size_t>(connection.componentIndex)].get();
        Node *fromNode = findNodeById(context, connection.fromNodeId);
        Node *toNode = findNodeById(context, connection.toNodeId);
        if (component == nullptr || fromNode == nullptr || toNode == nullptr)
        {
            continue;
        }

        const double flow = estimateConnectionFlow(context, connection);
        std::vector<double> flows(1, flow);
        std::vector<Node *> compNodes;
        compNodes.push_back(fromNode);
        compNodes.push_back(toNode);

        const double pressureDrop =
            component->calculatePressureDropWithPorts(flows,
                                                      compNodes,
                                                      connection.fromPortIndex,
                                                      connection.toPortIndex);
        double resistance = 0.0;
        if (std::abs(flow) > 1e-12)
        {
            resistance = pressureDrop / flow;
        }
        else
        {
            resistance = estimateConnectionResistance(context, connection);
        }
        const double heatTransfer = component->calculateHeatTransfer(compNodes);

        ComponentIterationResult result;
        result.componentIndex = connection.componentIndex;
        result.type = component->getType();
        result.name = component->getName();
        result.fromNodeId = connection.fromNodeId;
        result.toNodeId = connection.toNodeId;
        result.fromPortIndex = connection.fromPortIndex;
        result.toPortIndex = connection.toPortIndex;
        result.flow = flow;
        result.pressureDrop = pressureDrop;
        result.resistance = resistance;
        result.heatTransfer = heatTransfer;
        snapshot.components.push_back(result);
    }

    context.debugIterations->push_back(snapshot);
}
