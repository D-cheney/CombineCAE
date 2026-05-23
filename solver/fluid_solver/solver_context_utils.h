#ifndef SOLVER_CONTEXT_UTILS_H
#define SOLVER_CONTEXT_UTILS_H

#include "solver_context.h"

#include <vector>

Node* findNodeById(SolverContext& context, int nodeId);
const Node* findNodeById(const SolverContext& context, int nodeId);

bool hasNodeId(const SolverContext& context, int nodeId);
bool isBoundaryNode(const SolverContext& context, int nodeId);

void applyBoundaryConditions(SolverContext& context);
std::vector<int> getInternalNodeIds(const SolverContext& context);

int getNodeIndexById(const SolverContext& context, int nodeId);
double estimateConnectionFlow(SolverContext& context, const ComponentConnection& connection);
double estimateConnectionResistance(SolverContext& context, const ComponentConnection& connection);
double computeNetFlowAtNode(SolverContext& context, int nodeId);
void computeAllConnectionFlows(SolverContext& context);
void recordIterationComponentResults(SolverContext& context, int iteration);

#endif // SOLVER_CONTEXT_UTILS_H
