#include "simple_iteration_solver.h"

#include "utils/coupling/coupling_io.h"
#include "../solver_context_utils.h"

#include <algorithm>
#include <cmath>
#include <iostream>

SteadySolverType SimpleIterationSolverMethod::kind() const {
    return SteadySolverType::SimpleIteration;
}

const char* SimpleIterationSolverMethod::name() const {
    return "SimpleIteration";
}

bool SimpleIterationSolverMethod::solve(SolverContext& context) {
    context.iterationHistory.residuals.clear();
    context.iterationHistory.totalIterations = 0;
    context.iterationHistory.converged = false;

    for (int iter = 0; iter < context.maxIterations; ++iter) {
        if (context.couplingConfig && context.couplingConfig->enabled) {
            if (context.couplingConfig->readEveryIteration || iter == 0) {
                CouplingIO::applyCouplingInputs(context,
                                                *context.couplingConfig,
                                                context.currentTime,
                                                iter + 1);
            }
        }
        applyBoundaryConditions(context);

        bool allConverged = true;
        double maxResidual = 0.0;

        for (const auto& nodePtr : context.nodes) {
            if (!nodePtr) {
                continue;
            }

            if (isBoundaryNode(context, nodePtr->id)) {
                continue;
            }

            const double netFlow = computeNetFlowAtNode(context, nodePtr->id);
            maxResidual = std::max(maxResidual, std::abs(netFlow));
            if (std::abs(netFlow) > context.convergenceTolerance) {
                allConverged = false;

                nodePtr->pressure += context.relaxationFactor * netFlow * 1000.0;
                if (nodePtr->pressure < 1000.0) {
                    nodePtr->pressure = 1000.0;
                }
            }
        }

        recordIterationComponentResults(context, iter + 1);
        if (context.couplingConfig && context.couplingConfig->enabled &&
            context.couplingConfig->writeEveryIteration) {
            CouplingIO::writeCouplingOutputs(context,
                                             *context.couplingConfig,
                                             context.currentTime,
                                             iter + 1);
        }

        context.iterationHistory.residuals.push_back(maxResidual);
        context.iterationHistory.totalIterations = iter + 1;

        if (allConverged) {
            context.iterationHistory.converged = true;
            std::cout << "Steady solution converged after " << (iter + 1)
                      << " iterations using " << name() << "." << std::endl;
            return true;
        }
    }

    std::cerr << "Steady solution failed to converge after " << context.maxIterations
              << " iterations using " << name() << "." << std::endl;
    return false;
}
