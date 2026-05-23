#include "coupling/coupling_manager.h"
#include <fstream>
#include <iostream>

bool CouplingManager::solve2DCase(const std::string& caseId,
                                   const std::map<int, Boundary2D>& boundaries) {
    auto hit = heatSolvers_.find(caseId);
    if (hit != heatSolvers_.end()) {
        hit->second->applyBoundaries(boundaries);
        return hit->second->solve();
    }
    auto pit = pressureSolvers_.find(caseId);
    if (pit != pressureSolvers_.end()) {
        pit->second->applyBoundaries(boundaries);
        return pit->second->solve();
    }
    return false;
}

TwoDResult CouplingManager::get2DResultInternal(const std::string& caseId) {
    auto hit = heatSolvers_.find(caseId);
    if (hit != heatSolvers_.end()) {
        return hit->second->extractResults();
    }
    auto pit = pressureSolvers_.find(caseId);
    if (pit != pressureSolvers_.end()) {
        return pit->second->extractResults();
    }
    return TwoDResult();
}

void CouplingManager::printSummary() const {
    printf("\n=== Coupling Summary ===\n");
    printf("Mode: %s\n", config_.strategy.mode.c_str());
    printf("Enabled: %s\n", config_.enabled ? "true" : "false");
    printf("Total iterations: %d\n", history_.totalIterations);
    printf("Converged: %s\n", history_.converged ? "YES" : "NO");
    printf("Final max change: %.6e\n", history_.finalMaxChange);
    printf("2D cases: %zu\n", cachedResults_.size());
    for (const auto& pair : cachedResults_) {
        printf("  Case '%s' (%s): success=%s\n",
               pair.first.c_str(), pair.second.physics.c_str(),
               pair.second.success ? "true" : "false");
    }
    printf("=========================\n");
}
