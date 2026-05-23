/**
 * @file coupling_manager.h
 * @brief Manages 1D-2D coupling iterations (one-way, staggered).
 *
 * Responsibilities:
 * - Organize one-way, staggered, or strong-coupled workflows
 * - Manage iteration, relaxation, convergence criteria
 * - Record coupling history
 */

#ifndef COUPLING_MANAGER_H
#define COUPLING_MANAGER_H

#include "coupling/coupling_types.h"
#include "coupling/relaxation_controller.h"
#include "coupling/boundary_mapper.h"
#include "coupling/twod_solver_interface.h"

#include <string>
#include <vector>
#include <map>
#include <memory>
#include <functional>
#include <fstream>
#include <iostream>

// ============================================================================
// CouplingManager
// ============================================================================
class CouplingManager {
public:
    // Callback for solving 1D network
    using Solve1DFunc = std::function<bool()>;

    // Callback for reading 1D values
    using Read1DValueFunc = std::function<double(const std::string&, const std::string&)>;

    // Callback for writing 1D values
    using Write1DValueFunc = std::function<void(const std::string&, const std::string&, double)>;

private:
    CouplingConfig config_;
    Solve1DFunc solve1D_;
    Read1DValueFunc read1D_;
    Write1DValueFunc write1D_;

    BoundaryMapper mapper_;
    RelaxationController relaxation_;
    CouplingHistory history_;

    // 2D solver instances (one per case)
    std::map<std::string, std::unique_ptr<Heat2DSolver>> heatSolvers_;
    std::map<std::string, std::unique_ptr<Pressure2DSolver>> pressureSolvers_;

    // Cached 2D results
    std::map<std::string, TwoDResult> cachedResults_;

    bool verbose_;

public:
    CouplingManager() : verbose_(false) {}

    // ========================================================================
    // Configuration
    // ========================================================================

    void setConfig(const CouplingConfig& config) { config_ = config; }
    void setSolve1DFunction(Solve1DFunc func) { solve1D_ = func; }
    void setRead1DFunction(Read1DValueFunc func) {
        read1D_ = func;
        mapper_.setRead1DFunction(func);
    }
    void setWrite1DFunction(Write1DValueFunc func) {
        write1D_ = func;
        mapper_.setWrite1DFunction(func);
    }
    void setVerbose(bool verbose) {
        verbose_ = verbose;
        mapper_.setVerbose(verbose);
    }

    const CouplingHistory& getHistory() const { return history_; }

    // ========================================================================
    // Initialization
    // ========================================================================

    /**
     * Initialize all 2D solvers based on config.
     */
    bool initialize() {
        // Create solvers for each 2D case
        for (const auto& twodCase : config_.twodCases) {
            if (!twodCase.enabled) continue;

            if (twodCase.physics == "heat") {
                auto solver = SolverFactory::createHeatSolver(SolverFactory::BackendType::Analytical);
                if (solver->initialize(twodCase)) {
                    heatSolvers_[twodCase.id] = std::move(solver);
                }
            }
            else if (twodCase.physics == "pressure") {
                auto solver = SolverFactory::createPressureSolver(SolverFactory::BackendType::Analytical);
                if (solver->initialize(twodCase)) {
                    pressureSolvers_[twodCase.id] = std::move(solver);
                }
            }
        }

        // Configure relaxation
        if (config_.strategy.mode == "staggered") {
            relaxation_.setMode(RelaxationController::Mode::Fixed);
            relaxation_.setFixedAlpha(config_.strategy.relaxationFactor);
        }
        else {
            relaxation_.setMode(RelaxationController::Mode::Fixed);
            relaxation_.setFixedAlpha(1.0); // No relaxation for one-way
        }

        // Validate mappings
        auto errors = mapper_.validateMappings(
            config_.couplingInterfaces, config_.twodCases);
        if (!errors.empty()) {
            for (const auto& err : errors) {
                printf("CouplingManager: Validation error: %s\n", err.c_str());
            }
            return false;
        }

        return true;
    }

    // ========================================================================
    // One-way coupling
    // ========================================================================

    /**
     * Execute one-way coupling: 1D -> 2D (no feedback).
     */
    bool executeOneWay() {
        if (verbose_) {
            printf("=== One-Way Coupling ===\n");
        }

        // 1. Solve 1D
        if (solve1D_) {
            if (!solve1D_()) {
                printf("CouplingManager: 1D solve failed\n");
                return false;
            }
        }

        // 2. For each interface: map 1D -> 2D, solve 2D
        for (const auto& iface : config_.couplingInterfaces) {
            if (!iface.enabled) continue;

            const TwoDCase* case2D = config_.getTwoDCase(iface.twodCase);
            if (!case2D) continue;

            // Map 1D values to 2D boundaries
            auto boundaries = mapper_.mapTo2D(iface, *case2D);
            if (boundaries.empty()) continue;

            // Solve 2D
            bool solved = solve2DCase(iface.twodCase, boundaries);
            if (!solved) {
                printf("CouplingManager: 2D solve failed for case %s\n",
                       iface.twodCase.c_str());
                return false;
            }

            // Extract results (no feedback in one-way)
            TwoDResult result = get2DResultInternal(iface.twodCase);
            cachedResults_[iface.twodCase] = result;

            if (verbose_) {
                printf("  Interface '%s': 2D case '%s' solved\n",
                       iface.name.c_str(), iface.twodCase.c_str());
            }
        }

        history_.totalIterations = 1;
        history_.converged = true;
        return true;
    }

    // ========================================================================
    // Staggered (weak) coupling
    // ========================================================================

    /**
     * Execute staggered coupling: 1D <-> 2D with iteration.
     */
    bool executeStaggered() {
        if (verbose_) {
            printf("=== Staggered Coupling ===\n");
            printf("  Max iterations: %d\n", config_.strategy.maxOuterIterations);
            printf("  Relaxation: %.3f\n", config_.strategy.relaxationFactor);
        }

        relaxation_.reset();
        history_ = CouplingHistory();

        // Store previous iteration values for convergence check
        double prevTotalHeatFlow = 0.0;
        double prevAvgTemp = 0.0;

        double maxChange = 0.0;

        for (int iter = 1; iter <= config_.strategy.maxOuterIterations; ++iter) {
            CouplingIterationRecord record;
            record.iteration = iter;

            // 1. Solve 1D
            if (solve1D_) {
                if (!solve1D_()) {
                    printf("CouplingManager: 1D solve failed at iteration %d\n", iter);
                    return false;
                }
            }

            double maxChange = 0.0;

            // 2. For each interface: map 1D -> 2D, solve 2D, feedback
            for (const auto& iface : config_.couplingInterfaces) {
                if (!iface.enabled) continue;

                const TwoDCase* case2D = config_.getTwoDCase(iface.twodCase);
                if (!case2D) continue;

                // Map 1D values to 2D boundaries
                auto boundaries = mapper_.mapTo2D(iface, *case2D);
                if (boundaries.empty()) continue;

                // Solve 2D
                bool solved = solve2DCase(iface.twodCase, boundaries);
                if (!solved) {
                    printf("CouplingManager: 2D solve failed for case %s\n",
                           iface.twodCase.c_str());
                    return false;
                }

                // Extract results
                TwoDResult result = get2DResultInternal(iface.twodCase);
                cachedResults_[iface.twodCase] = result;

                // Compute changes for convergence
                double Q = result.getScalar("total_heat_flow", 0.0);
                double Tavg = result.getScalar("avg_temperature", 0.0);

                if (iter > 1) {
                    double dQ = std::abs(Q - prevTotalHeatFlow);
                    double dT = std::abs(Tavg - prevAvgTemp);
                    maxChange = std::max(maxChange, dQ);
                    maxChange = std::max(maxChange, dT);
                }

                // Feedback to 1D (with relaxation)
                mapper_.mapFrom2D(iface, result);
            }

            // Check convergence
            bool converged = true;
            if (iter > 1) {
                double refQ = std::max(std::abs(prevTotalHeatFlow), 1e-12);
                double refT = std::max(std::abs(prevAvgTemp), 1e-12);

                if (maxChange / refQ > config_.strategy.toleranceHeatFlow) converged = false;
                if (maxChange / refT > config_.strategy.toleranceTemperature) converged = false;
            }

            record.maxHeatFlowChange = maxChange;
            record.maxTemperatureChange = maxChange;
            record.relaxationFactor = config_.strategy.relaxationFactor;
            record.converged = converged;
            history_.records.push_back(record);

            prevTotalHeatFlow = 0.0;
            prevAvgTemp = 0.0;
            for (const auto& pair : cachedResults_) {
                prevTotalHeatFlow += pair.second.getScalar("total_heat_flow", 0.0);
                prevAvgTemp += pair.second.getScalar("avg_temperature", 0.0);
            }

            if (verbose_) {
                printf("  Iter %d: max_change=%.6e, converged=%s\n",
                       iter, maxChange, converged ? "YES" : "NO");
            }

            if (converged) {
                history_.totalIterations = iter;
                history_.converged = true;
                history_.finalMaxChange = maxChange;

                if (verbose_) {
                    printf("  Converged after %d iterations\n", iter);
                }
                return true;
            }
        }

        history_.totalIterations = config_.strategy.maxOuterIterations;
        history_.converged = false;
        history_.finalMaxChange = maxChange;

        printf("CouplingManager: Did not converge after %d iterations\n",
               config_.strategy.maxOuterIterations);
        return false;
    }

    // ========================================================================
    // Main execution
    // ========================================================================

    /**
     * Execute coupling based on configured mode.
     */
    bool execute() {
        if (!config_.enabled) {
            if (verbose_) {
                printf("CouplingManager: Coupling disabled\n");
            }
            return true;
        }

        if (config_.strategy.mode == "one_way") {
            return executeOneWay();
        }
        else if (config_.strategy.mode == "staggered") {
            return executeStaggered();
        }
        else {
            printf("CouplingManager: Unknown mode '%s', defaulting to one_way\n",
                   config_.strategy.mode.c_str());
            return executeOneWay();
        }
    }

    // ========================================================================
    // Result access
    // ========================================================================

    const TwoDResult* get2DResult(const std::string& caseId) const {
        auto it = cachedResults_.find(caseId);
        return (it != cachedResults_.end()) ? &it->second : nullptr;
    }

    const std::map<std::string, TwoDResult>& getAllResults() const {
        return cachedResults_;
    }

    // ========================================================================
    // Export
    // ========================================================================

    bool exportResults(const std::string& directory) const {
        for (const auto& pair : cachedResults_) {
            std::string path = directory + "/" + pair.first + "_result.txt";
            std::ofstream out(path.c_str());
            if (!out.is_open()) continue;

            out << "# 2D Result: " << pair.first << "\n";
            out << "# Physics: " << pair.second.physics << "\n";
            out << "# Success: " << (pair.second.success ? "true" : "false") << "\n";
            out << "\n# Scalar Metrics:\n";
            for (const auto& m : pair.second.scalarMetrics) {
                out << m.first << " = " << m.second << "\n";
            }
            out << "\n# Boundary Heat Flows:\n";
            for (const auto& b : pair.second.boundaryHeatFlow) {
                out << "  Tag " << b.first << ": " << b.second << " W\n";
            }
            out << "\n# Boundary Avg Temperatures:\n";
            for (const auto& b : pair.second.boundaryAvgTemperature) {
                out << "  Tag " << b.first << ": " << b.second << " K\n";
            }
        }
        return true;
    }

    /**
     * Print coupling summary.
     */
    void printSummary() const;

private:
    bool solve2DCase(const std::string& caseId,
                     const std::map<int, Boundary2D>& boundaries);

    TwoDResult get2DResultInternal(const std::string& caseId);
};

#endif // COUPLING_MANAGER_H
