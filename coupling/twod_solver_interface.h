/**
 * @file twod_solver_interface.h
 * @brief Abstract interfaces for 2D solvers (heat, pressure).
 *
 * This header defines the abstract interfaces that concrete 2D solver
 * implementations must follow. The actual FEniCSx or other solver
 * backends implement these interfaces.
 */

#ifndef TWOD_SOLVER_INTERFACE_H
#define TWOD_SOLVER_INTERFACE_H

#include "coupling/coupling_types.h"

#include <string>
#include <vector>
#include <map>
#include <memory>

// ============================================================================
// Heat2DSolver - Abstract interface for 2D heat conduction solver
// ============================================================================
class Heat2DSolver {
public:
    virtual ~Heat2DSolver() = default;

    /**
     * Initialize solver with case configuration.
     */
    virtual bool initialize(const TwoDCase& caseConfig) = 0;

    /**
     * Apply boundary conditions.
     * @param boundaries Map of boundary tag -> boundary condition
     */
    virtual void applyBoundaries(const std::map<int, Boundary2D>& boundaries) = 0;

    /**
     * Solve the 2D heat conduction problem.
     * @return true if solve succeeded
     */
    virtual bool solve() = 0;

    /**
     * Extract results from solved field.
     */
    virtual TwoDResult extractResults() const = 0;

    /**
     * Get temperature at a point (for debugging).
     */
    virtual double getTemperature(double x, double y) const = 0;

    /**
     * Get max temperature in domain.
     */
    virtual double getMaxTemperature() const = 0;

    /**
     * Get average temperature in domain.
     */
    virtual double getAvgTemperature() const = 0;

    /**
     * Get total heat flow through a boundary.
     */
    virtual double getBoundaryHeatFlow(int boundaryTag) const = 0;

    /**
     * Get average temperature on a boundary.
     */
    virtual double getBoundaryAvgTemperature(int boundaryTag) const = 0;

    /**
     * Get max temperature on a boundary.
     */
    virtual double getBoundaryMaxTemperature(int boundaryTag) const = 0;

    /**
     * Export field to file (VTK/XDMF).
     */
    virtual bool exportField(const std::string& filePath) const = 0;

    /**
     * Reset solver state.
     */
    virtual void reset() = 0;
};

// ============================================================================
// Pressure2DSolver - Abstract interface for 2D pressure/flow solver
// ============================================================================
class Pressure2DSolver {
public:
    virtual ~Pressure2DSolver() = default;

    /**
     * Initialize solver with case configuration.
     */
    virtual bool initialize(const TwoDCase& caseConfig) = 0;

    /**
     * Apply boundary conditions.
     */
    virtual void applyBoundaries(const std::map<int, Boundary2D>& boundaries) = 0;

    /**
     * Solve the 2D pressure problem.
     */
    virtual bool solve() = 0;

    /**
     * Extract results from solved field.
     */
    virtual TwoDResult extractResults() const = 0;

    /**
     * Get pressure at a point.
     */
    virtual double getPressure(double x, double y) const = 0;

    /**
     * Get average pressure in domain.
     */
    virtual double getAvgPressure() const = 0;

    /**
     * Get average pressure on a boundary.
     */
    virtual double getBoundaryAvgPressure(int boundaryTag) const = 0;

    /**
     * Get pressure drop between two boundaries.
     */
    virtual double getPressureDrop(int inletTag, int outletTag) const = 0;

    /**
     * Export field to file.
     */
    virtual bool exportField(const std::string& filePath) const = 0;

    /**
     * Reset solver state.
     */
    virtual void reset() = 0;
};

// ============================================================================
// SolverFactory - Factory for creating 2D solver instances
// ============================================================================
class SolverFactory {
public:
    enum class BackendType {
        Analytical,     // Analytical/simple solver (for testing)
        FEniCSx,        // FEniCSx backend
        Custom          // Custom backend
    };

    static std::unique_ptr<Heat2DSolver> createHeatSolver(BackendType type = BackendType::Analytical);
    static std::unique_ptr<Pressure2DSolver> createPressureSolver(BackendType type = BackendType::Analytical);
};

// ============================================================================
// AnalyticalHeat2DSolver - Simple analytical solver for testing
// ============================================================================
class AnalyticalHeat2DSolver : public Heat2DSolver {
private:
    TwoDCase caseConfig_;
    std::map<int, Boundary2D> boundaries_;
    bool solved_;
    double avgTemperature_;
    double maxTemperature_;
    std::map<int, double> boundaryHeatFlows_;
    std::map<int, double> boundaryAvgTemps_;
    std::map<int, double> boundaryMaxTemps_;

public:
    AnalyticalHeat2DSolver();

    bool initialize(const TwoDCase& caseConfig) override;
    void applyBoundaries(const std::map<int, Boundary2D>& boundaries) override;
    bool solve() override;
    TwoDResult extractResults() const override;
    double getTemperature(double x, double y) const override;
    double getMaxTemperature() const override;
    double getAvgTemperature() const override;
    double getBoundaryHeatFlow(int boundaryTag) const override;
    double getBoundaryAvgTemperature(int boundaryTag) const override;
    double getBoundaryMaxTemperature(int boundaryTag) const override;
    bool exportField(const std::string& filePath) const override;
    void reset() override;
};

// ============================================================================
// AnalyticalPressure2DSolver - Simple analytical solver for testing
// ============================================================================
class AnalyticalPressure2DSolver : public Pressure2DSolver {
private:
    TwoDCase caseConfig_;
    std::map<int, Boundary2D> boundaries_;
    bool solved_;
    double avgPressure_;
    std::map<int, double> boundaryAvgPressures_;

public:
    AnalyticalPressure2DSolver();

    bool initialize(const TwoDCase& caseConfig) override;
    void applyBoundaries(const std::map<int, Boundary2D>& boundaries) override;
    bool solve() override;
    TwoDResult extractResults() const override;
    double getPressure(double x, double y) const override;
    double getAvgPressure() const override;
    double getBoundaryAvgPressure(int boundaryTag) const override;
    double getPressureDrop(int inletTag, int outletTag) const override;
    bool exportField(const std::string& filePath) const override;
    void reset() override;
};

#endif // TWOD_SOLVER_INTERFACE_H
