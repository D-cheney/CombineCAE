/**
 * @file coupling_types.h
 * @brief Core data structures for 1D-2D coupling framework.
 *
 * This header defines the foundational types for the coupling system:
 * - TwoDCase: 2D simulation case configuration
 * - TwoDResult: 2D simulation results
 * - CouplingInterface: 1D-2D interface definition
 * - CouplingStrategy: Overall coupling strategy
 * - CouplingHistory: Iteration history
 */

#ifndef COUPLING_TYPES_H
#define COUPLING_TYPES_H

#include <string>
#include <vector>
#include <map>
#include <unordered_map>

// ============================================================================
// Geometry Specification
// ============================================================================
struct GeometrySpec {
    std::string type;       // "rectangle", "channel", "plate", etc.
    double width;           // Width (m)
    double height;          // Height (m)
    double thickness;       // Thickness (m), for 2D plane
    std::string meshFile;   // Optional: pre-generated mesh file
    double meshSize;        // Target mesh size (m)
    
    GeometrySpec()
        : type("rectangle"), width(1.0), height(1.0), thickness(0.01),
          meshSize(0.01) {}
};

// ============================================================================
// Material Definition for 2D
// ============================================================================
struct Material2D {
    std::string region;     // Region name
    double thermalConductivity; // k (W/(m*K))
    double density;         // rho (kg/m3)
    double specificHeat;    // cp (J/(kg*K))
    double permeability;    // For pressure/Darcy model (m2)
    double viscosity;       // For pressure model (Pa*s)
    
    Material2D()
        : region("default"), thermalConductivity(1.0),
          density(1000.0), specificHeat(1000.0),
          permeability(1e-12), viscosity(0.001) {}
};

// ============================================================================
// Boundary Definition
// ============================================================================
struct Boundary2D {
    int tag;                // Gmsh physical group tag
    std::string name;       // Boundary name
    std::string type;       // "dirichlet", "neumann", "robin"
    double value;           // Boundary value (T, q, h, p, etc.)
    double referenceValue;  // Reference value (e.g., T_fluid for Robin)
    
    Boundary2D()
        : tag(0), type("dirichlet"), value(0.0), referenceValue(0.0) {}
};

// ============================================================================
// Solver Options for 2D
// ============================================================================
struct SolverOptions2D {
    std::string linearSolver;   // "direct", "iterative"
    double tolerance;           // Solver tolerance
    int maxIterations;          // Max iterations (for iterative)
    bool verbose;               // Enable verbose output
    
    SolverOptions2D()
        : linearSolver("direct"), tolerance(1e-10),
          maxIterations(1000), verbose(false) {}
};

// ============================================================================
// TwoDCase: Complete 2D simulation case configuration
// ============================================================================
struct TwoDCase {
    std::string id;                         // Case unique ID
    std::string name;                       // Case name
    std::string physics;                    // "heat", "pressure", "thermo_hydraulic"
    GeometrySpec geometry;                  // Geometry specification
    std::vector<Material2D> materials;      // Material definitions
    std::vector<Boundary2D> boundaries;     // Boundary definitions
    std::map<int, std::string> boundaryNames; // Tag -> name mapping
    SolverOptions2D solverOptions;          // Solver configuration
    bool enabled;                           // Whether case is active
    
    TwoDCase() : physics("heat"), enabled(true) {}
    
    // Get boundary by tag
    const Boundary2D* getBoundary(int tag) const {
        for (const auto& b : boundaries) {
            if (b.tag == tag) return &b;
        }
        return nullptr;
    }
    
    // Get boundary by name
    const Boundary2D* getBoundaryByName(const std::string& name) const {
        for (const auto& b : boundaries) {
            if (b.name == name) return &b;
        }
        return nullptr;
    }
    
    // Get material by region
    const Material2D* getMaterial(const std::string& region) const {
        for (const auto& m : materials) {
            if (m.region == region) return &m;
        }
        return nullptr;
    }
};

// ============================================================================
// TwoDResult: 2D simulation results
// ============================================================================
struct TwoDResult {
    std::string caseId;                     // Associated case ID
    bool success;                           // Whether solve succeeded
    std::string physics;                    // Physics type
    double solveTime;                       // Solve time (seconds)
    
    // Scalar metrics
    std::unordered_map<std::string, double> scalarMetrics;
    // e.g., "max_temperature", "avg_temperature", "total_heat_flow"
    
    // Boundary integrals
    std::unordered_map<int, double> boundaryHeatFlow;       // tag -> total heat flow (W)
    std::unordered_map<int, double> boundaryAvgTemperature; // tag -> avg temperature (K)
    std::unordered_map<int, double> boundaryAvgPressure;    // tag -> avg pressure (Pa)
    std::unordered_map<int, double> boundaryMaxTemperature; // tag -> max temperature (K)
    std::unordered_map<int, double> boundaryPressureDrop;   // tag -> pressure drop (Pa)
    
    // Field file path (for visualization)
    std::string fieldFile;
    
    TwoDResult() : success(false), physics(""), solveTime(0.0) {}
    
    // Get scalar metric
    double getScalar(const std::string& key, double defaultVal = 0.0) const {
        auto it = scalarMetrics.find(key);
        return (it != scalarMetrics.end()) ? it->second : defaultVal;
    }
    
    // Set scalar metric
    void setScalar(const std::string& key, double value) {
        scalarMetrics[key] = value;
    }
};

// ============================================================================
// Transfer Definition: How data is transferred between 1D and 2D
// ============================================================================
struct TransferDef {
    std::string type;             // "dirichlet_temperature", "robin_temperature",
                                  // "dirichlet_pressure", "neumann_heatflow"
    std::string sourceTemperature; // Source field in 1D (e.g., "fluid_temperature")
    std::string sourceH;           // Source h coefficient (e.g., "heat_transfer_coeff")
    std::string sourcePressure;    // Source pressure field in 1D
    
    TransferDef()
        : type("dirichlet_temperature"),
          sourceTemperature("fluid_temperature"),
          sourceH("heat_transfer_coeff") {}
};

// ============================================================================
// Feedback Definition: What data is fed back from 2D to 1D
// ============================================================================
struct FeedbackDef {
    std::string heatflow;           // Target field in 1D for heat flow
    std::string wallTemperatureAvg; // Target field for avg wall temperature
    std::string wallTemperatureMax; // Target field for max wall temperature
    std::string pressureDrop;       // Target field for additional pressure drop
    std::string resistanceCoeff;    // Target field for resistance coefficient
    
    FeedbackDef()
        : heatflow("wall_heat_flow"),
          wallTemperatureAvg("wall_T_avg"),
          wallTemperatureMax("wall_T_max"),
          pressureDrop("delta_p_local"),
          resistanceCoeff("resistance_coeff") {}
};

// ============================================================================
// CouplingInterface: Definition of a single 1D-2D coupling interface
// ============================================================================
struct CouplingInterface {
    std::string name;                   // Interface unique name
    std::string onedComponent;          // 1D component ID/name
    std::string onedSide;               // 1D side: "wall", "inlet", "outlet"
    std::string twodCase;               // 2D case ID
    int twodBoundaryTag;                // 2D boundary physical group tag
    TransferDef transferTo2D;           // How to transfer 1D -> 2D
    FeedbackDef feedbackTo1D;           // How to feedback 2D -> 1D
    bool enabled;                       // Whether interface is active
    
    CouplingInterface()
        : onedSide("wall"), twodBoundaryTag(0), enabled(true) {}
};

// ============================================================================
// CouplingStrategy: Overall coupling strategy configuration
// ============================================================================
struct CouplingStrategy {
    std::string mode;               // "one_way", "staggered", "strong_coupled_reserved"
    int maxOuterIterations;         // Max outer coupling iterations
    double relaxationFactor;        // Relaxation factor (0-1)
    double toleranceTemperature;    // Temperature convergence tolerance (K)
    double tolerancePressure;       // Pressure convergence tolerance (Pa)
    double toleranceHeatFlow;       // Heat flow convergence tolerance (W)
    bool useAitkenAcceleration;     // Enable Aitken acceleration
    
    CouplingStrategy()
        : mode("one_way"), maxOuterIterations(20), relaxationFactor(0.5),
          toleranceTemperature(1e-4), tolerancePressure(1e-3),
          toleranceHeatFlow(1e-4), useAitkenAcceleration(false) {}
};

// ============================================================================
// CouplingHistory: Iteration history for monitoring
// ============================================================================
struct CouplingIterationRecord {
    int iteration;
    double maxTemperatureChange;
    double maxPressureChange;
    double maxHeatFlowChange;
    double relaxationFactor;
    bool converged;
    
    CouplingIterationRecord()
        : iteration(0), maxTemperatureChange(0.0), maxPressureChange(0.0),
          maxHeatFlowChange(0.0), relaxationFactor(1.0), converged(false) {}
};

struct CouplingHistory {
    std::vector<CouplingIterationRecord> records;
    int totalIterations;
    bool converged;
    double finalMaxChange;
    
    CouplingHistory() : totalIterations(0), converged(false), finalMaxChange(0.0) {}
};

// ============================================================================
// CouplingConfig: Complete coupling configuration
// ============================================================================
struct CouplingConfig {
    CouplingStrategy strategy;
    std::vector<TwoDCase> twodCases;
    std::vector<CouplingInterface> couplingInterfaces;
    bool enabled;
    
    CouplingConfig() : enabled(false) {}
    
    // Get 2D case by ID
    TwoDCase* getTwoDCase(const std::string& id) {
        for (auto& c : twodCases) {
            if (c.id == id) return &c;
        }
        return nullptr;
    }
    
    const TwoDCase* getTwoDCase(const std::string& id) const {
        for (const auto& c : twodCases) {
            if (c.id == id) return &c;
        }
        return nullptr;
    }
    
    // Get coupling interface by name
    CouplingInterface* getInterface(const std::string& name) {
        for (auto& iface : couplingInterfaces) {
            if (iface.name == name) return &iface;
        }
        return nullptr;
    }
};

#endif // COUPLING_TYPES_H
