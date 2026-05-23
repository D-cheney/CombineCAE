#ifndef NETWORK_IO_V2_H
#define NETWORK_IO_V2_H

#include "core/component.h"
#include "core/material_properties.h"
#include "core/node.h"
#include "solver/fluid_solver/fluid_solver.h"
#include "utils/boundary_conditions.h"
#include "utils/coupling/coupling_config.h"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace NetworkIOv2 {

// ============================================================================
// Version Information
// ============================================================================
constexpr const char* FORMAT_VERSION = "2.0.0";
constexpr const char* FORMAT_NAME_INPUT = "FluidSystemInput";
constexpr const char* FORMAT_NAME_OUTPUT = "FluidSystemOutput";

// ============================================================================
// Value with Unit
// ============================================================================
struct ValueUnit {
    double value;
    std::string unit;
    
    ValueUnit() : value(0.0), unit("") {}
    ValueUnit(double v, const std::string& u = "") : value(v), unit(u) {}
};

// ============================================================================
// Meta Information
// ============================================================================
struct Meta {
    std::string formatName;
    std::string formatVersion;
    std::vector<std::string> compatibleVersions;
    std::string caseId;
    std::string title;
    std::string description;
    std::string author;
    std::string createdAt;
    std::string units;
    std::vector<std::string> tags;
    
    struct Source {
        std::string generator;
        std::string generatorVersion;
    } source;
    
    Meta() : formatName(FORMAT_NAME_INPUT), formatVersion(FORMAT_VERSION), units("SI") {}
};

// ============================================================================
// Node Definition (v2)
// ============================================================================
struct NodeDef {
    std::string id;
    std::string name;
    std::string type;
    std::vector<double> coords;
    
    struct Variables {
        ValueUnit pressure;
        ValueUnit temperature;
        ValueUnit massFlowRate;
    } variables;
    
    struct Constraints {
        bool pressureFixed;
        bool temperatureFixed;
        Constraints() : pressureFixed(false), temperatureFixed(false) {}
    } constraints;
    
    std::map<std::string, std::string> metadata;
    
    NodeDef() : type("junction") {}
};

// ============================================================================
// Port Definition
// ============================================================================
struct PortDef {
    std::string name;
    std::string nodeRef;
    int portIndex;
    
    PortDef() : portIndex(0) {}
};

// ============================================================================
// Component Definition (v2)
// ============================================================================
struct ComponentDef {
    std::string id;
    std::string name;
    std::string type;
    std::string subtype;
    std::vector<PortDef> ports;
    std::string materialRef;
    
    struct Geometry {
        ValueUnit length;
        ValueUnit diameter;
        ValueUnit roughness;
        ValueUnit width;
        ValueUnit height;
        ValueUnit inletDiameter;
        ValueUnit outletDiameter;
    } geometry;
    
    std::map<std::string, std::string> parameters;
    std::vector<std::string> capabilities;
    
    struct Models {
        struct HydraulicModel {
            std::string name;
            std::map<std::string, std::string> options;
        } hydraulic;
        struct ThermalModel {
            std::string name;
            std::map<std::string, std::string> options;
        } thermal;
    } models;
    
    struct InitialState {
        ValueUnit massFlowRate;
        ValueUnit pressureDrop;
    } initialState;
    
    std::map<std::string, std::string> metadata;
    
    ComponentDef() {}
};

// ============================================================================
// Material Definition (v2)
// ============================================================================
struct MaterialDef {
    std::string id;
    std::string name;
    std::string phase;
    std::string fluidType;
    
    struct Properties {
        ValueUnit density;
        ValueUnit dynamicViscosity;
        ValueUnit kinematicViscosity;
        ValueUnit specificHeat;
        ValueUnit thermalConductivity;
        ValueUnit molarMass;
        ValueUnit compressibility;
        ValueUnit expansionCoeff;
        ValueUnit prandtlNumber;
    } properties;
    
    std::map<std::string, std::string> propertyModels;
    
    MaterialDef() : phase("liquid"), fluidType("incompressible") {}
};

// ============================================================================
// Boundary Condition Definition (v2)
// ============================================================================
struct BoundaryDef {
    std::string id;
    
    struct Target {
        std::string kind;
        std::string id;
        std::string port;
    } target;
    
    std::string variable;
    std::string type;
    ValueUnit value;
    
    struct Signal {
        std::string type;
        std::vector<std::pair<double, double>> points;
    } signal;
    
    BoundaryDef() {}
};

// ============================================================================
// Initial State (v2)
// ============================================================================
struct InitialStateDef {
    std::map<std::string, NodeDef::Variables> nodes;
    std::map<std::string, ComponentDef::InitialState> components;
};

// ============================================================================
// Physics Configuration (v2)
// ============================================================================
struct PhysicsDef {
    std::vector<std::string> activeModules;
    
    struct FluidModule {
        std::string formulation;
        bool compressibility;
        bool energy;
        bool species;
        
        FluidModule() : formulation("network_1d"), compressibility(false), energy(false), species(false) {}
    } fluid;
    
    struct ThermalModule {
        bool energyEquation;
        bool solidConduction;
        
        ThermalModule() : energyEquation(false), solidConduction(false) {}
    } thermal;
    
    struct StructureModule {
        bool fsiEnabled;
        
        StructureModule() : fsiEnabled(false) {}
    } structure;
    
    PhysicsDef() {
        activeModules.push_back("fluid");
    }
};

// ============================================================================
// Solver Control (v2)
// ============================================================================
struct SolverControl {
    std::string steadyMethod;
    std::string linearMethod;
    double tolerance;
    double relativeTolerance;
    double incrementTolerance;
    int maxIterations;
    double relaxationFactor;
    std::string backend;
    int cpuThreads;
    
    struct GPU {
        bool enabled;
        int deviceId;
        GPU() : enabled(false), deviceId(0) {}
    } gpu;
    
    SolverControl()
        : steadyMethod("NewtonRaphson"),
          linearMethod("GaussSeidel"),
          tolerance(1e-6),
          relativeTolerance(1e-8),
          incrementTolerance(1e-8),
          maxIterations(200),
          relaxationFactor(1.0),
          backend("cpu"),
          cpuThreads(0) {}
};

// ============================================================================
// Time Control (v2)
// ============================================================================
struct TimeControl {
    double start;
    double end;
    double timeStep;
    double outputInterval;
    
    struct Adaptive {
        bool enabled;
        double minStep;
        double maxStep;
        Adaptive() : enabled(false), minStep(1e-5), maxStep(0.1) {}
    } adaptive;
    
    TimeControl() : start(0.0), end(1.0), timeStep(0.01), outputInterval(0.1) {}
};

// ============================================================================
// Output Control (v2)
// ============================================================================
struct OutputControl {
    bool saveFinalState;
    bool saveRestart;
    bool saveFrames;
    std::vector<std::string> frameVariables;
    
    OutputControl() : saveFinalState(true), saveRestart(true), saveFrames(true) {}
};

// ============================================================================
// Controls (v2)
// ============================================================================
struct ControlsDef {
    std::string analysisType;
    SolverControl solver;
    TimeControl time;
    OutputControl outputControl;
    
    struct Numerics {
        bool underRelaxation;
        std::string jacobianUpdatePolicy;
        Numerics() : underRelaxation(true), jacobianUpdatePolicy("every_iteration") {}
    } numerics;
    
    ControlsDef() : analysisType("steady") {}
};

// ============================================================================
// Case Definition (v2)
// ============================================================================
struct CaseDef {
    std::string id;
    std::string name;
    bool enabled;
    
    struct AnalysisOverride {
        std::string analysisType;
    } analysisOverride;
    
    std::vector<BoundaryDef> boundaryConditions;
    
    struct ComponentControl {
        std::string targetId;
        std::string variable;
        double value;
    };
    std::vector<ComponentControl> componentControls;
    
    CaseDef() : enabled(true) {}
};

// ============================================================================
// Frame Definition (v2) - for transient input
// ============================================================================
struct FrameDef {
    double time;
    
    struct BoundaryUpdate {
        BoundaryDef::Target target;
        std::string variable;
        std::string type;
        ValueUnit value;
    };
    std::vector<BoundaryUpdate> boundaryUpdates;
    
    struct ControlUpdate {
        BoundaryDef::Target target;
        std::string variable;
        double value;
    };
    std::vector<ControlUpdate> controlUpdates;
    
    std::map<std::string, std::string> exchangeInputs;
    
    FrameDef() : time(0.0) {}
};

// ============================================================================
// Exchange Configuration (v2)
// ============================================================================
struct ExchangeDef {
    bool enabled;
    std::string mode;
    
    struct Channel {
        std::string id;
        std::string direction;
        std::string provider;
        std::string consumer;
        
        struct Mapping {
            std::string source;
            std::string target;
        };
        std::vector<Mapping> mapping;
        
        std::string updatePolicy;
    };
    std::vector<Channel> channels;
    
    ExchangeDef() : enabled(false), mode("file_based") {}
};

// ============================================================================
// Extensions (v2)
// ============================================================================
struct ExtensionsDef {
    std::map<std::string, std::string> vendorSpecific;
    std::map<std::string, std::string> customModels;
    std::map<std::string, std::string> uiHints;
};

// ============================================================================
// Input Configuration (v2)
// ============================================================================
struct InputConfig {
    Meta meta;
    
    struct Model {
        std::string dimension;
        std::string topologyType;
        std::vector<NodeDef> nodes;
        std::vector<ComponentDef> components;
        std::vector<MaterialDef> materials;
        std::vector<BoundaryDef> boundaryTemplates;
        InitialStateDef initialState;
        
        Model() : dimension("1D"), topologyType("network") {}
    } model;
    
    PhysicsDef physics;
    ControlsDef controls;
    std::vector<CaseDef> cases;
    std::vector<FrameDef> frames;
    ExchangeDef exchange;
    ExtensionsDef extensions;
};

// ============================================================================
// Output State (v2)
// ============================================================================
struct OutputState {
    double time;
    std::map<std::string, NodeDef::Variables> nodes;
    std::map<std::string, ComponentDef::InitialState> components;
};

// ============================================================================
// Output Frame (v2)
// ============================================================================
struct OutputFrame {
    int index;
    double time;
    double stepSize;
    bool converged;
    int iterations;
    double residual;
    OutputState state;
    std::map<std::string, std::string> exchangeOutputs;
};

// ============================================================================
// Restart Data (v2)
// ============================================================================
struct RestartData {
    bool available;
    OutputState recommendedState;
    double time;
    
    RestartData() : available(false), time(0.0) {}
};

// ============================================================================
// Output Result (v2)
// ============================================================================
struct OutputConfig {
    Meta meta;
    
    struct ModelRef {
        std::string caseId;
        std::string inputFile;
        std::string modelHash;
    } modelRef;
    
    PhysicsDef physics;
    ControlsDef controls;
    
    struct Execution {
        bool success;
        std::string analysisType;
        double elapsedSeconds;
        std::string backend;
        int cpuThreads;
        bool gpuUsed;
        std::vector<std::string> warnings;
        std::vector<std::string> errors;
        
        Execution() : success(true), elapsedSeconds(0.0), cpuThreads(0), gpuUsed(false) {}
    } execution;
    
    struct Summary {
        bool converged;
        int totalTimeSteps;
        int savedFrames;
        double finalTime;
        int finalIterationCount;
        double maxResidual;
        
        struct Monitors {
            double maxPressure;
            double minPressure;
            double maxTemperature;
            double maxMassFlowRate;
            Monitors() : maxPressure(0.0), minPressure(0.0), maxTemperature(0.0), maxMassFlowRate(0.0) {}
        } monitors;
        
        Summary() : converged(true), totalTimeSteps(0), savedFrames(0), finalTime(0.0),
                    finalIterationCount(0), maxResidual(0.0) {}
    } summary;
    
    struct Results {
        OutputState finalState;
        std::vector<double> residualHistory;
    } results;
    
    std::vector<OutputFrame> frames;
    RestartData restart;
    ExchangeDef exchange;
    ExtensionsDef extensions;
    
    OutputConfig() {
        meta.formatName = FORMAT_NAME_OUTPUT;
        meta.formatVersion = FORMAT_VERSION;
    }
};

// ============================================================================
// Public API
// ============================================================================

// Format detection
enum class FormatVersion {
    V1_Legacy,
    V2_0
};

FormatVersion detectFormat(const std::string& jsonContent);

// V2 Input loading
bool loadInputV2(const std::string& filePath, FluidNetworkSolver& solver, InputConfig* config = nullptr);
bool loadInputV2FromString(const std::string& jsonContent, FluidNetworkSolver& solver, InputConfig* config = nullptr);

// V2 Output generation
bool dumpOutputV2(const FluidNetworkSolver& solver, const OutputConfig& extraConfig, const std::string& filePath);
bool dumpOutputV2SteadyState(const FluidNetworkSolver& solver, const std::string& filePath);
bool dumpOutputV2Transient(const FluidNetworkSolver& solver, const std::string& filePath);

// Restart support
bool exportRestartFile(const OutputConfig& output, const std::string& filePath);
bool loadRestartFile(const std::string& filePath, InitialStateDef& initialState, double& time);

// Utility functions
MaterialProperties materialDefToProperties(const MaterialDef& def);
MaterialDef propertiesToMaterialDef(const MaterialProperties& props, const std::string& id);

} // namespace NetworkIOv2

#endif // NETWORK_IO_V2_H
