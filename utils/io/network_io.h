#ifndef NETWORK_IO_H
#define NETWORK_IO_H

#include "solver/fluid_solver/fluid_solver.h"
#include "utils/coupling/coupling_config.h"

#include <map>
#include <string>
#include <vector>

// Include v2 format support
#include "network_io_v2.h"

namespace NetworkIO
{

    struct RunConfig
    {
        std::string description;
        std::string simulationName;
        std::string simulationDimension;
        std::vector<std::string> physicsModules;
        std::string couplingMode;
        std::string meshPrimaryFile;
        std::vector<std::string> interfaceNames;
        std::map<std::string, std::string> extensionFields;

        std::string solverMethod;
        double tolerance;
        int maxIterations;
        double relaxationFactor;
        int cpuThreads;
        std::string computeBackend;
        bool debugEnabled;
        CouplingConfig coupling;

        // Enhanced convergence parameters for Newton-Raphson
        double relativeConvergenceTolerance;
        double solutionIncrementTolerance;

        bool transientEnabled;
        double transientStartTime;
        double transientEndTime;
        double transientTimeStep;
        double transientOutputInterval;

        RunConfig()
            : description(""),
              simulationName(""),
              simulationDimension("1D"),
              couplingMode("none"),
              meshPrimaryFile(""),
              solverMethod("GaussSeidel"),
              tolerance(1e-8),
              maxIterations(5000),
              relaxationFactor(1.0),
              cpuThreads(0),
              computeBackend(""),
              debugEnabled(false),
              coupling(),
              relativeConvergenceTolerance(1e-6),
              solutionIncrementTolerance(1e-6),
              transientEnabled(false),
              transientStartTime(0.0),
              transientEndTime(1.0),
              transientTimeStep(0.1),
              transientOutputInterval(0.1)
        {
            physicsModules.push_back("fluid");
        }
    };

    bool loadFromFile(const std::string &filePath, FluidNetworkSolver &solver, RunConfig *config = nullptr);
    bool loadFromTextFile(const std::string &filePath, FluidNetworkSolver &solver, RunConfig *config = nullptr);
    bool loadFromJsonFile(const std::string &filePath, FluidNetworkSolver &solver, RunConfig *config = nullptr);

    bool dumpSteadyStateToJson(const FluidNetworkSolver &solver,
                               const std::string &filePath = "simulation_results.json");
    bool dumpTransientToJson(const FluidNetworkSolver &solver,
                             const std::string &filePath = "transient_results.json");

    bool validateSolverNetwork(const FluidNetworkSolver &solver, std::string *errorMessage = nullptr);

    /**
     * @brief Get list of all registered component types from the registry.
     *
     * Useful for validation and user feedback.
     *
     * @return Vector of registered type names (e.g., "pipe", "valve", etc.)
     */
    std::vector<std::string> getRegisteredComponentTypes();

    /**
     * @brief Check if a component type is registered in the factory.
     *
     * @param type_name The component type name to check
     * @return true if the type exists in the registry
     */
    bool isComponentTypeRegistered(const std::string &type_name);

    /**
     * @brief Helper function to create a component from JSON-extracted parameters.
     *
     * This function centralizes component creation logic, reducing code duplication
     * in the loadFromJsonFile function. It handles type-specific parameter extraction
     * and component instantiation.
     *
     * @param typeKey The normalized component type key (lowercase)
     * @param componentName The name to assign to the component
     * @param fluid The fluid properties to use for the component
     * @param params A map of parameter names to string values (extracted from JSON)
     * @return A unique_ptr to the created Component, or nullptr if creation failed
     */
    std::unique_ptr<Component> createComponentFromParams(
        const std::string &typeKey,
        const std::string &componentName,
        const MaterialProperties &fluid,
        const std::map<std::string, std::string> &params);

    // ========================================================================
    // V2 Format Support
    // ========================================================================
    
    /**
     * @brief Detect JSON format version (v1 legacy or v2.0)
     * 
     * @param jsonContent JSON string content
     * @return FormatVersion enum value
     */
    NetworkIOv2::FormatVersion detectFormatVersion(const std::string &jsonContent);
    
    /**
     * @brief Load input file with automatic format detection
     * 
     * Automatically detects v1 or v2 format and loads accordingly.
     * 
     * @param filePath Path to input file
     * @param solver FluidNetworkSolver to configure
     * @param config Optional RunConfig to populate (v1 format)
     * @param configV2 Optional InputConfig to populate (v2 format)
     * @return true if successful
     */
    bool loadFromFileAuto(const std::string &filePath, 
                          FluidNetworkSolver &solver, 
                          RunConfig *config = nullptr,
                          NetworkIOv2::InputConfig *configV2 = nullptr);
    
    /**
     * @brief Load from v2 format JSON file
     * 
     * @param filePath Path to v2 format input file
     * @param solver FluidNetworkSolver to configure
     * @param config Optional InputConfig to populate
     * @return true if successful
     */
    bool loadFromJsonFileV2(const std::string &filePath,
                            FluidNetworkSolver &solver,
                            NetworkIOv2::InputConfig *config = nullptr);
    
    /**
     * @brief Dump results in v2 format
     * 
     * @param solver FluidNetworkSolver with results
     * @param filePath Output file path
     * @return true if successful
     */
    bool dumpToJsonV2(const FluidNetworkSolver &solver,
                      const std::string &filePath = "simulation_results_v2.json");
    
    /**
     * @brief Export restart file from current solver state
     * 
     * @param solver FluidNetworkSolver with results
     * @param filePath Restart file path
     * @return true if successful
     */
    bool exportRestartFile(const FluidNetworkSolver &solver,
                           const std::string &filePath = "restart_input.json");

} // namespace NetworkIO

#endif // NETWORK_IO_H
