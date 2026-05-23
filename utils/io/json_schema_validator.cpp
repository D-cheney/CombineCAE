#include "json_schema_validator.h"
#include "utils/io/json_mini.h"

#include <sstream>
#include <algorithm>
#include <iostream>

#ifdef USE_NLOHMANN_JSON_SCHEMA
#include <nlohmann/json.hpp>
#include <nlohmann/json-schema.hpp>
#endif

namespace NetworkIO
{

    std::vector<std::string> JsonSchemaValidator::getValidComponentTypes()
    {
        return {
            "pipe", "bend", "channel", "contraction", "expander",
            "t_joint", "y_joint", "local_loss",
            "seal",
            "liquid_filter", "gas_filter", "mesh_filter", "paper_filter",
            "level_tank", "pressure_vessel",
            "hydraulic_accumulator", "pneumatic_accumulator",
            "gate_valve", "ball_valve", "butterfly_valve",
            "control_valve", "flow_control_valve", "isolation_valve",
            "check_valve", "pressure_relief_valve", "gas_regulator",
            "pump", "centrifugal_pump", "piston_pump", "gear_pump", "screw_pump",
            "heat_exchanger", "radiator",
            "oil_water_separator",
            "coupling_flow", "coupling_thermal", "coupling_link",
            // Legacy aliases
            "tjoint", "yjoint",
            "liquidfilter", "gasfilter", "meshfilter", "paperfilter",
            "leveltank", "pressurevessel",
            "hydraulicaccumulator", "pneumaticaccumulator",
            "gatevalve", "ballvalve", "butterflyvalve",
            "controlvalve", "flowcontrolvalve", "isolationvalve",
            "checkvalve", "pressurereliefvalve", "gasregulator",
            "centrifugalpump", "pistonpump", "gearpump", "screwpump",
            "heatexchanger",
            "oilwaterseparator",
            "couplingflow", "couplingthermal", "couplinglink"};
    }

    std::vector<std::string> JsonSchemaValidator::getValidSolverMethods()
    {
        return {
            "GaussSeidel",
            "SimpleIteration",
            "LinearNetworkSolver",
            "NewtonRaphson",
            "SIMPLE"};
    }

    bool JsonSchemaValidator::isEnumValid(const std::string &value, const std::set<std::string> &allowed)
    {
        return allowed.find(value) != allowed.end();
    }

    std::string formatValidationErrors(const std::vector<ValidationError> &errors)
    {
        std::ostringstream oss;
        if (errors.empty())
        {
            oss << "No validation errors";
        }
        else
        {
            oss << "JSON Schema validation failed with " << errors.size() << " error(s):\n";
            for (size_t i = 0; i < errors.size(); ++i)
            {
                oss << "  [" << (i + 1) << "] " << errors[i].toString() << "\n";
            }
        }
        return oss.str();
    }

    bool JsonSchemaValidator::validateTopLevel(const std::string &jsonString,
                                               std::vector<ValidationError> &errors)
    {
        // Check for required root-level sections
        bool hasDescription = jsonString.find("\"description\"") != std::string::npos;
        bool hasSimulationName = jsonString.find("\"simulationName\"") != std::string::npos || jsonString.find("\"name\"") != std::string::npos;
        bool hasComponents = jsonString.find("\"components\"") != std::string::npos;

        // Components array is required
        if (!hasComponents)
        {
            errors.push_back(ValidationError("$",
                                             "Missing required field 'components'",
                                             "array of component objects",
                                             "not present"));
            return false;
        }

        return true;
    }

    bool JsonSchemaValidator::validateComponentArray(const std::string &componentSection,
                                                     std::vector<ValidationError> &errors)
    {
        auto validTypes = getValidComponentTypes();
        std::set<std::string> validTypeSet(validTypes.begin(), validTypes.end());

        // Parse component objects (simple approach)
        size_t compCount = 0;
        size_t pos = 0;

        // Count and validate components
        while ((pos = componentSection.find("\"type\"", pos)) != std::string::npos)
        {
            std::string typeValue;
            size_t valueStart = componentSection.find("\"", pos + 7);
            if (valueStart != std::string::npos)
            {
#ifdef USE_NLOHMANN_JSON_SCHEMA
                // If nlohmann json-schema-validator is available, use it for robust validation
                try
                {
                    auto j = nlohmann::json::parse(jsonString);
                    // look for schema file next to executable or docs/network_schema.json
                    std::ifstream sfile("docs/network_schema.json");
                    if (sfile.is_open())
                    {
                        nlohmann::json schema_json = nlohmann::json::parse(sfile);
                        nlohmann::json_schema::json_validator validator;
                        validator.set_root_schema(schema_json);
                        validator.validate(j);
                        return true;
                    }
                }
                catch (const std::exception &ex)
                {
                    errors.push_back(ValidationError("$", std::string("nlohmann schema validation error: ") + ex.what()));
                    return false;
                }
#endif
                valueStart++;
                size_t valueEnd = componentSection.find("\"", valueStart);
                if (valueEnd != std::string::npos)
                {
                    typeValue = componentSection.substr(valueStart, valueEnd - valueStart);

                    // Convert to lowercase for comparison
                    std::string lowerType = typeValue;
                    std::transform(lowerType.begin(), lowerType.end(), lowerType.begin(), ::tolower);

                    if (!isEnumValid(lowerType, validTypeSet))
                    {
                        std::ostringstream oss;
                        oss << "components[" << compCount << "].type";
                        std::string validList = "";
                        for (size_t i = 0; i < std::min(size_t(5), validTypes.size()); ++i)
                        {
                            validList += validTypes[i];
                            if (i < 4)
                                validList += ", ";
                        }
                        validList += "...";
                        errors.push_back(ValidationError(oss.str(),
                                                         "Invalid component type",
                                                         validList,
                                                         typeValue));
                    }
                }
                compCount++;
            }
            pos += 7;
        }

        return errors.empty();
    }

    bool JsonSchemaValidator::validateConnectionArray(const std::string &connectionSection,
                                                      std::vector<ValidationError> &errors)
    {
        // Validate connection structure
        // Check for from_node, to_node, component_index
        size_t connCount = 0;
        size_t pos = 0;

        // Simple check: ensure connections have required fields
        while ((pos = connectionSection.find("\"from_node\"", pos)) != std::string::npos)
        {
            size_t toNodePos = connectionSection.find("\"to_node\"", pos);
            size_t compIndexPos = connectionSection.find("\"component_index\"", pos);

            if (toNodePos == std::string::npos || compIndexPos == std::string::npos)
            {
                std::ostringstream oss;
                oss << "connections[" << connCount << "]";
                errors.push_back(ValidationError(oss.str(),
                                                 "Missing required fields in connection",
                                                 "from_node, to_node, component_index",
                                                 "incomplete"));
            }

            connCount++;
            pos += 11;
        }

        return errors.empty();
    }

    bool JsonSchemaValidator::validate(const std::string &jsonString,
                                       std::vector<ValidationError> &errors)
    {
        errors.clear();

        // Check if JSON is empty
        if (jsonString.empty())
        {
            errors.push_back(ValidationError("$", "Empty JSON input", "valid JSON object", "empty string"));
            return false;
        }

        // Check basic JSON structure (braces)
        if (jsonString.front() != '{' || jsonString.back() != '}')
        {
            errors.push_back(ValidationError("$",
                                             "Invalid JSON structure",
                                             "JSON object (starts with { and ends with })",
                                             "not a valid JSON object"));
            return false;
        }

        // Validate top-level structure
        if (!validateTopLevel(jsonString, errors))
        {
            return false;
        }

        // Validate components array if present
        size_t compStart = jsonString.find("\"components\"");
        if (compStart != std::string::npos)
        {
            size_t arrayStart = jsonString.find("[", compStart);
            size_t arrayEnd = jsonString.find("]", arrayStart);
            if (arrayStart != std::string::npos && arrayEnd != std::string::npos)
            {
                std::string componentArray = jsonString.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                validateComponentArray(componentArray, errors);
            }
        }

        // Validate connections array if present
        size_t connStart = jsonString.find("\"connections\"");
        if (connStart != std::string::npos)
        {
            size_t arrayStart = jsonString.find("[", connStart);
            size_t arrayEnd = jsonString.find("]", arrayStart);
            if (arrayStart != std::string::npos && arrayEnd != std::string::npos)
            {
                std::string connectionArray = jsonString.substr(arrayStart + 1, arrayEnd - arrayStart - 1);
                validateConnectionArray(connectionArray, errors);
            }
        }

        // Validate solver method if present
        size_t solverStart = jsonString.find("\"solverMethod\"");
        if (solverStart != std::string::npos)
        {
            size_t valueStart = jsonString.find("\"", solverStart + 15);
            if (valueStart != std::string::npos)
            {
                valueStart++;
                size_t valueEnd = jsonString.find("\"", valueStart);
                if (valueEnd != std::string::npos)
                {
                    std::string solverMethod = jsonString.substr(valueStart, valueEnd - valueStart);
                    auto validMethods = getValidSolverMethods();
                    std::set<std::string> validSet(validMethods.begin(), validMethods.end());

                    if (!isEnumValid(solverMethod, validSet))
                    {
                        errors.push_back(ValidationError("$.solverMethod",
                                                         "Invalid solver method",
                                                         "GaussSeidel, SimpleIteration, LinearNetworkSolver, NewtonRaphson, SIMPLE",
                                                         solverMethod));
                    }
                }
            }
        }

        return errors.empty();
    }

} // namespace NetworkIO
