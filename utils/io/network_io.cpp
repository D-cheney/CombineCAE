#include "network_io.h"

#include "core/components.h"
#include "core/component_registry.h"
#include "core/material_library.h"
#include "json_schema_validator.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <memory>
#include <sstream>
#include <vector>

namespace
{

    std::string trim(const std::string &str)
    {
        const std::size_t start = str.find_first_not_of(" \t\n\r");
        if (start == std::string::npos)
        {
            return "";
        }
        const std::size_t end = str.find_last_not_of(" \t\n\r");
        return str.substr(start, end - start + 1);
    }

    std::string toUpperCopy(const std::string &text)
    {
        std::string out = text;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c)
                       { return static_cast<char>(std::toupper(c)); });
        return out;
    }

    std::string toLowerCopy(const std::string &text)
    {
        std::string out = text;
        std::transform(out.begin(), out.end(), out.begin(),
                       [](unsigned char c)
                       { return static_cast<char>(std::tolower(c)); });
        return out;
    }

    bool parseDouble(const std::string &text, double &value)
    {
        try
        {
            value = std::stod(text);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool parseInt(const std::string &text, int &value)
    {
        try
        {
            value = std::stoi(text);
            return true;
        }
        catch (...)
        {
            return false;
        }
    }

    bool parseBool(const std::string &text, bool &value)
    {
        const std::string key = toLowerCopy(trim(text));
        if (key == "true" || key == "1" || key == "yes" || key == "on")
        {
            value = true;
            return true;
        }
        if (key == "false" || key == "0" || key == "no" || key == "off")
        {
            value = false;
            return true;
        }
        return false;
    }

    double clampRelaxation(double value)
    {
        if (value <= 0.0)
        {
            return 1.0;
        }
        if (value > 1.0)
        {
            return 1.0;
        }
        return value;
    }

    constexpr double kPi = 3.14159265358979323846;

    const char *steadySolverTypeName(SteadySolverType type)
    {
        switch (type)
        {
        case SteadySolverType::SimpleIteration:
            return "SimpleIteration";
        case SteadySolverType::NewtonRaphson:
            return "NewtonRaphson";
        case SteadySolverType::LinearGaussianElimination:
            return "LinearGaussianElimination";
        case SteadySolverType::LinearGaussSeidel:
            return "LinearGaussSeidel";
        case SteadySolverType::LinearParallelGaussian:
            return "LinearParallelGaussian";
        case SteadySolverType::LinearJacobiParallel:
            return "LinearJacobiParallel";
        case SteadySolverType::LinearConjugateGradient:
            return "LinearConjugateGradient";
        case SteadySolverType::LinearCudaDense:
            return "LinearCudaDense";
        default:
            return "Unknown";
        }
    }

    struct FluidPropsView
    {
        double density;
        double dynamicViscosity;
        double kinematicViscosity;
        double specificHeat;
        double thermalConductivity;
        bool compressible;
    };

    double computeDensityFromMaterial(const MaterialProperties &material,
                                      double pressure,
                                      double temperature)
    {
        if (material.fluidType == FluidType::Compressible &&
            pressure > 0.0 && temperature > 0.0)
        {
            return pressure / (287.0 * temperature);
        }
        return material.density;
    }

    FluidPropsView makePropsFromMaterial(const MaterialProperties &material,
                                         double pressure,
                                         double temperature)
    {
        FluidPropsView props;
        props.density = computeDensityFromMaterial(material, pressure, temperature);
        props.dynamicViscosity = material.dynamicViscosity;
        props.kinematicViscosity =
            (props.density > 0.0) ? props.dynamicViscosity / props.density
                                  : material.kinematicViscosity;
        props.specificHeat = material.specificHeat;
        props.thermalConductivity = material.thermalConductivity;
        props.compressible = material.fluidType == FluidType::Compressible;
        return props;
    }

    FluidPropsView makePropsFromHeatExchanger(const HeatExchanger &hex)
    {
        FluidPropsView props;
        props.density = hex.getDensity();
        props.dynamicViscosity = hex.getViscosity();
        props.kinematicViscosity =
            (props.density > 0.0) ? props.dynamicViscosity / props.density : 0.0;
        props.specificHeat = hex.getSpecificHeat();
        props.thermalConductivity = hex.getThermalConductivity();
        props.compressible = false;
        return props;
    }

    FluidPropsView getComponentFluidProps(const Component *comp,
                                          const MaterialProperties &fallback,
                                          double pressure,
                                          double temperature)
    {
        if (const auto *pipe = dynamic_cast<const Pipe *>(comp))
        {
            return makePropsFromMaterial(pipe->getMaterial(), pressure, temperature);
        }
        if (const auto *valve = dynamic_cast<const Valve *>(comp))
        {
            return makePropsFromMaterial(valve->getMaterial(), pressure, temperature);
        }
        if (const auto *pump = dynamic_cast<const Pump *>(comp))
        {
            return makePropsFromMaterial(pump->getMaterial(), pressure, temperature);
        }
        if (const auto *loss = dynamic_cast<const LocalLossComponent *>(comp))
        {
            return makePropsFromMaterial(loss->getMaterial(), pressure, temperature);
        }
        if (const auto *channel = dynamic_cast<const Channel *>(comp))
        {
            return makePropsFromMaterial(channel->getMaterial(), pressure, temperature);
        }
        if (const auto *hex = dynamic_cast<const HeatExchanger *>(comp))
        {
            (void)pressure;
            (void)temperature;
            return makePropsFromHeatExchanger(*hex);
        }
        return makePropsFromMaterial(fallback, pressure, temperature);
    }

    double componentFlowArea(const Component *comp)
    {
        if (const auto *pipe = dynamic_cast<const Pipe *>(comp))
        {
            const double d = pipe->getDiameter();
            if (d > 0.0)
            {
                return kPi * d * d / 4.0;
            }
        }
        if (const auto *channel = dynamic_cast<const Channel *>(comp))
        {
            return channel->getFlowArea();
        }
        if (const auto *loss = dynamic_cast<const LocalLossComponent *>(comp))
        {
            return loss->getFlowArea();
        }
        if (const auto *hex = dynamic_cast<const HeatExchanger *>(comp))
        {
            return hex->getFlowArea();
        }
        return 0.0;
    }

    void writePortFields(std::ostream &out,
                         const std::string &indent,
                         int portIndex,
                         const char *role,
                         int nodeId,
                         double pressure,
                         double temperature,
                         const FluidPropsView &fluid,
                         double flow,
                         double area)
    {
        const bool velocityValid = area > 0.0;
        const double velocity = velocityValid ? flow / area : 0.0;
        const double massFlow = fluid.density * flow;
        double mach = 0.0;
        if (fluid.compressible && temperature > 0.0)
        {
            constexpr double rGas = 287.0;
            double gamma = 1.4;
            if (fluid.specificHeat > rGas)
            {
                gamma = fluid.specificHeat / (fluid.specificHeat - rGas);
                if (gamma < 1.0)
                {
                    gamma = 1.4;
                }
            }
            const double soundSpeed = std::sqrt(gamma * rGas * temperature);
            if (soundSpeed > 1e-12)
            {
                mach = std::abs(velocity) / soundSpeed;
            }
        }
        const double enthalpy = fluid.specificHeat * temperature;

        out << indent << "\"portIndex\": " << portIndex << ",\n";
        out << indent << "\"role\": \"" << (role ? role : "") << "\",\n";
        out << indent << "\"nodeId\": " << nodeId << ",\n";
        out << indent << "\"pressure_Pa\": " << std::setprecision(12) << pressure << ",\n";
        out << indent << "\"temperature_K\": " << std::setprecision(8) << temperature << ",\n";
        out << indent << "\"density\": " << std::setprecision(8) << fluid.density << ",\n";
        out << indent << "\"dynamicViscosity\": " << std::setprecision(8)
            << fluid.dynamicViscosity << ",\n";
        out << indent << "\"kinematicViscosity\": " << std::setprecision(8)
            << fluid.kinematicViscosity << ",\n";
        out << indent << "\"specificHeat\": " << std::setprecision(8)
            << fluid.specificHeat << ",\n";
        out << indent << "\"thermalConductivity\": " << std::setprecision(8)
            << fluid.thermalConductivity << ",\n";
        out << indent << "\"flowRate\": " << std::setprecision(12) << flow << ",\n";
        out << indent << "\"volumetricFlow\": " << std::setprecision(12) << flow << ",\n";
        out << indent << "\"massFlowRate\": " << std::setprecision(12) << massFlow << ",\n";
        out << indent << "\"area\": " << std::setprecision(12) << area << ",\n";
        out << indent << "\"velocity\": " << std::setprecision(12) << velocity << ",\n";
        out << indent << "\"mach\": " << std::setprecision(8) << mach << ",\n";
        out << indent << "\"enthalpy\": " << std::setprecision(12) << enthalpy << ",\n";
        out << indent << "\"velocityValid\": " << (velocityValid ? "true" : "false") << "\n";
    }

    void writeComponentValues(std::ostream &out,
                              const std::string &indent,
                              const ComponentValueList &values)
    {
        if (values.empty())
        {
            out << indent << "\"available\": false\n";
            return;
        }

        for (std::size_t i = 0; i < values.values.size(); ++i)
        {
            const ComponentValue &entry = values.values[i];
            out << indent << "\"" << entry.key << "\": ";
            switch (entry.type)
            {
            case ComponentValueType::Boolean:
                out << (entry.boolValue ? "true" : "false");
                break;
            case ComponentValueType::String:
                out << "\"" << entry.stringValue << "\"";
                break;
            case ComponentValueType::Number:
            default:
                out << std::setprecision(12) << entry.numberValue;
                break;
            }
            out << (i + 1 < values.values.size() ? ",\n" : "\n");
        }
    }

    void writeComponentParameters(std::ostream &out,
                                  const std::string &indent,
                                  const Component *comp)
    {
        if (comp == nullptr)
        {
            out << indent << "\"available\": false\n";
            return;
        }

        ComponentValueList params;
        comp->appendInputParameters(params);
        writeComponentValues(out, indent, params);
    }

    void writeComponentResults(std::ostream &out,
                               const std::string &indent,
                               const Component *comp)
    {
        if (comp == nullptr)
        {
            out << indent << "\"available\": false\n";
            return;
        }

        ComponentValueList results;
        comp->appendResultParameters(results);
        writeComponentValues(out, indent, results);
    }

    void writeDebugIterations(std::ostream &out,
                              const std::string &indent,
                              const std::vector<IterationComponentSnapshot> &snapshots)
    {
        out << indent << "\"iterations\": [\n";
        for (std::size_t i = 0; i < snapshots.size(); ++i)
        {
            const IterationComponentSnapshot &snapshot = snapshots[i];
            out << indent << "  {\n";
            out << indent << "    \"iteration\": " << snapshot.iteration << ",\n";
            out << indent << "    \"components\": [\n";
            for (std::size_t j = 0; j < snapshot.components.size(); ++j)
            {
                const ComponentIterationResult &result = snapshot.components[j];
                out << indent << "      {\n";
                out << indent << "        \"component_index\": " << result.componentIndex << ",\n";
                out << indent << "        \"type\": \"" << result.type << "\",\n";
                out << indent << "        \"name\": \"" << result.name << "\",\n";
                out << indent << "        \"from_node\": " << result.fromNodeId << ",\n";
                out << indent << "        \"to_node\": " << result.toNodeId << ",\n";
                out << indent << "        \"from_port\": " << result.fromPortIndex << ",\n";
                out << indent << "        \"to_port\": " << result.toPortIndex << ",\n";
                out << indent << "        \"flow\": " << std::setprecision(12) << result.flow << ",\n";
                out << indent << "        \"pressureDrop\": "
                    << std::setprecision(12) << result.pressureDrop << ",\n";
                out << indent << "        \"resistance\": "
                    << std::setprecision(12) << result.resistance << ",\n";
                out << indent << "        \"heatTransfer\": "
                    << std::setprecision(12) << result.heatTransfer << "\n";
                out << indent << "      }" << (j + 1 < snapshot.components.size() ? "," : "")
                    << "\n";
            }
            out << indent << "    ]\n";
            out << indent << "  }" << (i + 1 < snapshots.size() ? "," : "") << "\n";
        }
        out << indent << "]\n";
    }

    bool getNodeStateFromSnapshot(const std::vector<std::unique_ptr<Node>> &nodes,
                                  const std::vector<double> &pressures,
                                  const std::vector<double> &temperatures,
                                  int nodeId,
                                  double &pressureOut,
                                  double &temperatureOut)
    {
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            const auto &node = nodes[i];
            if (node && node->id == nodeId)
            {
                if (i < pressures.size())
                {
                    pressureOut = pressures[i];
                }
                else
                {
                    pressureOut = node->pressure;
                }
                if (i < temperatures.size())
                {
                    temperatureOut = temperatures[i];
                }
                else
                {
                    temperatureOut = node->temperature;
                }
                return true;
            }
        }
        return false;
    }

    void writeNodesFromSnapshot(std::ostream &out,
                                const std::string &indent,
                                const std::vector<std::unique_ptr<Node>> &nodes,
                                const std::vector<double> &pressures,
                                const std::vector<double> &temperatures,
                                bool trailingComma)
    {
        out << indent << "\"nodes\": [\n";
        std::size_t total = 0;
        for (const auto &node : nodes)
        {
            if (node)
            {
                ++total;
            }
        }
        std::size_t written = 0;
        for (std::size_t i = 0; i < nodes.size(); ++i)
        {
            const Node *node = nodes[i].get();
            if (node == nullptr)
            {
                continue;
            }

            double pressure = node->pressure;
            double temperature = node->temperature;
            if (i < pressures.size())
            {
                pressure = pressures[i];
            }
            if (i < temperatures.size())
            {
                temperature = temperatures[i];
            }

            out << indent << "  {\n";
            out << indent << "    \"id\": " << node->id << ",\n";
            out << indent << "    \"name\": \"" << node->name << "\",\n";
            out << indent << "    \"pressure_Pa\": " << std::setprecision(12) << pressure << ",\n";
            out << indent << "    \"temperature_K\": " << std::setprecision(8) << temperature << ",\n";
            out << indent << "    \"flow_injection\": " << std::setprecision(8)
                << node->flowInjection << ",\n";
            out << indent << "    \"thermal_injection_W\": " << std::setprecision(8)
                << node->thermalInjectionPower << "\n";
            out << indent << "  }" << (++written < total ? "," : "") << "\n";
        }
        out << indent << "]" << (trailingComma ? "," : "") << "\n";
    }

    std::vector<std::string> splitByComma(const std::string &text)
    {
        std::vector<std::string> out;
        std::string item;
        std::istringstream ss(text);
        while (std::getline(ss, item, ','))
        {
            item = trim(item);
            if (!item.empty())
            {
                out.push_back(item);
            }
        }
        return out;
    }

    std::string readRemainder(std::istringstream &ss)
    {
        std::string tail;
        std::getline(ss, tail);
        return trim(tail);
    }

    std::string stripOptionalQuotes(const std::string &text)
    {
        std::string value = trim(text);
        if (value.size() >= 2 && value.front() == '"' && value.back() == '"')
        {
            value = value.substr(1, value.size() - 2);
        }
        return trim(value);
    }

    std::vector<std::string> readObjectKeys(const std::string &objectStr)
    {
        std::vector<std::string> keys;
        const std::size_t left = objectStr.find('{');
        const std::size_t right = objectStr.rfind('}');
        if (left == std::string::npos || right == std::string::npos || right <= left)
        {
            return keys;
        }

        std::size_t i = left + 1;
        int depth = 1;
        bool inString = false;
        while (i < right && depth > 0)
        {
            const char c = objectStr[i];
            if (c == '"' && (i == 0 || objectStr[i - 1] != '\\'))
            {
                if (!inString && depth == 1)
                {
                    const std::size_t keyStart = i + 1;
                    std::size_t keyEnd = keyStart;
                    while (keyEnd < right)
                    {
                        if (objectStr[keyEnd] == '"' && objectStr[keyEnd - 1] != '\\')
                        {
                            break;
                        }
                        ++keyEnd;
                    }
                    if (keyEnd < right)
                    {
                        std::size_t pos = keyEnd + 1;
                        while (pos < right &&
                               std::isspace(static_cast<unsigned char>(objectStr[pos])))
                        {
                            ++pos;
                        }
                        if (pos < right && objectStr[pos] == ':')
                        {
                            keys.push_back(objectStr.substr(keyStart, keyEnd - keyStart));
                        }
                        i = keyEnd;
                    }
                }
                inString = !inString;
                ++i;
                continue;
            }

            if (!inString)
            {
                if (c == '{' || c == '[')
                {
                    ++depth;
                }
                else if (c == '}' || c == ']')
                {
                    --depth;
                }
            }
            ++i;
        }
        return keys;
    }

    void pushUniqueLower(std::vector<std::string> &values, const std::string &rawValue)
    {
        if (rawValue.empty())
        {
            return;
        }

        const std::string value = toLowerCopy(trim(rawValue));
        if (value.empty())
        {
            return;
        }

        if (std::find(values.begin(), values.end(), value) == values.end())
        {
            values.push_back(value);
        }
    }

    void parsePhysicsList(const std::string &raw, std::vector<std::string> &outModules)
    {
        if (raw.empty())
        {
            return;
        }

        std::istringstream tokenStream(raw);
        std::string token;
        while (tokenStream >> token)
        {
            const std::vector<std::string> parts = splitByComma(token);
            if (parts.empty())
            {
                pushUniqueLower(outModules, token);
                continue;
            }
            for (const std::string &part : parts)
            {
                pushUniqueLower(outModules, part);
            }
        }
    }

    Node *ensureNode(FluidNetworkSolver &solver, int nodeId)
    {
        Node *node = solver.getNode(nodeId);
        if (node != nullptr)
        {
            return node;
        }

        std::ostringstream name;
        name << "Node" << nodeId;
        solver.addNode(std::unique_ptr<Node>(new Node(nodeId, name.str())));
        return solver.getNode(nodeId);
    }

    int addPipeComponent(FluidNetworkSolver &solver, const std::vector<double> &values)
    {
        const MaterialProperties &fluid = solver.getWorkingFluid();

        double length = 10.0;
        double diameter = 0.05;
        double roughness = 0.0001;
        double fixedResistance = -1.0;

        if (values.size() >= 3)
        {
            length = values[0];
            diameter = values[1];
            roughness = values[2];
        }
        else if (values.size() == 1)
        {
            fixedResistance = values[0];
        }

        std::ostringstream name;
        name << "Pipe_" << solver.getComponents().size();

        std::unique_ptr<Components::Pipe> pipe(
            new Components::Pipe(name.str(), length, diameter, roughness, fluid));
        if (fixedResistance > 0.0)
        {
            pipe->setFixedResistance(fixedResistance);
        }

        const int compIndex = static_cast<int>(solver.getComponents().size());
        solver.addComponent(std::move(pipe));
        return compIndex;
    }

    int addValveComponent(FluidNetworkSolver &solver, double kv, double openFraction)
    {
        const MaterialProperties &fluid = solver.getWorkingFluid();

        std::ostringstream name;
        name << "Valve_" << solver.getComponents().size();

        std::unique_ptr<Components::Valve> valve(
            new Components::Valve(name.str(), kv, openFraction, fluid));

        const int compIndex = static_cast<int>(solver.getComponents().size());
        solver.addComponent(std::move(valve));
        return compIndex;
    }

    int addHeatExchangerComponent(FluidNetworkSolver &solver, const std::vector<double> &values)
    {
        double ua = 1.0;
        double area = 0.01;
        double length = 1.0;
        double pitch = 10.0;

        if (!values.empty())
        {
            ua = values[0];
        }
        if (values.size() >= 2)
        {
            area = values[1];
        }
        if (values.size() >= 3)
        {
            length = values[2];
        }
        if (values.size() >= 4)
        {
            pitch = values[3];
        }

        std::ostringstream name;
        name << "HEX_" << solver.getComponents().size();

        std::unique_ptr<Components::HeatExchanger> hex(
            new Components::HeatExchanger(name.str(), ua, area, length, pitch));

        const int compIndex = static_cast<int>(solver.getComponents().size());
        solver.addComponent(std::move(hex));
        return compIndex;
    }

    void connectComponentPorts(FluidNetworkSolver &solver,
                               int compIndex,
                               const std::vector<int> &ports)
    {
        if (ports.size() < 2)
        {
            return;
        }

        const bool allowMultiple = ports.size() > 2;
        for (std::size_t i = 1; i < ports.size(); ++i)
        {
            solver.connectComponent(compIndex,
                                    ports[0],
                                    ports[i],
                                    0,
                                    static_cast<int>(i),
                                    allowMultiple);
        }
    }

    namespace JsonMini
    {

        std::string readValue(const std::string &content, const std::string &key)
        {
            std::size_t pos = content.find("\"" + key + "\"");
            if (pos == std::string::npos)
            {
                return "";
            }

            pos = content.find(':', pos);
            if (pos == std::string::npos)
            {
                return "";
            }

            ++pos;
            while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos])))
            {
                ++pos;
            }
            if (pos >= content.size())
            {
                return "";
            }

            if (content[pos] == '"')
            {
                ++pos;
                const std::size_t end = content.find('"', pos);
                if (end == std::string::npos)
                {
                    return "";
                }
                return content.substr(pos, end - pos);
            }

            if (content[pos] == '{' || content[pos] == '[')
            {
                const char open = content[pos];
                const char close = (open == '{') ? '}' : ']';
                int depth = 1;
                std::size_t end = pos + 1;
                bool inString = false;
                while (end < content.size() && depth > 0)
                {
                    const char c = content[end];
                    if (c == '"' && content[end - 1] != '\\')
                    {
                        inString = !inString;
                    }
                    if (!inString)
                    {
                        if (c == open)
                        {
                            ++depth;
                        }
                        else if (c == close)
                        {
                            --depth;
                        }
                    }
                    ++end;
                }
                return content.substr(pos, end - pos);
            }

            std::size_t end = pos;
            while (end < content.size() && content[end] != ',' && content[end] != '}' && content[end] != ']')
            {
                ++end;
            }
            return trim(content.substr(pos, end - pos));
        }

        std::vector<std::string> readArrayItems(const std::string &arrayStr)
        {
            std::vector<std::string> items;
            const std::size_t left = arrayStr.find('[');
            const std::size_t right = arrayStr.rfind(']');
            if (left == std::string::npos || right == std::string::npos || right <= left)
            {
                return items;
            }

            std::string body = arrayStr.substr(left + 1, right - left - 1);
            std::string item;
            int depth = 0;
            bool inString = false;

            for (std::size_t i = 0; i < body.size(); ++i)
            {
                const char c = body[i];
                if (c == '"' && (i == 0 || body[i - 1] != '\\'))
                {
                    inString = !inString;
                }

                if (!inString)
                {
                    if (c == '{' || c == '[')
                    {
                        ++depth;
                    }
                    else if (c == '}' || c == ']')
                    {
                        --depth;
                    }
                    else if (c == ',' && depth == 0)
                    {
                        const std::string t = trim(item);
                        if (!t.empty())
                        {
                            items.push_back(t);
                        }
                        item.clear();
                        continue;
                    }
                }
                item.push_back(c);
            }

            const std::string t = trim(item);
            if (!t.empty())
            {
                items.push_back(t);
            }
            return items;
        }

        std::vector<int> readIntArray(const std::string &arrayStr)
        {
            std::vector<int> values;
            for (const std::string &item : JsonMini::readArrayItems(arrayStr))
            {
                int v = 0;
                if (parseInt(stripOptionalQuotes(item), v))
                {
                    values.push_back(v);
                }
            }
            return values;
        }

    } // namespace JsonMini

} // namespace

namespace NetworkIO
{

    bool loadFromFile(const std::string &filePath, FluidNetworkSolver &solver, RunConfig *config)
    {
        const std::size_t dotPos = filePath.find_last_of('.');
        if (dotPos != std::string::npos)
        {
            const std::string ext = toLowerCopy(filePath.substr(dotPos));
            if (ext == ".json")
            {
                return loadFromJsonFile(filePath, solver, config);
            }
        }
        return loadFromTextFile(filePath, solver, config);
    }

    bool loadFromTextFile(const std::string &filePath, FluidNetworkSolver &solver, RunConfig *config)
    {
        std::ifstream ifs(filePath.c_str());
        if (!ifs.is_open())
        {
            std::cerr << "Cannot open input file: " << filePath << std::endl;
            return false;
        }

        solver.clearNetwork();

        RunConfig localConfig;
        RunConfig &cfg = config ? *config : localConfig;
        cfg = RunConfig();

        std::string line;
        int lineNumber = 0;
        while (std::getline(ifs, line))
        {
            ++lineNumber;

            const std::size_t commentPos = line.find('#');
            if (commentPos != std::string::npos)
            {
                line = line.substr(0, commentPos);
            }

            line = trim(line);
            if (line.empty())
            {
                continue;
            }

            std::istringstream ss(line);
            std::string cmd;
            ss >> cmd;
            cmd = toUpperCopy(cmd);

            if (cmd == "SOLVER")
            {
                std::string method;
                ss >> method;
                if (!method.empty())
                {
                    cfg.solverMethod = method;
                }
                double tol = 0.0;
                int maxIter = 0;
                double relax = 0.0;
                if (ss >> tol)
                {
                    cfg.tolerance = tol;
                    if (ss >> maxIter)
                    {
                        cfg.maxIterations = maxIter;
                        if (ss >> relax)
                        {
                            cfg.relaxationFactor = clampRelaxation(relax);
                        }
                    }
                }
                continue;
            }

            if (cmd == "ITERATIONS" || cmd == "MAX_ITER" || cmd == "MAX_ITERATIONS")
            {
                int maxIter = 0;
                if (ss >> maxIter && maxIter > 0)
                {
                    cfg.maxIterations = maxIter;
                }
                else
                {
                    std::cerr << "Invalid iteration count at line " << lineNumber << std::endl;
                }
                continue;
            }

            if (cmd == "TOLERANCE")
            {
                double tol = 0.0;
                if (ss >> tol && tol > 0.0)
                {
                    cfg.tolerance = tol;
                }
                else
                {
                    std::cerr << "Invalid tolerance at line " << lineNumber << std::endl;
                }
                continue;
            }

            if (cmd == "RESIDUAL" || cmd == "RESIDUAL_TOL" || cmd == "RESIDUAL_TOLERANCE")
            {
                double tol = 0.0;
                if (ss >> tol && tol > 0.0)
                {
                    cfg.tolerance = tol;
                }
                else
                {
                    std::cerr << "Invalid residual tolerance at line " << lineNumber << std::endl;
                }
                continue;
            }

            if (cmd == "RELAXATION" || cmd == "RELAX" || cmd == "OMEGA")
            {
                double relax = 0.0;
                if (ss >> relax)
                {
                    cfg.relaxationFactor = clampRelaxation(relax);
                }
                else
                {
                    std::cerr << "Invalid relaxation factor at line " << lineNumber << std::endl;
                }
                continue;
            }

            if (cmd == "THREADS" || cmd == "CPU_THREADS" || cmd == "OMP_THREADS")
            {
                int threads = 0;
                if (ss >> threads && threads > 0)
                {
                    cfg.cpuThreads = threads;
                }
                else
                {
                    std::cerr << "Invalid thread count at line " << lineNumber << std::endl;
                }
                continue;
            }

            if (cmd == "BACKEND" || cmd == "COMPUTE_BACKEND")
            {
                std::string backend;
                ss >> backend;
                if (!backend.empty())
                {
                    cfg.computeBackend = backend;
                }
                else
                {
                    std::cerr << "Invalid backend definition at line " << lineNumber << std::endl;
                }
                continue;
            }

            if (cmd == "COUPLING_ON" || cmd == "COUPLING_ENABLE")
            {
                cfg.coupling.enabled = true;
                continue;
            }

            if (cmd == "COUPLING_OFF" || cmd == "COUPLING_DISABLE")
            {
                cfg.coupling.enabled = false;
                continue;
            }

            if (cmd == "COUPLING_IO" || cmd == "COUPLING_FILES")
            {
                std::string inputFile;
                std::string outputFile;
                if (ss >> inputFile)
                {
                    cfg.coupling.inputFile = inputFile;
                }
                if (ss >> outputFile)
                {
                    cfg.coupling.outputFile = outputFile;
                }
                cfg.coupling.enabled = true;
                continue;
            }

            if (cmd == "DEBUG" || cmd == "TRACE")
            {
                std::string flag;
                if (!(ss >> flag))
                {
                    cfg.debugEnabled = true;
                }
                else
                {
                    bool enabled = false;
                    if (parseBool(flag, enabled))
                    {
                        cfg.debugEnabled = enabled;
                    }
                    else
                    {
                        std::cerr << "Invalid debug flag at line " << lineNumber << std::endl;
                    }
                }
                continue;
            }

            if (cmd == "STEADY" || cmd == "STEADY_STATE")
            {
                cfg.transientEnabled = false;
                continue;
            }

            if (cmd == "MODE" || cmd == "SIMULATION_TYPE")
            {
                std::string mode;
                ss >> mode;
                const std::string modeKey = toLowerCopy(mode);
                if (modeKey.find("trans") != std::string::npos)
                {
                    cfg.transientEnabled = true;
                }
                else if (!modeKey.empty())
                {
                    cfg.transientEnabled = false;
                }
                continue;
            }

            if (cmd == "TRANSIENT")
            {
                cfg.transientEnabled = true;
                double start = 0.0;
                double end = 0.0;
                double dt = 0.0;
                double interval = 0.0;
                if (ss >> start >> end >> dt >> interval)
                {
                    cfg.transientStartTime = start;
                    cfg.transientEndTime = end;
                    cfg.transientTimeStep = dt;
                    cfg.transientOutputInterval = interval;
                }
                continue;
            }

            if (cmd == "TIME_STEP")
            {
                double dt = 0.0;
                if (ss >> dt && dt > 0.0)
                {
                    cfg.transientTimeStep = dt;
                }
                continue;
            }

            if (cmd == "OUTPUT_INTERVAL")
            {
                double interval = 0.0;
                if (ss >> interval && interval > 0.0)
                {
                    cfg.transientOutputInterval = interval;
                }
                continue;
            }

            if (cmd == "DESCRIPTION")
            {
                cfg.description = readRemainder(ss);
                continue;
            }

            if (cmd == "SIMULATION" || cmd == "SIMULATION_NAME" || cmd == "CASE_NAME")
            {
                cfg.simulationName = readRemainder(ss);
                continue;
            }

            if (cmd == "DIMENSION")
            {
                std::string dim;
                ss >> dim;
                if (!dim.empty())
                {
                    cfg.simulationDimension = toUpperCopy(dim);
                }
                continue;
            }

            if (cmd == "PHYSICS")
            {
                cfg.physicsModules.clear();
                parsePhysicsList(readRemainder(ss), cfg.physicsModules);
                if (cfg.physicsModules.empty())
                {
                    cfg.physicsModules.push_back("fluid");
                }
                continue;
            }

            if (cmd == "MODULE")
            {
                std::string moduleName;
                ss >> moduleName;
                pushUniqueLower(cfg.physicsModules, moduleName);
                continue;
            }

            if (cmd == "COUPLING")
            {
                std::string mode;
                ss >> mode;
                if (!mode.empty())
                {
                    cfg.couplingMode = toLowerCopy(mode);
                }
                continue;
            }

            if (cmd == "MESH")
            {
                cfg.meshPrimaryFile = readRemainder(ss);
                continue;
            }

            if (cmd == "INTERFACE")
            {
                std::string name;
                ss >> name;
                if (!name.empty())
                {
                    pushUniqueLower(cfg.interfaceNames, name);
                    const std::string detail = readRemainder(ss);
                    if (!detail.empty())
                    {
                        cfg.extensionFields["interface." + toLowerCopy(name)] = detail;
                    }
                }
                continue;
            }

            if (cmd == "EXT")
            {
                std::string key;
                ss >> key;
                if (!key.empty())
                {
                    cfg.extensionFields[key] = readRemainder(ss);
                }
                continue;
            }

            if (cmd == "NODES")
            {
                continue;
            }

            if (cmd == "NODE")
            {
                int id = -1;
                ss >> id;
                if (id < 0)
                {
                    std::cerr << "Invalid NODE definition at line " << lineNumber << std::endl;
                    continue;
                }
                Node *node = ensureNode(solver, id);
                if (node == nullptr)
                {
                    continue;
                }

                std::string token;
                while (ss >> token)
                {
                    const std::string key = toUpperCopy(token);
                    if (key == "NAME")
                    {
                        ss >> node->name;
                    }
                    else if (key == "PRESSURE")
                    {
                        ss >> node->pressure;
                    }
                    else if (key == "TEMP" || key == "TEMPERATURE")
                    {
                        ss >> node->temperature;
                    }
                    else if (key == "FIXED")
                    {
                        ss >> node->fixedPressure;
                        node->pressure = node->fixedPressure;
                        node->isPressureFixed = true;
                        solver.addBoundaryCondition(BoundaryCondition(id, BoundaryConditionType::Pressure,
                                                                      node->fixedPressure));
                    }
                    else if (key == "INJ" || key == "INJECTION")
                    {
                        ss >> node->flowInjection;
                    }
                    else if (key == "INJT" || key == "THERMAL")
                    {
                        ss >> node->thermalInjectionPower;
                    }
                }
                continue;
            }

            if (cmd == "BC_PRESSURE" || cmd == "BCP")
            {
                int id = -1;
                double value = 0.0;
                ss >> id >> value;
                ensureNode(solver, id);
                solver.addBoundaryCondition(BoundaryCondition(id, BoundaryConditionType::Pressure, value));
                Node *node = solver.getNode(id);
                if (node != nullptr)
                {
                    node->isPressureFixed = true;
                    node->fixedPressure = value;
                    node->pressure = value;
                }
                continue;
            }

            if (cmd == "BC_TEMPERATURE" || cmd == "BCT")
            {
                int id = -1;
                double value = 0.0;
                ss >> id >> value;
                ensureNode(solver, id);
                solver.addBoundaryCondition(BoundaryCondition(id, BoundaryConditionType::Temperature, value));
                continue;
            }

            if (cmd == "BC_FLOW" || cmd == "BCF")
            {
                int id = -1;
                double value = 0.0;
                ss >> id >> value;
                ensureNode(solver, id);
                solver.addBoundaryCondition(BoundaryCondition(id, BoundaryConditionType::FlowRate, value));
                Node *node = solver.getNode(id);
                if (node != nullptr)
                {
                    node->flowInjection = value;
                }
                continue;
            }

            if (cmd == "PIPE")
            {
                int from = -1;
                int to = -1;
                ss >> from >> to;
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid PIPE definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);

                std::vector<double> vals;
                double v = 0.0;
                while (ss >> v)
                {
                    vals.push_back(v);
                }
                if (vals.empty())
                {
                    vals.push_back(10.0);
                    vals.push_back(0.05);
                    vals.push_back(0.0001);
                }
                const int compIndex = addPipeComponent(solver, vals);
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "VALVE")
            {
                int from = -1;
                int to = -1;
                double kv = 1.0;
                double open = 1.0;
                ss >> from >> to >> kv;
                if (ss.good())
                {
                    ss >> open;
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid VALVE definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const int compIndex = addValveComponent(solver, kv, open);
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "HEX" || cmd == "HEATX" || cmd == "HEAT_EXCHANGER")
            {
                int from = -1;
                int to = -1;
                ss >> from >> to;
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid HEX definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);

                std::vector<double> vals;
                double v = 0.0;
                while (ss >> v)
                {
                    vals.push_back(v);
                }
                if (vals.empty())
                {
                    vals.push_back(1.0);
                }
                const int compIndex = addHeatExchangerComponent(solver, vals);
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "RADIATOR")
            {
                int from = -1;
                int to = -1;
                double ua = 1.0;
                double area = 0.01;
                double length = 1.0;
                double pitch = 10.0;
                ss >> from >> to >> ua;
                if (ss.good())
                {
                    ss >> area >> length >> pitch;
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid RADIATOR definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                std::ostringstream name;
                name << "Radiator_" << solver.getComponents().size();
                std::unique_ptr<Components::Radiator> rad(
                    new Components::Radiator(name.str(), ua, area, length, pitch));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(rad));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "BEND")
            {
                int from = -1;
                int to = -1;
                double k = 0.5;
                double dia = 0.05;
                double angle = 90.0;
                double direction = 0.0;
                ss >> from >> to >> k >> dia;
                if (ss.good())
                {
                    ss >> angle;
                    if (ss.good())
                    {
                        ss >> direction;
                    }
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid BEND definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "Bend_" << solver.getComponents().size();
                std::unique_ptr<Components::Bend> bend(
                    new Components::Bend(name.str(), k, dia, fluid, angle, direction));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(bend));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "TJOINT" || cmd == "T_JOINT")
            {
                int n0 = -1;
                int n1 = -1;
                int n2 = -1;
                double k = 1.0;
                double dia = 0.05;
                double branchLoss = 1.0;
                ss >> n0 >> n1 >> n2 >> k >> dia;
                if (ss.good())
                {
                    ss >> branchLoss;
                }
                if (n0 < 0 || n1 < 0 || n2 < 0)
                {
                    std::cerr << "Invalid TJOINT definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, n0);
                ensureNode(solver, n1);
                ensureNode(solver, n2);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "TJoint_" << solver.getComponents().size();
                std::unique_ptr<Components::TJoint> joint(
                    new Components::TJoint(name.str(), k, dia, fluid, branchLoss));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(joint));
                connectComponentPorts(solver, compIndex, {n0, n1, n2});
                continue;
            }

            if (cmd == "YJOINT" || cmd == "Y_JOINT")
            {
                int n0 = -1;
                int n1 = -1;
                int n2 = -1;
                double k = 0.7;
                double dia = 0.05;
                double branchLoss = 0.7;
                ss >> n0 >> n1 >> n2 >> k >> dia;
                if (ss.good())
                {
                    ss >> branchLoss;
                }
                if (n0 < 0 || n1 < 0 || n2 < 0)
                {
                    std::cerr << "Invalid YJOINT definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, n0);
                ensureNode(solver, n1);
                ensureNode(solver, n2);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "YJoint_" << solver.getComponents().size();
                std::unique_ptr<Components::YJoint> joint(
                    new Components::YJoint(name.str(), k, dia, fluid, branchLoss));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(joint));
                connectComponentPorts(solver, compIndex, {n0, n1, n2});
                continue;
            }

            if (cmd == "CHANNEL")
            {
                int from = -1;
                int to = -1;
                double length = 1.0;
                double width = 0.01;
                double height = 0.01;
                double roughness = 0.0;
                ss >> from >> to >> length >> width >> height;
                if (ss.good())
                {
                    ss >> roughness;
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid CHANNEL definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "Channel_" << solver.getComponents().size();
                std::unique_ptr<Components::Channel> channel(
                    new Components::Channel(name.str(), length, width, height, roughness, fluid));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(channel));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "LIQUID_FILTER" || cmd == "GAS_FILTER" || cmd == "MESH_FILTER" ||
                cmd == "PAPER_FILTER")
            {
                int from = -1;
                int to = -1;
                double k = 1.0;
                double dia = 0.05;
                double fouling = 1.0;
                ss >> from >> to >> k >> dia;
                if (ss.good())
                {
                    ss >> fouling;
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid FILTER definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << cmd << "_" << solver.getComponents().size();
                std::unique_ptr<Component> filter;
                if (cmd == "LIQUID_FILTER")
                {
                    filter.reset(new Components::LiquidFilter(name.str(), k, dia, fluid));
                }
                else if (cmd == "GAS_FILTER")
                {
                    filter.reset(new Components::GasFilter(name.str(), k, dia, fluid));
                }
                else if (cmd == "MESH_FILTER")
                {
                    filter.reset(new Components::MeshFilter(name.str(), k, dia, fluid));
                }
                else
                {
                    filter.reset(new Components::PaperFilter(name.str(), k, dia, fluid));
                }
                if (auto *base = dynamic_cast<FilterBase *>(filter.get()))
                {
                    base->setFoulingFactor(fouling);
                }
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(filter));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "EXPANDER" || cmd == "CONTRACTION")
            {
                int from = -1;
                int to = -1;
                double inDia = 0.05;
                double outDia = 0.1;
                double k = -1.0;
                ss >> from >> to >> inDia >> outDia;
                if (ss.good())
                {
                    ss >> k;
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid " << cmd << " definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << cmd << "_" << solver.getComponents().size();
                std::unique_ptr<Component> comp;
                if (cmd == "EXPANDER")
                {
                    comp.reset(new Components::Expander(name.str(), inDia, outDia, fluid, k));
                }
                else
                {
                    comp.reset(new Components::Contraction(name.str(), inDia, outDia, fluid, k));
                }
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(comp));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "PRESSURE_VESSEL" || cmd == "LEVEL_TANK")
            {
                int from = -1;
                int to = -1;
                double vol = 1.0;
                double dia = 0.5;
                double k = 0.2;
                ss >> from >> to >> vol >> dia;
                if (ss.good())
                {
                    ss >> k;
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid " << cmd << " definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << cmd << "_" << solver.getComponents().size();
                std::unique_ptr<Component> comp;
                if (cmd == "PRESSURE_VESSEL")
                {
                    comp.reset(new Components::PressureVessel(name.str(), vol, dia, fluid, k));
                }
                else
                {
                    comp.reset(new Components::LevelTank(name.str(), vol, dia, fluid, k));
                }
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(comp));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "HYDRAULIC_ACCUMULATOR" || cmd == "PNEUMATIC_ACCUMULATOR")
            {
                int from = -1;
                int to = -1;
                double vol = 1.0;
                double dia = 0.5;
                double precharge = 1e5;
                double gasVol = 0.1;
                double k = 0.2;
                ss >> from >> to >> vol >> dia >> precharge >> gasVol;
                if (ss.good())
                {
                    ss >> k;
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid " << cmd << " definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << cmd << "_" << solver.getComponents().size();
                std::unique_ptr<Component> comp;
                if (cmd == "HYDRAULIC_ACCUMULATOR")
                {
                    comp.reset(new Components::HydraulicAccumulator(name.str(),
                                                                    vol,
                                                                    dia,
                                                                    fluid,
                                                                    precharge,
                                                                    gasVol,
                                                                    k));
                }
                else
                {
                    comp.reset(new Components::PneumaticAccumulator(name.str(),
                                                                    vol,
                                                                    dia,
                                                                    fluid,
                                                                    precharge,
                                                                    gasVol,
                                                                    k));
                }
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(comp));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "SEAL")
            {
                int from = -1;
                int to = -1;
                double k = 1.0;
                double dia = 0.05;
                ss >> from >> to >> k >> dia;
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid SEAL definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "Seal_" << solver.getComponents().size();
                std::unique_ptr<Components::Seal> seal(
                    new Components::Seal(name.str(), k, dia, fluid));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(seal));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "OIL_WATER_SEPARATOR")
            {
                int from = -1;
                int to = -1;
                double k = 1.0;
                double dia = 0.05;
                double efficiency = 0.9;
                ss >> from >> to >> k >> dia;
                if (ss.good())
                {
                    ss >> efficiency;
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid OIL_WATER_SEPARATOR definition at line "
                              << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "OilWaterSeparator_" << solver.getComponents().size();
                std::unique_ptr<Components::OilWaterSeparator> sep(
                    new Components::OilWaterSeparator(name.str(), k, dia, fluid, efficiency));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(sep));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "COUPLING_LINK" || cmd == "COUPLING_FLOW" || cmd == "COUPLING_THERMAL")
            {
                int from = -1;
                int to = -1;
                std::string id;
                double resistance = 1.0;
                double pressureDrop = 0.0;
                double heatTransfer = 0.0;
                ss >> from >> to >> id;
                if (ss.good())
                {
                    ss >> resistance;
                    if (ss.good())
                    {
                        ss >> pressureDrop;
                        if (ss.good())
                        {
                            ss >> heatTransfer;
                        }
                    }
                }
                if (from < 0 || to < 0 || id.empty())
                {
                    std::cerr << "Invalid COUPLING definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                std::ostringstream name;
                name << "Coupling_" << solver.getComponents().size();
                std::unique_ptr<Component> comp;
                if (cmd == "COUPLING_FLOW")
                {
                    comp.reset(new Components::CouplingFlow(name.str(), id, resistance, pressureDrop));
                }
                else if (cmd == "COUPLING_THERMAL")
                {
                    comp.reset(new Components::CouplingThermal(name.str(),
                                                               id,
                                                               resistance,
                                                               pressureDrop,
                                                               heatTransfer));
                }
                else
                {
                    comp.reset(new Components::CouplingLink(name.str(),
                                                            "CouplingLink",
                                                            id,
                                                            resistance,
                                                            pressureDrop,
                                                            heatTransfer,
                                                            true));
                }
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(comp));
                solver.connectComponent(compIndex, from, to);
                cfg.coupling.enabled = true;
                continue;
            }

            if (cmd == "GATE_VALVE" || cmd == "BALL_VALVE" || cmd == "BUTTERFLY_VALVE")
            {
                int from = -1;
                int to = -1;
                double cv = 1.0;
                double open = 1.0;
                ss >> from >> to >> cv;
                if (ss.good())
                {
                    ss >> open;
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid " << cmd << " definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << cmd << "_" << solver.getComponents().size();
                std::unique_ptr<Component> comp;
                if (cmd == "GATE_VALVE")
                {
                    comp.reset(new Components::GateValve(name.str(), cv, open, fluid));
                }
                else if (cmd == "BALL_VALVE")
                {
                    comp.reset(new Components::BallValve(name.str(), cv, open, fluid));
                }
                else
                {
                    comp.reset(new Components::ButterflyValve(name.str(), cv, open, fluid));
                }
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(comp));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "CONTROL_VALVE" || cmd == "FLOW_CONTROL_VALVE")
            {
                int from = -1;
                int to = -1;
                double cv = 1.0;
                double open = 1.0;
                double value = 0.0;
                bool hasValue = false;
                ss >> from >> to >> cv;
                if (ss >> open)
                {
                    if (ss >> value)
                    {
                        hasValue = true;
                    }
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid " << cmd << " definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << cmd << "_" << solver.getComponents().size();
                std::unique_ptr<Component> comp;
                if (cmd == "FLOW_CONTROL_VALVE")
                {
                    auto *fcv = new Components::FlowControlValve(name.str(),
                                                                 cv,
                                                                 open,
                                                                 fluid,
                                                                 hasValue ? value : 0.0);
                    comp.reset(fcv);
                }
                else
                {
                    auto *cvv = new Components::ControlValve(name.str(), cv, open, fluid);
                    if (hasValue)
                    {
                        cvv->setControlSignal(value);
                    }
                    comp.reset(cvv);
                }
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(comp));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "ISOLATION_VALVE")
            {
                int from = -1;
                int to = -1;
                double cv = 1.0;
                int open = 1;
                ss >> from >> to >> cv >> open;
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid ISOLATION_VALVE definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "IsolationValve_" << solver.getComponents().size();
                std::unique_ptr<Components::IsolationValve> valve(
                    new Components::IsolationValve(name.str(), cv, open != 0, fluid));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(valve));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "CHECK_VALVE")
            {
                int from = -1;
                int to = -1;
                double cv = 1.0;
                double open = 1.0;
                double direction = 1.0;
                ss >> from >> to >> cv;
                if (ss.good())
                {
                    ss >> open;
                    if (ss.good())
                    {
                        ss >> direction;
                    }
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid CHECK_VALVE definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "CheckValve_" << solver.getComponents().size();
                std::unique_ptr<Components::CheckValve> valve(
                    new Components::CheckValve(name.str(), cv, open, fluid, direction));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(valve));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "PRESSURE_RELIEF_VALVE")
            {
                int from = -1;
                int to = -1;
                double cv = 1.0;
                double setP = 1e5;
                double open = 1.0;
                double band = 0.0;
                ss >> from >> to >> cv >> setP;
                if (ss.good())
                {
                    ss >> open;
                    if (ss.good())
                    {
                        ss >> band;
                    }
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid PRESSURE_RELIEF_VALVE definition at line "
                              << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "PressureReliefValve_" << solver.getComponents().size();
                std::unique_ptr<Components::PressureReliefValve> valve(
                    new Components::PressureReliefValve(name.str(), cv, open, fluid, setP, band));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(valve));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "GAS_REGULATOR")
            {
                int from = -1;
                int to = -1;
                double cv = 1.0;
                double setP = 1e5;
                double open = 1.0;
                double band = 0.0;
                ss >> from >> to >> cv >> setP;
                if (ss.good())
                {
                    ss >> open;
                    if (ss.good())
                    {
                        ss >> band;
                    }
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid GAS_REGULATOR definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << "GasRegulator_" << solver.getComponents().size();
                std::unique_ptr<Components::GasRegulator> valve(
                    new Components::GasRegulator(name.str(), cv, open, fluid, setP, band));
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(valve));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            if (cmd == "CENTRIFUGAL_PUMP" || cmd == "PISTON_PUMP" ||
                cmd == "GEAR_PUMP" || cmd == "SCREW_PUMP")
            {
                int from = -1;
                int to = -1;
                double a = 0.0;
                double b = 0.0;
                double c = 0.0;
                int allowReverse = 0;
                ss >> from >> to >> a;
                if (ss.good())
                {
                    ss >> b;
                    if (ss.good())
                    {
                        ss >> c;
                        if (ss.good())
                        {
                            ss >> allowReverse;
                        }
                    }
                }
                if (from < 0 || to < 0)
                {
                    std::cerr << "Invalid " << cmd << " definition at line " << lineNumber << std::endl;
                    continue;
                }
                ensureNode(solver, from);
                ensureNode(solver, to);
                const MaterialProperties &fluid = solver.getWorkingFluid();
                std::ostringstream name;
                name << cmd << "_" << solver.getComponents().size();
                std::unique_ptr<Component> comp;
                if (cmd == "CENTRIFUGAL_PUMP")
                {
                    comp.reset(new Components::CentrifugalPump(name.str(),
                                                               a,
                                                               b,
                                                               c,
                                                               fluid,
                                                               allowReverse != 0));
                }
                else if (cmd == "PISTON_PUMP")
                {
                    comp.reset(new Components::PistonPump(name.str(), a, fluid, allowReverse != 0));
                }
                else if (cmd == "GEAR_PUMP")
                {
                    comp.reset(new Components::GearPump(name.str(), a, fluid, allowReverse != 0));
                }
                else
                {
                    comp.reset(new Components::ScrewPump(name.str(), a, fluid, allowReverse != 0));
                }
                const int compIndex = static_cast<int>(solver.getComponents().size());
                solver.addComponent(std::move(comp));
                solver.connectComponent(compIndex, from, to);
                continue;
            }

            std::cerr << "Warning: unknown command at line " << lineNumber << ": " << cmd << std::endl;
        }

        solver.setCalculationParameters(cfg.transientTimeStep,
                                        cfg.tolerance,
                                        cfg.maxIterations,
                                        cfg.relaxationFactor);
        solver.setConvergenceParameters(cfg.tolerance,
                                        cfg.relativeConvergenceTolerance,
                                        cfg.solutionIncrementTolerance);
        solver.setSteadySolverTypeByName(cfg.solverMethod);
        solver.setDebugEnabled(cfg.debugEnabled);
        solver.setCouplingConfig(cfg.coupling);
        solver.setCpuThreads(cfg.cpuThreads);
        if (!cfg.computeBackend.empty())
        {
            if (!solver.setComputeBackendByName(cfg.computeBackend))
            {
                std::cerr << "Warning: unknown backend \"" << cfg.computeBackend << "\"." << std::endl;
            }
        }

        return true;
    }

    bool loadFromJsonFile(const std::string &filePath, FluidNetworkSolver &solver, RunConfig *config)
    {
        std::ifstream ifs(filePath.c_str());
        if (!ifs.is_open())
        {
            std::cerr << "Cannot open input file: " << filePath << std::endl;
            return false;
        }

        std::stringstream buffer;
        buffer << ifs.rdbuf();
        const std::string content = buffer.str();

        // Validate JSON schema before processing
        JsonSchemaValidator validator;
        std::vector<ValidationError> validationErrors;
        if (!validator.validate(content, validationErrors))
        {
            std::cerr << "JSON Schema validation failed:\n";
            std::cerr << formatValidationErrors(validationErrors) << std::endl;
            // Continue anyway - warnings only, not blocking
        }

        solver.clearNetwork();

        RunConfig localConfig;
        RunConfig &cfg = config ? *config : localConfig;
        cfg = RunConfig();

        cfg.description = JsonMini::readValue(content, "description");
        cfg.simulationName = JsonMini::readValue(content, "simulationName");

        const std::string simulationObj = JsonMini::readValue(content, "simulation");
        if (!simulationObj.empty())
        {
            const std::string simName = JsonMini::readValue(simulationObj, "name");
            if (!simName.empty())
            {
                cfg.simulationName = simName;
            }

            const std::string simDescription = JsonMini::readValue(simulationObj, "description");
            if (!simDescription.empty())
            {
                cfg.description = simDescription;
            }

            std::string dim = JsonMini::readValue(simulationObj, "dimension");
            dim = stripOptionalQuotes(dim);
            if (!dim.empty())
            {
                dim = toUpperCopy(dim);
                if (dim == "1" || dim == "2" || dim == "3")
                {
                    dim += "D";
                }
                cfg.simulationDimension = dim;
            }

            const std::string coupling = JsonMini::readValue(simulationObj, "coupling");
            if (!coupling.empty())
            {
                cfg.couplingMode = toLowerCopy(stripOptionalQuotes(coupling));
            }

            std::string meshFile = JsonMini::readValue(simulationObj, "meshFile");
            if (meshFile.empty())
            {
                const std::string meshObj = JsonMini::readValue(simulationObj, "mesh");
                if (!meshObj.empty())
                {
                    meshFile = JsonMini::readValue(meshObj, "primary");
                    if (meshFile.empty())
                    {
                        meshFile = JsonMini::readValue(meshObj, "fluid");
                    }
                    if (meshFile.empty())
                    {
                        meshFile = JsonMini::readValue(meshObj, "main");
                    }
                }
            }
            if (!meshFile.empty())
            {
                cfg.meshPrimaryFile = stripOptionalQuotes(meshFile);
            }

            std::string physicsArr = JsonMini::readValue(simulationObj, "physicsModules");
            if (physicsArr.empty())
            {
                physicsArr = JsonMini::readValue(simulationObj, "physics");
            }
            if (!physicsArr.empty())
            {
                cfg.physicsModules.clear();
                for (const std::string &item : JsonMini::readArrayItems(physicsArr))
                {
                    parsePhysicsList(stripOptionalQuotes(item), cfg.physicsModules);
                }
                if (cfg.physicsModules.empty())
                {
                    cfg.physicsModules.push_back("fluid");
                }
            }

            const std::string interfacesObj = JsonMini::readValue(simulationObj, "interfaces");
            if (!interfacesObj.empty())
            {
                for (const std::string &interfaceName : readObjectKeys(interfacesObj))
                {
                    pushUniqueLower(cfg.interfaceNames, interfaceName);
                    const std::string payload = JsonMini::readValue(interfacesObj, interfaceName);
                    if (!payload.empty())
                    {
                        cfg.extensionFields["interface." + toLowerCopy(interfaceName)] = payload;
                    }
                }
            }

            const std::string extensionsObj = JsonMini::readValue(simulationObj, "extensions");
            if (!extensionsObj.empty())
            {
                for (const std::string &key : readObjectKeys(extensionsObj))
                {
                    cfg.extensionFields[key] =
                        stripOptionalQuotes(JsonMini::readValue(extensionsObj, key));
                }
            }
        }

        const std::string solverObj = JsonMini::readValue(content, "solver");
        if (!solverObj.empty())
        {
            const std::string method = JsonMini::readValue(solverObj, "method");
            if (!method.empty())
            {
                cfg.solverMethod = method;
            }
            const std::string backend = JsonMini::readValue(solverObj, "backend");
            if (!backend.empty())
            {
                cfg.computeBackend = stripOptionalQuotes(backend);
            }
            int threads = 0;
            if (parseInt(JsonMini::readValue(solverObj, "threads"), threads) && threads > 0)
            {
                cfg.cpuThreads = threads;
            }
            if (parseInt(JsonMini::readValue(solverObj, "cpuThreads"), threads) && threads > 0)
            {
                cfg.cpuThreads = threads;
            }
            if (parseInt(JsonMini::readValue(solverObj, "openmpThreads"), threads) && threads > 0)
            {
                cfg.cpuThreads = threads;
            }
            const std::string tol = JsonMini::readValue(solverObj, "tolerance");
            const std::string resTol = JsonMini::readValue(solverObj, "residualTolerance");
            const std::string maxIter = JsonMini::readValue(solverObj, "maxIterations");
            const std::string relax = JsonMini::readValue(solverObj, "relaxation");
            const std::string relaxFactor = JsonMini::readValue(solverObj, "relaxationFactor");
            const std::string underRelax = JsonMini::readValue(solverObj, "underRelaxation");
            const std::string omega = JsonMini::readValue(solverObj, "omega");
            const std::string relTol = JsonMini::readValue(solverObj, "relativeConvergenceTolerance");
            const std::string dxTol = JsonMini::readValue(solverObj, "solutionIncrementTolerance");

            if (!tol.empty())
            {
                parseDouble(tol, cfg.tolerance);
            }
            if (!resTol.empty())
            {
                parseDouble(resTol, cfg.tolerance);
            }
            if (!maxIter.empty())
            {
                parseInt(maxIter, cfg.maxIterations);
            }
            if (!relax.empty())
            {
                double value = 0.0;
                if (parseDouble(relax, value))
                {
                    cfg.relaxationFactor = clampRelaxation(value);
                }
            }
            if (!relaxFactor.empty())
            {
                double value = 0.0;
                if (parseDouble(relaxFactor, value))
                {
                    cfg.relaxationFactor = clampRelaxation(value);
                }
            }
            if (!underRelax.empty())
            {
                double value = 0.0;
                if (parseDouble(underRelax, value))
                {
                    cfg.relaxationFactor = clampRelaxation(value);
                }
            }
            if (!omega.empty())
            {
                double value = 0.0;
                if (parseDouble(omega, value))
                {
                    cfg.relaxationFactor = clampRelaxation(value);
                }
            }
            // Parse enhanced convergence parameters
            if (!relTol.empty())
            {
                parseDouble(relTol, cfg.relativeConvergenceTolerance);
            }
            if (!dxTol.empty())
            {
                parseDouble(dxTol, cfg.solutionIncrementTolerance);
            }

            bool debugFlag = false;
            if (parseBool(JsonMini::readValue(solverObj, "debug"), debugFlag) ||
                parseBool(JsonMini::readValue(solverObj, "debugEnabled"), debugFlag))
            {
                cfg.debugEnabled = debugFlag;
            }
        }

        const std::string runControlObj = JsonMini::readValue(content, "runControl");
        if (!runControlObj.empty())
        {
    const std::string method = JsonMini::readValue(runControlObj, "solverMethod");
    if (!method.empty())
    {
        cfg.solverMethod = method;
    }
    const std::string backend = JsonMini::readValue(runControlObj, "backend");
    const std::string computeBackend = JsonMini::readValue(runControlObj, "computeBackend");
    if (!computeBackend.empty())
    {
        cfg.computeBackend = stripOptionalQuotes(computeBackend);
    }
    else if (!backend.empty())
    {
        cfg.computeBackend = stripOptionalQuotes(backend);
    }

    int threads = 0;
    if (parseInt(JsonMini::readValue(runControlObj, "threads"), threads) && threads > 0)
    {
        cfg.cpuThreads = threads;
    }
    if (parseInt(JsonMini::readValue(runControlObj, "cpuThreads"), threads) && threads > 0)
    {
        cfg.cpuThreads = threads;
    }
    if (parseInt(JsonMini::readValue(runControlObj, "openmpThreads"), threads) && threads > 0)
    {
        cfg.cpuThreads = threads;
    }

    const std::string tol = JsonMini::readValue(runControlObj, "tolerance");
    const std::string resTol = JsonMini::readValue(runControlObj, "residualTolerance");
    if (!tol.empty())
    {
        parseDouble(tol, cfg.tolerance);
    }
    if (!resTol.empty())
    {
        parseDouble(resTol, cfg.tolerance);
    }

    int maxIter = 0;
    if (parseInt(JsonMini::readValue(runControlObj, "iterations"), maxIter) &&
        maxIter > 0)
    {
        cfg.maxIterations = maxIter;
    }
    if (parseInt(JsonMini::readValue(runControlObj, "maxIterations"), maxIter) &&
        maxIter > 0)
    {
        cfg.maxIterations = maxIter;
    }

    bool transientEnabled = false;
    if (parseBool(JsonMini::readValue(runControlObj, "transientEnabled"),
                  transientEnabled))
    {
        cfg.transientEnabled = transientEnabled;
    }

    const std::string mode = JsonMini::readValue(runControlObj, "mode");
    if (!mode.empty())
    {
        const std::string modeKey = toLowerCopy(stripOptionalQuotes(mode));
        if (modeKey.find("trans") != std::string::npos)
        {
            cfg.transientEnabled = true;
        }
        else if (!modeKey.empty())
        {
            cfg.transientEnabled = false;
        }
    }

    bool steadyFlag = false;
    if (parseBool(JsonMini::readValue(runControlObj, "steady"), steadyFlag) ||
        parseBool(JsonMini::readValue(runControlObj, "steadyState"), steadyFlag))
    {
        if (steadyFlag)
        {
            cfg.transientEnabled = false;
        }
    }

    double relaxValue = 0.0;
    if (parseDouble(JsonMini::readValue(runControlObj, "relaxation"), relaxValue) ||
        parseDouble(JsonMini::readValue(runControlObj, "relaxationFactor"), relaxValue) ||
        parseDouble(JsonMini::readValue(runControlObj, "underRelaxation"), relaxValue) ||
        parseDouble(JsonMini::readValue(runControlObj, "omega"), relaxValue))
    {
        cfg.relaxationFactor = clampRelaxation(relaxValue);
    }

    parseDouble(JsonMini::readValue(runControlObj, "timeStep"), cfg.transientTimeStep);
    parseDouble(JsonMini::readValue(runControlObj, "outputInterval"),
                cfg.transientOutputInterval);

    bool debugFlag = false;
    if (parseBool(JsonMini::readValue(runControlObj, "debug"), debugFlag) ||
        parseBool(JsonMini::readValue(runControlObj, "debugEnabled"), debugFlag))
    {
        cfg.debugEnabled = debugFlag;
    }

    bool couplingFlag = false;
    if (parseBool(JsonMini::readValue(runControlObj, "couplingEnabled"), couplingFlag) ||
        parseBool(JsonMini::readValue(runControlObj, "enableCoupling"), couplingFlag))
    {
        cfg.coupling.enabled = couplingFlag;
    }
    const std::string couplingInput = JsonMini::readValue(runControlObj, "couplingInput");
    if (!couplingInput.empty())
    {
        cfg.coupling.inputFile = stripOptionalQuotes(couplingInput);
        cfg.coupling.enabled = true;
    }
    const std::string couplingOutput = JsonMini::readValue(runControlObj, "couplingOutput");
    if (!couplingOutput.empty())
    {
        cfg.coupling.outputFile = stripOptionalQuotes(couplingOutput);
        cfg.coupling.enabled = true;
    }
}

const std::string transientObj = JsonMini::readValue(content, "transient");
if (!transientObj.empty())
{
    bool transientEnabled = false;
    if (parseBool(JsonMini::readValue(transientObj, "enabled"), transientEnabled))
    {
        cfg.transientEnabled = transientEnabled;
    }

    parseDouble(JsonMini::readValue(transientObj, "startTime"), cfg.transientStartTime);
    parseDouble(JsonMini::readValue(transientObj, "endTime"), cfg.transientEndTime);
    parseDouble(JsonMini::readValue(transientObj, "timeStep"), cfg.transientTimeStep);
    parseDouble(JsonMini::readValue(transientObj, "outputInterval"),
                cfg.transientOutputInterval);
}

const std::string couplingObj = JsonMini::readValue(content, "coupling");
if (!couplingObj.empty())
{
    bool couplingEnabled = false;
    if (parseBool(JsonMini::readValue(couplingObj, "enabled"), couplingEnabled) ||
        parseBool(JsonMini::readValue(couplingObj, "enable"), couplingEnabled))
    {
        cfg.coupling.enabled = couplingEnabled;
    }

    const std::string inputFile = JsonMini::readValue(couplingObj, "inputFile");
    if (!inputFile.empty())
    {
        cfg.coupling.inputFile = stripOptionalQuotes(inputFile);
        cfg.coupling.enabled = true;
    }
    const std::string outputFile = JsonMini::readValue(couplingObj, "outputFile");
    if (!outputFile.empty())
    {
        cfg.coupling.outputFile = stripOptionalQuotes(outputFile);
        cfg.coupling.enabled = true;
    }

    bool flag = false;
    if (parseBool(JsonMini::readValue(couplingObj, "readEveryIteration"), flag) ||
        parseBool(JsonMini::readValue(couplingObj, "readEachIteration"), flag))
    {
        cfg.coupling.readEveryIteration = flag;
    }
    if (parseBool(JsonMini::readValue(couplingObj, "writeEveryIteration"), flag) ||
        parseBool(JsonMini::readValue(couplingObj, "writeEachIteration"), flag))
    {
        cfg.coupling.writeEveryIteration = flag;
    }
}

const std::string nodesArr = JsonMini::readValue(content, "nodes");
for (const std::string &nodeObj : JsonMini::readArrayItems(nodesArr))
{
    int id = -1;
    parseInt(JsonMini::readValue(nodeObj, "id"), id);
    if (id < 0)
    {
        continue;
    }

    Node *node = ensureNode(solver, id);
    if (node == nullptr)
    {
        continue;
    }

    const std::string name = JsonMini::readValue(nodeObj, "name");
    if (!name.empty())
    {
        node->name = name;
    }

    const std::string p = JsonMini::readValue(nodeObj, "pressure");
    const std::string t = JsonMini::readValue(nodeObj, "temperature");
    const std::string inj = JsonMini::readValue(nodeObj, "injection");
    const std::string injT = JsonMini::readValue(nodeObj, "injectionT");
    const std::string fixed = toLowerCopy(JsonMini::readValue(nodeObj, "fixed"));

    if (!p.empty())
    {
        parseDouble(p, node->pressure);
    }
    if (!t.empty())
    {
        parseDouble(t, node->temperature);
    }
    if (!inj.empty())
    {
        parseDouble(inj, node->flowInjection);
    }
    if (!injT.empty())
    {
        parseDouble(injT, node->thermalInjectionPower);
    }
    if (fixed == "true" || fixed == "1")
    {
        node->isPressureFixed = true;
        node->fixedPressure = node->pressure;
        solver.addBoundaryCondition(BoundaryCondition(id, BoundaryConditionType::Pressure,
                                                      node->pressure));
    }
}

const std::string pipesArr = JsonMini::readValue(content, "pipes");
for (const std::string &pipeObj : JsonMini::readArrayItems(pipesArr))
{
    std::vector<int> ports =
        JsonMini::readIntArray(JsonMini::readValue(pipeObj, "ports"));
    if (ports.size() < 2)
    {
        int from = -1;
        int to = -1;
        parseInt(JsonMini::readValue(pipeObj, "from"), from);
        parseInt(JsonMini::readValue(pipeObj, "to"), to);
        if (from < 0 || to < 0)
        {
            continue;
        }
        ports.clear();
        ports.push_back(from);
        ports.push_back(to);
    }

    for (int nodeId : ports)
    {
        if (nodeId >= 0)
        {
            ensureNode(solver, nodeId);
        }
    }

    std::vector<double> vals;
    double value = 0.0;

    const std::string resistance = JsonMini::readValue(pipeObj, "resistance");
    if (!resistance.empty() && parseDouble(resistance, value))
    {
        vals.push_back(value);
    }
    else
    {
        const std::string length = JsonMini::readValue(pipeObj, "length");
        const std::string diameter = JsonMini::readValue(pipeObj, "diameter");
        const std::string roughness = JsonMini::readValue(pipeObj, "roughness");
        if (parseDouble(length, value))
        {
            vals.push_back(value);
        }
        if (parseDouble(diameter, value))
        {
            vals.push_back(value);
        }
        if (parseDouble(roughness, value))
        {
            vals.push_back(value);
        }
    }

    if (vals.empty())
    {
        vals.push_back(10.0);
        vals.push_back(0.05);
        vals.push_back(0.0001);
    }
    const int compIndex = addPipeComponent(solver, vals);
    connectComponentPorts(solver, compIndex, ports);
}

const std::string valvesArr = JsonMini::readValue(content, "valves");
for (const std::string &valveObj : JsonMini::readArrayItems(valvesArr))
{
    std::vector<int> ports =
        JsonMini::readIntArray(JsonMini::readValue(valveObj, "ports"));
    if (ports.size() < 2)
    {
        int from = -1;
        int to = -1;
        parseInt(JsonMini::readValue(valveObj, "from"), from);
        parseInt(JsonMini::readValue(valveObj, "to"), to);
        if (from < 0 || to < 0)
        {
            continue;
        }
        ports.clear();
        ports.push_back(from);
        ports.push_back(to);
    }

    for (int nodeId : ports)
    {
        if (nodeId >= 0)
        {
            ensureNode(solver, nodeId);
        }
    }

    double kv = 1.0;
    double open = 1.0;
    parseDouble(JsonMini::readValue(valveObj, "kv"), kv);
    parseDouble(JsonMini::readValue(valveObj, "open"), open);

    const int compIndex = addValveComponent(solver, kv, open);
    connectComponentPorts(solver, compIndex, ports);
}

const std::string hexArr = JsonMini::readValue(content, "heatExchangers");
for (const std::string &hexObj : JsonMini::readArrayItems(hexArr))
{
    std::vector<int> ports =
        JsonMini::readIntArray(JsonMini::readValue(hexObj, "ports"));
    if (ports.size() < 2)
    {
        int from = -1;
        int to = -1;
        parseInt(JsonMini::readValue(hexObj, "from"), from);
        parseInt(JsonMini::readValue(hexObj, "to"), to);
        if (from < 0 || to < 0)
        {
            continue;
        }
        ports.clear();
        ports.push_back(from);
        ports.push_back(to);
    }

    for (int nodeId : ports)
    {
        if (nodeId >= 0)
        {
            ensureNode(solver, nodeId);
        }
    }

    double ua = 1.0;
    double area = 0.01;
    double length = 1.0;
    double pitch = 10.0;

    parseDouble(JsonMini::readValue(hexObj, "UA"), ua);
    parseDouble(JsonMini::readValue(hexObj, "flowArea"), area);
    parseDouble(JsonMini::readValue(hexObj, "length"), length);
    parseDouble(JsonMini::readValue(hexObj, "pitchSpacing"), pitch);

    std::vector<double> vals;
    vals.push_back(ua);
    vals.push_back(area);
    vals.push_back(length);
    vals.push_back(pitch);
    const int compIndex = addHeatExchangerComponent(solver, vals);
    connectComponentPorts(solver, compIndex, ports);
}

// Initialize material library and load default materials
MaterialLibrary matLib;
std::ifstream fluidsFileStream("fluids/water.json");
if (fluidsFileStream.is_open())
{
    std::stringstream fluidsBuffer;
    fluidsBuffer << fluidsFileStream.rdbuf();
    matLib.loadFromJsonString(fluidsBuffer.str());
    std::cout << "[DEBUG] Loaded materials from fluids/water.json" << std::endl;
    auto names = matLib.listMaterialNames();
    std::cout << "[DEBUG] Available materials: ";
    for (const auto &n : names)
        std::cout << n << " ";
    std::cout << std::endl;
}

// Load working fluid/materials from JSON if provided
{
    std::string wfObj = JsonMini::readValue(content, "workingFluid");
    if (wfObj.empty())
        wfObj = JsonMini::readValue(content, "fluid");
    if (!wfObj.empty())
    {
        // Check if wfObj is a material name or a JSON object
        std::string wfName = stripOptionalQuotes(wfObj);
        wfName = trim(wfName);
        if (wfName.front() == '{' || wfName.find('"') != wfName.find_last_of('"'))
        {
            // It's a JSON object
            MaterialProperties mat = MaterialProperties::fromJson(wfObj);
            solver.setWorkingFluid(mat);
        }
        else if (matLib.hasMaterial(wfName))
        {
            // It's a material name - use library
            MaterialProperties mat = matLib.getMaterial(wfName);
            solver.setWorkingFluid(mat);
            std::cout << "[DEBUG] Set working fluid to \"" << wfName << "\" from library" << std::endl;
        }
        else
        {
            // Try parsing as JSON anyway
            MaterialProperties mat = MaterialProperties::fromJson(wfObj);
            solver.setWorkingFluid(mat);
        }
    }
    else
    {
        const std::string mats = JsonMini::readValue(content, "materials");
        if (!mats.empty())
        {
            auto items = JsonMini::readArrayItems(mats);
            if (!items.empty())
            {
                MaterialProperties mat = MaterialProperties::fromJson(items[0]);
                solver.setWorkingFluid(mat);
            }
        }
    }
}

const std::string componentsArr = JsonMini::readValue(content, "components");
for (const std::string &compObj : JsonMini::readArrayItems(componentsArr))
{
    std::string type = JsonMini::readValue(compObj, "type");
    type = toLowerCopy(stripOptionalQuotes(type));
    std::string typeKey = type;
    typeKey.erase(std::remove_if(typeKey.begin(),
                                 typeKey.end(),
                                 [](char c)
                                 { return c == '_' || c == '-'; }),
                  typeKey.end());
    if (typeKey.empty())
    {
        continue;
    }

    std::vector<int> ports =
        JsonMini::readIntArray(JsonMini::readValue(compObj, "ports"));
    if (ports.size() < 2)
    {
        int from = -1;
        int to = -1;
        parseInt(JsonMini::readValue(compObj, "from"), from);
        parseInt(JsonMini::readValue(compObj, "to"), to);
        if (from < 0 || to < 0)
        {
            continue;
        }
        ports.clear();
        ports.push_back(from);
        ports.push_back(to);
    }

    for (int nodeId : ports)
    {
        if (nodeId >= 0)
        {
            ensureNode(solver, nodeId);
        }
    }

    const MaterialProperties &fluid = solver.getWorkingFluid();
    std::ostringstream name;
    name << typeKey << "_" << solver.getComponents().size();

    auto connectPorts = [&](int compIndex)
    {
        connectComponentPorts(solver, compIndex, ports);
    };

    auto connectTwoPorts = [&](int compIndex)
    {
        if (ports.size() >= 2)
        {
            solver.connectComponent(compIndex, ports[0], ports[1]);
        }
    };

    auto readDouble = [&](const char *key, double &out)
    {
        const std::string v = JsonMini::readValue(compObj, key);
        if (!v.empty())
        {
            parseDouble(v, out);
        }
    };

    auto readDoubleFlag = [&](const char *key, double &out, bool &flag)
    {
        const std::string v = JsonMini::readValue(compObj, key);
        if (!v.empty() && parseDouble(v, out))
        {
            flag = true;
        }
    };

    if (typeKey == "bend")
    {
        double k = 0.5;
        double dia = 0.05;
        double angle = 90.0;
        readDouble("lossCoefficient", k);
        readDouble("k", k);
        readDouble("diameter", dia);
        readDouble("angle", angle);
        readDouble("angleDeg", angle);
        readDouble("bendAngleDeg", angle);
        std::unique_ptr<Component> comp = createComponentFromParams(typeKey, name.str(), fluid, {});
        if (comp)
        {
            const int compIndex = static_cast<int>(solver.getComponents().size());
            solver.addComponent(std::move(comp));
            connectTwoPorts(compIndex);
        }
        continue;
    }

    // Unified component creation using centralized factory function
    // This applies to all remaining component types (tjoint, yjoint, channel, filters, etc.)
    // Parameters are extracted on-demand by createComponentFromParams based on type
    {
        // Pre-extract all possible parameters into a map for the factory function
        std::map<std::string, std::string> params;

        // Component-specific parameters
        auto extractParam = [&](const char *key)
        {
            std::string val = JsonMini::readValue(compObj, key);
            if (!val.empty())
            {
                params[key] = val;
            }
        };

        // Geometry parameters
        extractParam("lossCoefficient");
        extractParam("k");
        extractParam("diameter");
        extractParam("diameterIn");
        extractParam("diameterInlet");
        extractParam("diameterOut");
        extractParam("diameterOutlet");
        extractParam("length");
        extractParam("width");
        extractParam("height");
        extractParam("angle");
        extractParam("angleDeg");
        extractParam("bendAngleDeg");
        extractParam("branchLoss");
        extractParam("branchLossCoefficient");
        extractParam("roughness");
        extractParam("fouling");
        extractParam("foulingFactor");

        // Component-specific parameters
        extractParam("volume");
        extractParam("initialPressure");
        extractParam("initialPressureGauge");
        extractParam("prechargeP");
        extractParam("preChargePressure");
        extractParam("maxP");
        extractParam("maxPressure");
        extractParam("interfaceArea");
        extractParam("efficiency");

        // Valve parameters
        extractParam("flowCoefficient");
        extractParam("cv");
        extractParam("open");
        extractParam("opening");
        extractParam("setPressure");
        extractParam("deadband");
        extractParam("direction");
        extractParam("flowDirection");
        extractParam("targetFlow");
        extractParam("targetFlowRate");
        extractParam("controlSignal");
        extractParam("signal");
        extractParam("value");

        // Pump parameters
        extractParam("ratedFlow");
        extractParam("shutoffPressure");
        extractParam("ratedPressure");
        extractParam("pressureRise");
        extractParam("allowReverse");

        // Heat exchanger parameters
        extractParam("ua");
        extractParam("flowArea");
        extractParam("area");
        extractParam("pitch");

        // Dynamic parameters
        extractParam("dynamic");

        // Call the centralized componentcreation function
        std::unique_ptr<Component> comp = createComponentFromParams(typeKey, name.str(), fluid, params);

        if (comp)
        {
            const int compIndex = static_cast<int>(solver.getComponents().size());
            solver.addComponent(std::move(comp));

            // Connect ports based on component type
            if (typeKey == "tjoint" || typeKey == "yjoint" || typeKey == "couplingflow" || typeKey == "couplingthermal")
            {
                connectPorts(compIndex);
            }
            else if (typeKey == "coupling" || typeKey == "couplinglink")
            {
                // Multi-port coupling components
                connectPorts(compIndex);
                cfg.coupling.enabled = true;
            }
            else
            {
                // Default: two-port components
                connectTwoPorts(compIndex);
            }
        }
        else
        {
            // Unknown component type - log warning
            std::cerr << "Warning in loadFromJsonFile: Failed to create component of type '"
                      << typeKey << "' named '" << name.str() << "'\n";
        }
        continue;
    }
}

solver.setCalculationParameters(cfg.transientTimeStep,
                                cfg.tolerance,
                                cfg.maxIterations,
                                cfg.relaxationFactor);
solver.setConvergenceParameters(cfg.tolerance,
                                cfg.relativeConvergenceTolerance,
                                cfg.solutionIncrementTolerance);
solver.setSteadySolverTypeByName(cfg.solverMethod);
solver.setDebugEnabled(cfg.debugEnabled);
solver.setCouplingConfig(cfg.coupling);
solver.setCpuThreads(cfg.cpuThreads);
if (!cfg.computeBackend.empty())
{
    if (!solver.setComputeBackendByName(cfg.computeBackend))
    {
        std::cerr << "Warning: unknown backend \"" << cfg.computeBackend << "\"." << std::endl;
    }
}

return true;
}

bool validateSolverNetwork(const FluidNetworkSolver &solver, std::string *errorMessage)
{
    if (solver.getNodes().empty())
    {
        if (errorMessage)
        {
            *errorMessage = "Network has no nodes";
        }
        return false;
    }
    if (solver.getComponents().empty())
    {
        if (errorMessage)
        {
            *errorMessage = "Network has no components";
        }
        return false;
    }

    bool hasPressureConstraint = false;
    for (const auto &bc : solver.getBoundaryConditions())
    {
        if (bc.type == BoundaryConditionType::Pressure)
        {
            hasPressureConstraint = true;
            break;
        }
    }

    if (!hasPressureConstraint)
    {
        for (const auto &node : solver.getNodes())
        {
            if (node && node->isPressureFixed)
            {
                hasPressureConstraint = true;
                break;
            }
        }
    }

    if (!hasPressureConstraint)
    {
        if (errorMessage)
        {
            *errorMessage = "Network has no pressure reference boundary condition";
        }
        return false;
    }

    return true;
}

bool dumpSteadyStateToJson(const FluidNetworkSolver &solver, const std::string &filePath)
{
    std::ofstream out(filePath.c_str());
    if (!out.is_open())
    {
        std::cerr << "Cannot open result file: " << filePath << std::endl;
        return false;
    }

    out << "{\n";
    out << "  \"simulationType\": \"steady_state\",\n";
    out << "  \"solver\": {\n";
    out << "    \"method\": \"" << steadySolverTypeName(solver.getSteadySolverType()) << "\",\n";
    out << "    \"iterations\": " << solver.getLastIterationHistory().totalIterations << ",\n";
    out << "    \"converged\": "
        << (solver.getLastIterationHistory().converged ? "true" : "false") << "\n";
    out << "  },\n";

    out << "  \"nodes\": [\n";
    const auto &nodes = solver.getNodes();
    for (std::size_t i = 0; i < nodes.size(); ++i)
    {
        const Node *node = nodes[i].get();
        if (node == nullptr)
        {
            continue;
        }

        out << "    {\n";
        out << "      \"id\": " << node->id << ",\n";
        out << "      \"name\": \"" << node->name << "\",\n";
        out << "      \"pressure_Pa\": " << std::setprecision(12) << node->pressure << ",\n";
        out << "      \"temperature_K\": " << std::setprecision(8) << node->temperature << ",\n";
        out << "      \"flow_injection\": " << std::setprecision(8) << node->flowInjection << ",\n";
        out << "      \"thermal_injection_W\": " << std::setprecision(8)
            << node->thermalInjectionPower << "\n";
        out << "    }" << (i + 1 < nodes.size() ? "," : "") << "\n";
    }
    out << "  ],\n";

    out << "  \"connections\": [\n";
    const auto &conns = solver.getComponentConnections();
    const auto &components = solver.getComponents();
    const auto &flows = solver.getLatestConnectionFlows();
    const MaterialProperties &fallbackFluid = solver.getWorkingFluid();
    for (std::size_t i = 0; i < conns.size(); ++i)
    {
        const ComponentConnection &conn = conns[i];
        const Component *comp = nullptr;
        if (conn.componentIndex >= 0 &&
            conn.componentIndex < static_cast<int>(components.size()))
        {
            comp = components[static_cast<std::size_t>(conn.componentIndex)].get();
        }

        const double flow = (i < flows.size()) ? flows[i] : 0.0;
        const double flowMag = std::abs(flow);
        const double fromFlow = (flow >= 0.0) ? flowMag : -flowMag;
        const double toFlow = -fromFlow;
        const char *fromRole = (fromFlow >= 0.0) ? "inlet" : "outlet";
        const char *toRole = (toFlow >= 0.0) ? "inlet" : "outlet";
        const Node *fromNode = solver.getNode(conn.fromNodeId);
        const Node *toNode = solver.getNode(conn.toNodeId);
        const double fromPressure = fromNode ? fromNode->pressure : 0.0;
        const double fromTemperature = fromNode ? fromNode->temperature : 0.0;
        const double toPressure = toNode ? toNode->pressure : 0.0;
        const double toTemperature = toNode ? toNode->temperature : 0.0;
        const double area = componentFlowArea(comp);
        const FluidPropsView fromFluid =
            getComponentFluidProps(comp, fallbackFluid, fromPressure, fromTemperature);
        const FluidPropsView toFluid =
            getComponentFluidProps(comp, fallbackFluid, toPressure, toTemperature);

        out << "    {\n";
        out << "      \"component_index\": " << conn.componentIndex << ",\n";
        out << "      \"type\": \"" << (comp ? comp->getType() : "Unknown") << "\",\n";
        out << "      \"name\": \"" << (comp ? comp->getName() : "Unknown") << "\",\n";
        out << "      \"from_node\": " << conn.fromNodeId << ",\n";
        out << "      \"to_node\": " << conn.toNodeId << ",\n";
        out << "      \"from_port\": " << conn.fromPortIndex << ",\n";
        out << "      \"to_port\": " << conn.toPortIndex << ",\n";
        out << "      \"flow\": " << flow << ",\n";
        out << "      \"parameters\": {\n";
        writeComponentParameters(out, "        ", comp);
        out << "      },\n";
        out << "      \"results\": {\n";
        writeComponentResults(out, "        ", comp);
        out << "      },\n";
        out << "      \"ports\": [\n";
        out << "        {\n";
        writePortFields(out,
                        "          ",
                        conn.fromPortIndex,
                        fromRole,
                        conn.fromNodeId,
                        fromPressure,
                        fromTemperature,
                        fromFluid,
                        fromFlow,
                        area);
        out << "        },\n";
        out << "        {\n";
        writePortFields(out,
                        "          ",
                        conn.toPortIndex,
                        toRole,
                        conn.toNodeId,
                        toPressure,
                        toTemperature,
                        toFluid,
                        toFlow,
                        area);
        out << "        }\n";
        out << "      ]\n";
        out << "    }" << (i + 1 < conns.size() ? "," : "") << "\n";
    }
    out << "  ],\n";

    out << "  \"residuals\": [";
    const auto &residuals = solver.getLastIterationHistory().residuals;
    for (std::size_t i = 0; i < residuals.size(); ++i)
    {
        out << residuals[i];
        if (i + 1 < residuals.size())
        {
            out << ", ";
        }
    }
    out << "]";

    if (solver.isDebugEnabled())
    {
        out << ",\n";
        out << "  \"debug\": {\n";
        out << "    \"enabled\": true,\n";
        writeDebugIterations(out, "    ", solver.getLastIterationComponentResults());
        out << "  }\n";
    }
    else
    {
        out << "\n";
    }

    out << "}\n";
    return true;
}

bool dumpTransientToJson(const FluidNetworkSolver &solver, const std::string &filePath)
{
    std::ofstream out(filePath.c_str());
    if (!out.is_open())
    {
        std::cerr << "Cannot open result file: " << filePath << std::endl;
        return false;
    }

    const auto &snapshots = solver.getTimeStepSnapshots();
    const auto &nodes = solver.getNodes();
    const auto &components = solver.getComponents();
    const auto &conns = solver.getComponentConnections();
    const MaterialProperties &fallbackFluid = solver.getWorkingFluid();

    out << "{\n";
    out << "  \"simulationType\": \"transient\",\n";
    out << "  \"timeStepCount\": " << snapshots.size() << ",\n";
    out << "  \"timeSteps\": [\n";

    for (std::size_t s = 0; s < snapshots.size(); ++s)
    {
        const TimeStepSnapshot &snapshot = snapshots[s];
        out << "    {\n";
        out << "      \"time\": " << snapshot.time << ",\n";

        out << "      \"nodePressures\": [";
        for (std::size_t i = 0; i < snapshot.nodePressures.size(); ++i)
        {
            out << snapshot.nodePressures[i];
            if (i + 1 < snapshot.nodePressures.size())
            {
                out << ", ";
            }
        }
        out << "],\n";

        out << "      \"nodeTemperatures\": [";
        for (std::size_t i = 0; i < snapshot.nodeTemperatures.size(); ++i)
        {
            out << snapshot.nodeTemperatures[i];
            if (i + 1 < snapshot.nodeTemperatures.size())
            {
                out << ", ";
            }
        }
        out << "],\n";

        out << "      \"connectionFlows\": [";
        for (std::size_t i = 0; i < snapshot.connectionFlows.size(); ++i)
        {
            out << snapshot.connectionFlows[i];
            if (i + 1 < snapshot.connectionFlows.size())
            {
                out << ", ";
            }
        }
        out << "],\n";

        writeNodesFromSnapshot(out,
                               "      ",
                               nodes,
                               snapshot.nodePressures,
                               snapshot.nodeTemperatures,
                               true);

        out << "      \"connections\": [\n";
        for (std::size_t i = 0; i < conns.size(); ++i)
        {
            const ComponentConnection &conn = conns[i];
            const Component *comp = nullptr;
            if (conn.componentIndex >= 0 &&
                conn.componentIndex < static_cast<int>(components.size()))
            {
                comp = components[static_cast<std::size_t>(conn.componentIndex)].get();
            }

            double fromPressure = 0.0;
            double fromTemperature = 0.0;
            double toPressure = 0.0;
            double toTemperature = 0.0;
            getNodeStateFromSnapshot(nodes,
                                     snapshot.nodePressures,
                                     snapshot.nodeTemperatures,
                                     conn.fromNodeId,
                                     fromPressure,
                                     fromTemperature);
            getNodeStateFromSnapshot(nodes,
                                     snapshot.nodePressures,
                                     snapshot.nodeTemperatures,
                                     conn.toNodeId,
                                     toPressure,
                                     toTemperature);

            const double flow =
                (i < snapshot.connectionFlows.size()) ? snapshot.connectionFlows[i] : 0.0;
            const double flowMag = std::abs(flow);
            const double fromFlow = (flow >= 0.0) ? flowMag : -flowMag;
            const double toFlow = -fromFlow;
            const char *fromRole = (fromFlow >= 0.0) ? "inlet" : "outlet";
            const char *toRole = (toFlow >= 0.0) ? "inlet" : "outlet";
            const double area = componentFlowArea(comp);
            const FluidPropsView fromFluid =
                getComponentFluidProps(comp, fallbackFluid, fromPressure, fromTemperature);
            const FluidPropsView toFluid =
                getComponentFluidProps(comp, fallbackFluid, toPressure, toTemperature);

            out << "        {\n";
            out << "          \"component_index\": " << conn.componentIndex << ",\n";
            out << "          \"type\": \"" << (comp ? comp->getType() : "Unknown") << "\",\n";
            out << "          \"name\": \"" << (comp ? comp->getName() : "Unknown") << "\",\n";
            out << "          \"from_node\": " << conn.fromNodeId << ",\n";
            out << "          \"to_node\": " << conn.toNodeId << ",\n";
            out << "          \"from_port\": " << conn.fromPortIndex << ",\n";
            out << "          \"to_port\": " << conn.toPortIndex << ",\n";
            out << "          \"flow\": " << flow << ",\n";
            out << "          \"parameters\": {\n";
            writeComponentParameters(out, "            ", comp);
            out << "          },\n";
            out << "          \"results\": {\n";
            writeComponentResults(out, "            ", comp);
            out << "          },\n";
            out << "          \"ports\": [\n";
            out << "            {\n";
            writePortFields(out,
                            "              ",
                            conn.fromPortIndex,
                            fromRole,
                            conn.fromNodeId,
                            fromPressure,
                            fromTemperature,
                            fromFluid,
                            fromFlow,
                            area);
            out << "            },\n";
            out << "            {\n";
            writePortFields(out,
                            "              ",
                            conn.toPortIndex,
                            toRole,
                            conn.toNodeId,
                            toPressure,
                            toTemperature,
                            toFluid,
                            toFlow,
                            area);
            out << "            }\n";
            out << "          ]\n";
            out << "        }" << (i + 1 < conns.size() ? "," : "") << "\n";
        }
        out << "      ],\n";

        out << "      \"solver\": {\n";
        out << "        \"method\": \"" << steadySolverTypeName(solver.getSteadySolverType())
            << "\",\n";
        out << "        \"iterations\": " << snapshot.iterationHistory.totalIterations << ",\n";
        out << "        \"converged\": "
            << (snapshot.iterationHistory.converged ? "true" : "false") << ",\n";
        out << "        \"residuals\": [";
        for (std::size_t i = 0; i < snapshot.iterationHistory.residuals.size(); ++i)
        {
            out << snapshot.iterationHistory.residuals[i];
            if (i + 1 < snapshot.iterationHistory.residuals.size())
            {
                out << ", ";
            }
        }
        out << "]\n";
        out << "      }";

        if (solver.isDebugEnabled())
        {
            out << ",\n";
            out << "      \"debug\": {\n";
            out << "        \"enabled\": true,\n";
            writeDebugIterations(out, "        ", snapshot.iterationComponentResults);
            out << "      }\n";
        }
        else
        {
            out << "\n";
        }

        out << "    }" << (s + 1 < snapshots.size() ? "," : "") << "\n";
    }

    out << "  ]\n";
    out << "}\n";
    return true;
}

std::vector<std::string> getRegisteredComponentTypes()
{
    return ComponentRegistry::instance().listTypes();
}

bool isComponentTypeRegistered(const std::string &typeName)
{
    return ComponentRegistry::instance().hasType(typeName);
}

/**
 * @brief Helper function to parse a string to double with default value.
 *
 * @param value The string value to parse
 * @param defaultVal The default value if parsing fails
 * @return The parsed double or default value
 */
static double parseDoubleFromString(const std::string &value, double defaultVal = 0.0)
{
    if (value.empty())
        return defaultVal;
    try
    {
        return std::stod(value);
    }
    catch (...)
    {
        return defaultVal;
    }
}

std::unique_ptr<Component> createComponentFromParams(
    const std::string &typeKey,
    const std::string &componentName,
    const MaterialProperties &fluid,
    const std::map<std::string, std::string> &params)
{
    // Helper lambda to safely get parameter values
    auto getParam = [&](const std::vector<const char *> &keys, double defaultVal = 0.0) -> double
    {
        for (const auto *key : keys)
        {
            auto it = params.find(key);
            if (it != params.end() && !it->second.empty())
            {
                double val = parseDoubleFromString(it->second, defaultVal);
                if (val != defaultVal || parseDoubleFromString(it->second) == val)
                {
                    return val;
                }
            }
        }
        return defaultVal;
    };

    auto getBoolParam = [&](const std::vector<const char *> &keys, bool defaultVal = false) -> bool
    {
        for (const auto *key : keys)
        {
            auto it = params.find(key);
            if (it != params.end() && !it->second.empty())
            {
                const auto &v = it->second;
                if (v == "1" || v == "true" || v == "True" || v == "TRUE")
                    return true;
                if (v == "0" || v == "false" || v == "False" || v == "FALSE")
                    return false;
            }
        }
        return defaultVal;
    };

    // ===== Pipe (no parameters needed) =====
    if (typeKey == "pipe")
    {
        return std::make_unique<Components::Pipe>(componentName, fluid);
    }

    // ===== Bend =====
    if (typeKey == "bend")
    {
        double k = getParam({"lossCoefficient", "k"}, 0.5);
        double dia = getParam({"diameter"}, 0.05);
        double angle = getParam({"angle", "angleDeg", "bendAngleDeg"}, 90.0);
        return std::make_unique<Components::Bend>(componentName, k, dia, fluid, angle);
    }

    // ===== TJoint =====
    if (typeKey == "tjoint")
    {
        double k = getParam({"lossCoefficient", "k"}, 1.0);
        double dia = getParam({"diameter"}, 0.05);
        double branch = getParam({"branchLoss", "branchLossCoefficient"}, 1.0);
        return std::make_unique<Components::TJoint>(componentName, k, dia, fluid, branch);
    }

    // ===== YJoint =====
    if (typeKey == "yjoint")
    {
        double k = getParam({"lossCoefficient", "k"}, 0.7);
        double dia = getParam({"diameter"}, 0.05);
        double branch = getParam({"branchLoss", "branchLossCoefficient"}, 0.7);
        return std::make_unique<Components::YJoint>(componentName, k, dia, fluid, branch);
    }

    // ===== Channel =====
    if (typeKey == "channel")
    {
        double length = getParam({"length"}, 1.0);
        double width = getParam({"width"}, 0.01);
        double height = getParam({"height"}, 0.01);
        double rough = getParam({"roughness"}, 0.0);
        return std::make_unique<Components::Channel>(componentName, length, width, height, rough, fluid);
    }

    // ===== Filters =====
    if (typeKey == "liquidfilter")
    {
        double k = getParam({"lossCoefficient", "k"}, 1.0);
        double dia = getParam({"diameter"}, 0.05);
        double fouling = getParam({"fouling", "foulingFactor"}, 1.0);
        return std::make_unique<Components::LiquidFilter>(componentName, k, dia, fouling, fluid);
    }

    if (typeKey == "gasfilter")
    {
        double k = getParam({"lossCoefficient", "k"}, 1.0);
        double dia = getParam({"diameter"}, 0.05);
        double fouling = getParam({"fouling", "foulingFactor"}, 1.0);
        return std::make_unique<Components::GasFilter>(componentName, k, dia, fouling, fluid);
    }

    if (typeKey == "meshfilter")
    {
        double k = getParam({"lossCoefficient", "k"}, 1.0);
        double dia = getParam({"diameter"}, 0.05);
        double fouling = getParam({"fouling", "foulingFactor"}, 1.0);
        return std::make_unique<Components::MeshFilter>(componentName, k, dia, fouling, fluid);
    }

    if (typeKey == "paperfilter")
    {
        double k = getParam({"lossCoefficient", "k"}, 1.0);
        double dia = getParam({"diameter"}, 0.05);
        double fouling = getParam({"fouling", "foulingFactor"}, 1.0);
        return std::make_unique<Components::PaperFilter>(componentName, k, dia, fouling, fluid);
    }

    // ===== Expansion/Contraction =====
    if (typeKey == "expander")
    {
        double diamIn = getParam({"diameterIn", "diameterInlet"}, 0.05);
        double diamOut = getParam({"diameterOut", "diameterOutlet"}, 0.10);
        return std::make_unique<Components::Expander>(componentName, diamIn, diamOut, fluid);
    }

    if (typeKey == "contraction")
    {
        double diamIn = getParam({"diameterIn", "diameterInlet"}, 0.10);
        double diamOut = getParam({"diameterOut", "diameterOutlet"}, 0.05);
        return std::make_unique<Components::Contraction>(componentName, diamIn, diamOut, fluid);
    }

    // ===== Pressure Vessel & Level Tank =====
    if (typeKey == "pressurevessel")
    {
        double volume = getParam({"volume"}, 0.001);
        double initialP = getParam({"initialPressure", "initialPressureGauge"}, 1e5);
        return std::make_unique<Components::PressureVessel>(componentName, volume, initialP, fluid);
    }

    if (typeKey == "leveltank")
    {
        double volume = getParam({"volume"}, 0.001);
        return std::make_unique<Components::LevelTank>(componentName, volume, fluid);
    }

    // ===== Accumulators =====
    if (typeKey == "hydraulicaccumulator")
    {
        double volume = getParam({"volume"}, 0.001);
        double preCharge = getParam({"prechargeP", "preChargePressure"}, 1e5);
        double maxP = getParam({"maxP", "maxPressure"}, 2e5);
        return std::make_unique<Components::HydraulicAccumulator>(componentName, volume, preCharge, maxP, fluid);
    }

    if (typeKey == "pneumaticaccumulator")
    {
        double volume = getParam({"volume"}, 0.001);
        double preCharge = getParam({"prechargeP", "preChargePressure"}, 1e5);
        double maxP = getParam({"maxP", "maxPressure"}, 2e5);
        return std::make_unique<Components::PneumaticAccumulator>(componentName, volume, preCharge, maxP, fluid);
    }

    // ===== Seal =====
    if (typeKey == "seal")
    {
        double k = getParam({"lossCoefficient", "k"}, 0.5);
        double ia = getParam({"interfaceArea"}, 0.01);
        bool dynamic = getBoolParam({"dynamic"}, false);
        return std::make_unique<Components::Seal>(componentName, k, ia, dynamic, fluid);
    }

    // ===== Oil-Water Separator =====
    if (typeKey == "oilwaterseparator")
    {
        double efficiency = getParam({"efficiency"}, 0.9);
        return std::make_unique<Components::OilWaterSeparator>(componentName, efficiency, fluid);
    }

    // ===== Valves =====
    if (typeKey == "gatevalve")
    {
        double cv = getParam({"flowCoefficient", "cv"}, 1.0);
        double open = getParam({"open", "opening"}, 1.0);
        return std::make_unique<Components::GateValve>(componentName, cv, open, fluid);
    }

    if (typeKey == "ballvalve")
    {
        double cv = getParam({"flowCoefficient", "cv"}, 1.0);
        double open = getParam({"open", "opening"}, 1.0);
        return std::make_unique<Components::BallValve>(componentName, cv, open, fluid);
    }

    if (typeKey == "butterflyvalve")
    {
        double cv = getParam({"flowCoefficient", "cv"}, 1.0);
        double open = getParam({"open", "opening"}, 1.0);
        return std::make_unique<Components::ButterflyValve>(componentName, cv, open, fluid);
    }

    if (typeKey == "flowcontrolvalve")
    {
        double cv = getParam({"flowCoefficient", "cv"}, 1.0);
        double open = getParam({"open", "opening"}, 1.0);
        double targetFlow = getParam({"targetFlow", "targetFlowRate"}, 0.0);
        return std::make_unique<Components::FlowControlValve>(componentName, cv, open, fluid, targetFlow);
    }

    if (typeKey == "controlvalve")
    {
        double cv = getParam({"flowCoefficient", "cv"}, 1.0);
        double open = getParam({"open", "opening"}, 1.0);
        auto valve = std::make_unique<Components::ControlValve>(componentName, cv, open, fluid);
        double controlSignal = getParam({"controlSignal", "signal", "value"}, 0.0);
        if (controlSignal != 0.0)
        {
            valve->setControlSignal(controlSignal);
        }
        return valve;
    }

    if (typeKey == "isolationvalve")
    {
        double cv = getParam({"flowCoefficient", "cv"}, 1.0);
        double open = getParam({"open", "opening"}, 1.0);
        return std::make_unique<Components::IsolationValve>(componentName, cv, open > 0.5, fluid);
    }

    if (typeKey == "checkvalve")
    {
        double cv = getParam({"flowCoefficient", "cv"}, 1.0);
        double open = getParam({"open", "opening"}, 1.0);
        double direction = getParam({"direction", "flowDirection"}, 1.0);
        return std::make_unique<Components::CheckValve>(componentName, cv, open, fluid, direction);
    }

    if (typeKey == "pressurereliefvalve")
    {
        double cv = getParam({"flowCoefficient", "cv"}, 1.0);
        double open = getParam({"open", "opening"}, 1.0);
        double setP = getParam({"setPressure"}, 1e5);
        double band = getParam({"deadband"}, 0.0);
        return std::make_unique<Components::PressureReliefValve>(componentName, cv, open, fluid, setP, band);
    }

    // Gas Regulator (catch-all for other valve-like components)
    if (typeKey == "gasregulator")
    {
        double cv = getParam({"flowCoefficient", "cv"}, 1.0);
        double open = getParam({"open", "opening"}, 1.0);
        double setP = getParam({"setPressure"}, 1e5);
        double band = getParam({"deadband"}, 0.0);
        return std::make_unique<Components::GasRegulator>(componentName, cv, open, fluid, setP, band);
    }

    // ===== Pumps =====
    if (typeKey == "centrifugalpump")
    {
        double ratedFlow = getParam({"ratedFlow"}, 0.0);
        double shutoff = getParam({"shutoffPressure"}, 0.0);
        double rated = getParam({"ratedPressure"}, 0.0);
        bool allowReverse = getBoolParam({"allowReverse"}, false);
        return std::make_unique<Components::CentrifugalPump>(componentName, ratedFlow, shutoff, rated, fluid, allowReverse);
    }

    if (typeKey == "pistonpump")
    {
        double rise = getParam({"pressureRise"}, 0.0);
        bool allowReverse = getBoolParam({"allowReverse"}, false);
        return std::make_unique<Components::PistonPump>(componentName, rise, fluid, allowReverse);
    }

    if (typeKey == "gearpump")
    {
        double rise = getParam({"pressureRise"}, 0.0);
        bool allowReverse = getBoolParam({"allowReverse"}, false);
        return std::make_unique<Components::GearPump>(componentName, rise, fluid, allowReverse);
    }

    if (typeKey == "screwpump")
    {
        double rise = getParam({"pressureRise"}, 0.0);
        bool allowReverse = getBoolParam({"allowReverse"}, false);
        return std::make_unique<Components::ScrewPump>(componentName, rise, fluid, allowReverse);
    }

    if (typeKey == "pump")
    {
        return std::make_unique<Components::Pump>(componentName, fluid);
    }

    // ===== Heat Exchanger & Radiator =====
    if (typeKey == "heatexchanger")
    {
        double ua = getParam({"ua"}, 1.0);
        double flowArea = getParam({"flowArea"}, 0.01);
        double length = getParam({"length"}, 1.0);
        return std::make_unique<Components::HeatExchanger>(componentName, ua, flowArea, length, fluid);
    }

    if (typeKey == "radiator")
    {
        double ua = getParam({"ua"}, 1.0);
        double flowArea = getParam({"flowArea", "area"}, 0.01);
        double length = getParam({"length"}, 1.0);
        double pitch = getParam({"pitch"}, 10.0);
        return std::make_unique<Components::Radiator>(componentName, ua, flowArea, length, pitch, fluid);
    }

    // ===== Coupling Components =====
    if (typeKey == "couplingflow")
    {
        return std::make_unique<Components::CouplingFlow>(componentName);
    }

    if (typeKey == "couplingthermal")
    {
        return std::make_unique<Components::CouplingThermal>(componentName);
    }

    // Unknown component type
    std::cerr << "Warning: Unknown component type '" << typeKey << "'\n";
    return nullptr;
}

} // namespace NetworkIO
