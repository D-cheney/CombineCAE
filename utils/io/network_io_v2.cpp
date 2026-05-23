#include "network_io_v2.h"

#include "core/components.h"
#include "core/component_registry.h"
#include "json_schema_validator.h"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <sstream>

namespace NetworkIOv2 {

namespace {

// ============================================================================
// Utility Functions
// ============================================================================

std::string trim(const std::string& str) {
    const std::size_t start = str.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) return "";
    const std::size_t end = str.find_last_not_of(" \t\n\r");
    return str.substr(start, end - start + 1);
}

std::string toLowerCopy(const std::string& text) {
    std::string out = text;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

std::string toUpperCopy(const std::string& text) {
    std::string out = text;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::toupper(c)); });
    return out;
}

bool parseDouble(const std::string& text, double& value) {
    if (text.empty()) return false;
    try {
        value = std::stod(text);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseInt(const std::string& text, int& value) {
    if (text.empty()) return false;
    try {
        value = std::stoi(text);
        return true;
    } catch (...) {
        return false;
    }
}

bool parseBool(const std::string& text, bool& value) {
    const std::string key = toLowerCopy(trim(text));
    if (key == "true" || key == "1" || key == "yes" || key == "on") {
        value = true;
        return true;
    }
    if (key == "false" || key == "0" || key == "no" || key == "off") {
        value = false;
        return true;
    }
    return false;
}

std::string stripOptionalQuotes(const std::string& text) {
    std::string value = trim(text);
    if (value.size() >= 2 && value.front() == '"' && value.back() == '"') {
        value = value.substr(1, value.size() - 2);
    }
    return trim(value);
}

// ============================================================================
// JSON Mini Parser (inline, for v2 format)
// ============================================================================

namespace JsonMini {

std::string readValue(const std::string& content, const std::string& key) {
    std::size_t pos = content.find("\"" + key + "\"");
    if (pos == std::string::npos) return "";
    
    pos = content.find(':', pos);
    if (pos == std::string::npos) return "";
    
    ++pos;
    while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos]))) {
        ++pos;
    }
    if (pos >= content.size()) return "";
    
    if (content[pos] == '"') {
        ++pos;
        const std::size_t end = content.find('"', pos);
        if (end == std::string::npos) return "";
        return content.substr(pos, end - pos);
    }
    
    if (content[pos] == '{' || content[pos] == '[') {
        const char open = content[pos];
        const char close = (open == '{') ? '}' : ']';
        int depth = 1;
        std::size_t end = pos + 1;
        bool inString = false;
        while (end < content.size() && depth > 0) {
            const char c = content[end];
            if (c == '"' && (end == 0 || content[end - 1] != '\\')) {
                inString = !inString;
            }
            if (!inString) {
                if (c == open) ++depth;
                else if (c == close) --depth;
            }
            ++end;
        }
        return content.substr(pos, end - pos);
    }
    
    std::size_t end = pos;
    while (end < content.size() && content[end] != ',' && content[end] != '}' && content[end] != ']') {
        ++end;
    }
    return trim(content.substr(pos, end - pos));
}

std::vector<std::string> readArrayItems(const std::string& arrayStr) {
    std::vector<std::string> items;
    const std::size_t left = arrayStr.find('[');
    const std::size_t right = arrayStr.rfind(']');
    if (left == std::string::npos || right == std::string::npos || right <= left) {
        return items;
    }
    
    std::string body = arrayStr.substr(left + 1, right - left - 1);
    std::string item;
    int depth = 0;
    bool inString = false;
    
    for (std::size_t i = 0; i < body.size(); ++i) {
        const char c = body[i];
        if (c == '"' && (i == 0 || body[i - 1] != '\\')) {
            inString = !inString;
        }
        
        if (!inString) {
            if (c == '{' || c == '[') ++depth;
            else if (c == '}' || c == ']') --depth;
            else if (c == ',' && depth == 0) {
                const std::string t = trim(item);
                if (!t.empty()) items.push_back(t);
                item.clear();
                continue;
            }
        }
        item.push_back(c);
    }
    
    const std::string t = trim(item);
    if (!t.empty()) items.push_back(t);
    return items;
}

std::vector<std::string> readObjectKeys(const std::string& objectStr) {
    std::vector<std::string> keys;
    const std::size_t left = objectStr.find('{');
    const std::size_t right = objectStr.rfind('}');
    if (left == std::string::npos || right == std::string::npos || right <= left) {
        return keys;
    }
    
    std::size_t i = left + 1;
    int depth = 1;
    bool inString = false;
    while (i < right && depth > 0) {
        const char c = objectStr[i];
        if (c == '"' && (i == 0 || objectStr[i - 1] != '\\')) {
            if (!inString && depth == 1) {
                const std::size_t keyStart = i + 1;
                std::size_t keyEnd = keyStart;
                while (keyEnd < right) {
                    if (objectStr[keyEnd] == '"' && objectStr[keyEnd - 1] != '\\') break;
                    ++keyEnd;
                }
                if (keyEnd < right) {
                    std::size_t pos = keyEnd + 1;
                    while (pos < right && std::isspace(static_cast<unsigned char>(objectStr[pos]))) ++pos;
                    if (pos < right && objectStr[pos] == ':') {
                        keys.push_back(objectStr.substr(keyStart, keyEnd - keyStart));
                    }
                    i = keyEnd;
                }
            }
            inString = !inString;
            ++i;
            continue;
        }
        
        if (!inString) {
            if (c == '{' || c == '[') ++depth;
            else if (c == '}' || c == ']') --depth;
        }
        ++i;
    }
    return keys;
}

std::vector<int> readIntArray(const std::string& arrayStr) {
    std::vector<int> values;
    for (const std::string& item : readArrayItems(arrayStr)) {
        int v = 0;
        if (parseInt(stripOptionalQuotes(item), v)) {
            values.push_back(v);
        }
    }
    return values;
}

std::vector<std::string> readStringArray(const std::string& arrayStr) {
    std::vector<std::string> values;
    for (const std::string& item : readArrayItems(arrayStr)) {
        values.push_back(stripOptionalQuotes(item));
    }
    return values;
}

} // namespace JsonMini

// ============================================================================
// Parsing Functions
// ============================================================================

ValueUnit parseValueUnit(const std::string& objStr) {
    ValueUnit vu;
    const std::string valStr = JsonMini::readValue(objStr, "value");
    if (!valStr.empty()) {
        parseDouble(valStr, vu.value);
    }
    const std::string unitStr = JsonMini::readValue(objStr, "unit");
    if (!unitStr.empty()) {
        vu.unit = stripOptionalQuotes(unitStr);
    }
    return vu;
}

Meta parseMeta(const std::string& metaStr) {
    Meta meta;
    meta.formatName = stripOptionalQuotes(JsonMini::readValue(metaStr, "format_name"));
    meta.formatVersion = stripOptionalQuotes(JsonMini::readValue(metaStr, "format_version"));
    meta.caseId = stripOptionalQuotes(JsonMini::readValue(metaStr, "case_id"));
    meta.title = stripOptionalQuotes(JsonMini::readValue(metaStr, "title"));
    meta.description = stripOptionalQuotes(JsonMini::readValue(metaStr, "description"));
    meta.author = stripOptionalQuotes(JsonMini::readValue(metaStr, "author"));
    meta.createdAt = stripOptionalQuotes(JsonMini::readValue(metaStr, "created_at"));
    meta.units = stripOptionalQuotes(JsonMini::readValue(metaStr, "units"));
    
    const std::string tagsArr = JsonMini::readValue(metaStr, "tags");
    meta.tags = JsonMini::readStringArray(tagsArr);
    
    const std::string sourceStr = JsonMini::readValue(metaStr, "source");
    if (!sourceStr.empty()) {
        meta.source.generator = stripOptionalQuotes(JsonMini::readValue(sourceStr, "generator"));
        meta.source.generatorVersion = stripOptionalQuotes(JsonMini::readValue(sourceStr, "generator_version"));
    }
    
    return meta;
}

NodeDef parseNodeDef(const std::string& nodeStr) {
    NodeDef node;
    node.id = stripOptionalQuotes(JsonMini::readValue(nodeStr, "id"));
    node.name = stripOptionalQuotes(JsonMini::readValue(nodeStr, "name"));
    node.type = stripOptionalQuotes(JsonMini::readValue(nodeStr, "type"));
    
    const std::string coordsArr = JsonMini::readValue(nodeStr, "coords");
    if (!coordsArr.empty()) {
        const auto items = JsonMini::readArrayItems(coordsArr);
        for (const auto& item : items) {
            double v = 0.0;
            if (parseDouble(item, v)) {
                node.coords.push_back(v);
            }
        }
    }
    
    const std::string varsStr = JsonMini::readValue(nodeStr, "variables");
    if (!varsStr.empty()) {
        const std::string pressStr = JsonMini::readValue(varsStr, "pressure");
        if (!pressStr.empty()) {
            node.variables.pressure = parseValueUnit(pressStr);
        }
        const std::string tempStr = JsonMini::readValue(varsStr, "temperature");
        if (!tempStr.empty()) {
            node.variables.temperature = parseValueUnit(tempStr);
        }
    }
    
    const std::string constrStr = JsonMini::readValue(nodeStr, "constraints");
    if (!constrStr.empty()) {
        bool bVal = false;
        if (parseBool(JsonMini::readValue(constrStr, "pressure_fixed"), bVal)) {
            node.constraints.pressureFixed = bVal;
        }
    }
    
    return node;
}

PortDef parsePortDef(const std::string& portStr) {
    PortDef port;
    port.name = stripOptionalQuotes(JsonMini::readValue(portStr, "name"));
    port.nodeRef = stripOptionalQuotes(JsonMini::readValue(portStr, "node"));
    int idx = 0;
    if (parseInt(JsonMini::readValue(portStr, "portIndex"), idx)) {
        port.portIndex = idx;
    }
    return port;
}

ComponentDef parseComponentDef(const std::string& compStr) {
    ComponentDef comp;
    comp.id = stripOptionalQuotes(JsonMini::readValue(compStr, "id"));
    comp.name = stripOptionalQuotes(JsonMini::readValue(compStr, "name"));
    comp.type = stripOptionalQuotes(JsonMini::readValue(compStr, "type"));
    comp.subtype = stripOptionalQuotes(JsonMini::readValue(compStr, "subtype"));
    comp.materialRef = stripOptionalQuotes(JsonMini::readValue(compStr, "material_ref"));
    
    // Parse ports
    const std::string portsArr = JsonMini::readValue(compStr, "ports");
    for (const std::string& portStr : JsonMini::readArrayItems(portsArr)) {
        comp.ports.push_back(parsePortDef(portStr));
    }
    
    // Parse geometry
    const std::string geomStr = JsonMini::readValue(compStr, "geometry");
    if (!geomStr.empty()) {
        const std::string lenStr = JsonMini::readValue(geomStr, "length");
        if (!lenStr.empty()) comp.geometry.length = parseValueUnit(lenStr);
        const std::string diaStr = JsonMini::readValue(geomStr, "diameter");
        if (!diaStr.empty()) comp.geometry.diameter = parseValueUnit(diaStr);
        const std::string roughStr = JsonMini::readValue(geomStr, "roughness");
        if (!roughStr.empty()) comp.geometry.roughness = parseValueUnit(roughStr);
    }
    
    // Parse parameters
    const std::string paramsStr = JsonMini::readValue(compStr, "parameters");
    if (!paramsStr.empty()) {
        for (const std::string& key : JsonMini::readObjectKeys(paramsStr)) {
            comp.parameters[key] = JsonMini::readValue(paramsStr, key);
        }
    }
    
    // Parse capabilities
    const std::string capArr = JsonMini::readValue(compStr, "capabilities");
    comp.capabilities = JsonMini::readStringArray(capArr);
    
    return comp;
}

MaterialDef parseMaterialDef(const std::string& matStr) {
    MaterialDef mat;
    mat.id = stripOptionalQuotes(JsonMini::readValue(matStr, "id"));
    mat.name = stripOptionalQuotes(JsonMini::readValue(matStr, "name"));
    mat.phase = stripOptionalQuotes(JsonMini::readValue(matStr, "phase"));
    mat.fluidType = stripOptionalQuotes(JsonMini::readValue(matStr, "fluid_type"));
    
    const std::string propsStr = JsonMini::readValue(matStr, "properties");
    if (!propsStr.empty()) {
        const std::string densStr = JsonMini::readValue(propsStr, "density");
        if (!densStr.empty()) mat.properties.density = parseValueUnit(densStr);
        const std::string viscStr = JsonMini::readValue(propsStr, "dynamic_viscosity");
        if (!viscStr.empty()) mat.properties.dynamicViscosity = parseValueUnit(viscStr);
        const std::string cpStr = JsonMini::readValue(propsStr, "specific_heat");
        if (!cpStr.empty()) mat.properties.specificHeat = parseValueUnit(cpStr);
        const std::string kStr = JsonMini::readValue(propsStr, "thermal_conductivity");
        if (!kStr.empty()) mat.properties.thermalConductivity = parseValueUnit(kStr);
    }
    
    return mat;
}

BoundaryDef parseBoundaryDef(const std::string& bcStr) {
    BoundaryDef bc;
    bc.id = stripOptionalQuotes(JsonMini::readValue(bcStr, "id"));
    
    const std::string targetStr = JsonMini::readValue(bcStr, "target");
    if (!targetStr.empty()) {
        bc.target.kind = stripOptionalQuotes(JsonMini::readValue(targetStr, "kind"));
        bc.target.id = stripOptionalQuotes(JsonMini::readValue(targetStr, "id"));
    }
    
    bc.variable = stripOptionalQuotes(JsonMini::readValue(bcStr, "variable"));
    bc.type = stripOptionalQuotes(JsonMini::readValue(bcStr, "type"));
    
    const std::string valStr = JsonMini::readValue(bcStr, "value");
    if (!valStr.empty()) {
        bc.value = parseValueUnit(valStr);
    }
    
    return bc;
}

FrameDef parseFrameDef(const std::string& frameStr) {
    FrameDef frame;
    double time = 0.0;
    if (parseDouble(JsonMini::readValue(frameStr, "time"), time)) {
        frame.time = time;
    }
    
    // Parse control updates
    const std::string ctrlArr = JsonMini::readValue(frameStr, "control_updates");
    for (const std::string& ctrlStr : JsonMini::readArrayItems(ctrlArr)) {
        FrameDef::ControlUpdate update;
        const std::string targetStr = JsonMini::readValue(ctrlStr, "target");
        if (!targetStr.empty()) {
            update.target.kind = stripOptionalQuotes(JsonMini::readValue(targetStr, "kind"));
            update.target.id = stripOptionalQuotes(JsonMini::readValue(targetStr, "id"));
        }
        update.variable = stripOptionalQuotes(JsonMini::readValue(ctrlStr, "variable"));
        parseDouble(JsonMini::readValue(ctrlStr, "value"), update.value);
        frame.controlUpdates.push_back(update);
    }
    
    // Parse boundary updates
    const std::string bcArr = JsonMini::readValue(frameStr, "boundary_updates");
    for (const std::string& bcStr : JsonMini::readArrayItems(bcArr)) {
        FrameDef::BoundaryUpdate update;
        const std::string targetStr = JsonMini::readValue(bcStr, "target");
        if (!targetStr.empty()) {
            update.target.kind = stripOptionalQuotes(JsonMini::readValue(targetStr, "kind"));
            update.target.id = stripOptionalQuotes(JsonMini::readValue(targetStr, "id"));
        }
        update.variable = stripOptionalQuotes(JsonMini::readValue(bcStr, "variable"));
        update.type = stripOptionalQuotes(JsonMini::readValue(bcStr, "type"));
        const std::string valStr = JsonMini::readValue(bcStr, "value");
        if (!valStr.empty()) {
            update.value = parseValueUnit(valStr);
        }
        frame.boundaryUpdates.push_back(update);
    }
    
    return frame;
}

PhysicsDef parsePhysicsDef(const std::string& physicsStr) {
    PhysicsDef physics;
    
    const std::string modulesArr = JsonMini::readValue(physicsStr, "active_modules");
    physics.activeModules = JsonMini::readStringArray(modulesArr);
    if (physics.activeModules.empty()) {
        physics.activeModules.push_back("fluid");
    }
    
    const std::string fluidStr = JsonMini::readValue(physicsStr, "fluid");
    if (!fluidStr.empty()) {
        physics.fluid.formulation = stripOptionalQuotes(JsonMini::readValue(fluidStr, "formulation"));
        bool bVal = false;
        if (parseBool(JsonMini::readValue(fluidStr, "energy"), bVal)) {
            physics.fluid.energy = bVal;
        }
    }
    
    return physics;
}

ControlsDef parseControlsDef(const std::string& controlsStr) {
    ControlsDef controls;
    controls.analysisType = stripOptionalQuotes(JsonMini::readValue(controlsStr, "analysis_type"));
    
    const std::string solverStr = JsonMini::readValue(controlsStr, "solver");
    if (!solverStr.empty()) {
        controls.solver.steadyMethod = stripOptionalQuotes(JsonMini::readValue(solverStr, "steady_method"));
        controls.solver.linearMethod = stripOptionalQuotes(JsonMini::readValue(solverStr, "linear_method"));
        
        double dVal = 0.0;
        if (parseDouble(JsonMini::readValue(solverStr, "tolerance"), dVal)) {
            controls.solver.tolerance = dVal;
        }
        if (parseDouble(JsonMini::readValue(solverStr, "max_iterations"), dVal)) {
            controls.solver.maxIterations = static_cast<int>(dVal);
        }
        if (parseDouble(JsonMini::readValue(solverStr, "relaxation_factor"), dVal)) {
            controls.solver.relaxationFactor = dVal;
        }
        
        controls.solver.backend = stripOptionalQuotes(JsonMini::readValue(solverStr, "backend"));
        
        int iVal = 0;
        if (parseInt(JsonMini::readValue(solverStr, "cpu_threads"), iVal)) {
            controls.solver.cpuThreads = iVal;
        }
    }
    
    const std::string timeStr = JsonMini::readValue(controlsStr, "time");
    if (!timeStr.empty()) {
        double dVal = 0.0;
        if (parseDouble(JsonMini::readValue(timeStr, "start"), dVal)) controls.time.start = dVal;
        if (parseDouble(JsonMini::readValue(timeStr, "end"), dVal)) controls.time.end = dVal;
        if (parseDouble(JsonMini::readValue(timeStr, "time_step"), dVal)) controls.time.timeStep = dVal;
        if (parseDouble(JsonMini::readValue(timeStr, "output_interval"), dVal)) controls.time.outputInterval = dVal;
    }
    
    return controls;
}

// ============================================================================
// Component Creation Helper
// ============================================================================

std::unique_ptr<Component> createComponentFromDef(const ComponentDef& def, 
                                                   const MaterialProperties& fluid) {
    const std::string typeKey = toLowerCopy(def.type);
    const std::string compName = def.name.empty() ? (typeKey + "_" + def.id) : def.name;
    
    // Helper to get parameter value
    auto getParam = [&](const std::string& key, double defaultVal) -> double {
        auto it = def.parameters.find(key);
        if (it != def.parameters.end()) {
            double v = 0.0;
            if (parseDouble(it->second, v)) return v;
        }
        return defaultVal;
    };
    
    // Helper to get bool parameter
    auto getBoolParam = [&](const std::string& key, bool defaultVal) -> bool {
        auto it = def.parameters.find(key);
        if (it != def.parameters.end()) {
            bool v = false;
            if (parseBool(it->second, v)) return v;
        }
        return defaultVal;
    };
    
    if (typeKey == "pipe") {
        double length = def.geometry.length.value > 0 ? def.geometry.length.value : getParam("length", 10.0);
        double diameter = def.geometry.diameter.value > 0 ? def.geometry.diameter.value : getParam("diameter", 0.05);
        double roughness = def.geometry.roughness.value > 0 ? def.geometry.roughness.value : getParam("roughness", 0.0001);
        return std::make_unique<Components::Pipe>(compName, length, diameter, roughness, fluid);
    }
    
    if (typeKey == "valve") {
        double kv = getParam("kv", 5.0);
        double open = getParam("open", 1.0);
        return std::make_unique<Components::Valve>(compName, kv, open, fluid);
    }
    
    if (typeKey == "bend") {
        double k = getParam("lossCoefficient", 0.5);
        double dia = getParam("diameter", 0.05);
        double angle = getParam("angleDeg", 90.0);
        return std::make_unique<Components::Bend>(compName, k, dia, fluid, angle);
    }
    
    if (typeKey == "tjoint") {
        double k = getParam("lossCoefficient", 1.0);
        double dia = getParam("diameter", 0.05);
        double branch = getParam("branchLossCoefficient", 1.0);
        return std::make_unique<Components::TJoint>(compName, k, dia, fluid, branch);
    }
    
    if (typeKey == "yjoint") {
        double k = getParam("lossCoefficient", 0.7);
        double dia = getParam("diameter", 0.05);
        double branch = getParam("branchLossCoefficient", 0.7);
        return std::make_unique<Components::YJoint>(compName, k, dia, fluid, branch);
    }
    
    if (typeKey == "centrifugal_pump") {
        double ratedFlow = getParam("ratedFlow", 0.02);
        double shutoff = getParam("shutoffPressure", 150000);
        double rated = getParam("ratedPressure", 100000);
        bool allowReverse = getBoolParam("allowReverse", false);
        return std::make_unique<Components::CentrifugalPump>(compName, ratedFlow, shutoff, rated, fluid, allowReverse);
    }
    
    if (typeKey == "gate_valve") {
        double cv = getParam("flowCoefficient", 5.0);
        double open = getParam("open", 1.0);
        return std::make_unique<Components::GateValve>(compName, cv, open, fluid);
    }
    
    if (typeKey == "ball_valve") {
        double cv = getParam("flowCoefficient", 5.0);
        double open = getParam("open", 1.0);
        return std::make_unique<Components::BallValve>(compName, cv, open, fluid);
    }
    
    if (typeKey == "butterfly_valve") {
        double cv = getParam("flowCoefficient", 5.0);
        double open = getParam("open", 1.0);
        return std::make_unique<Components::ButterflyValve>(compName, cv, open, fluid);
    }
    
    if (typeKey == "check_valve") {
        double cv = getParam("flowCoefficient", 5.0);
        double open = getParam("open", 1.0);
        return std::make_unique<Components::CheckValve>(compName, cv, open, fluid);
    }
    
    if (typeKey == "control_valve") {
        double cv = getParam("flowCoefficient", 5.0);
        double open = getParam("open", 1.0);
        return std::make_unique<Components::ControlValve>(compName, cv, open, fluid);
    }
    
    if (typeKey == "heat_exchanger") {
        double ua = getParam("ua", 1000.0);
        double area = getParam("flowArea", 0.01);
        double length = getParam("length", 1.0);
        double pitch = getParam("pitch", 10.0);
        return std::make_unique<Components::HeatExchanger>(compName, ua, area, length, pitch);
    }
    
    if (typeKey == "radiator") {
        double ua = getParam("ua", 1000.0);
        double area = getParam("flowArea", 0.01);
        double length = getParam("length", 1.0);
        double pitch = getParam("pitch", 10.0);
        return std::make_unique<Components::Radiator>(compName, ua, area, length, pitch);
    }
    
    if (typeKey == "channel") {
        double length = getParam("length", 1.0);
        double width = getParam("width", 0.01);
        double height = getParam("height", 0.01);
        double roughness = getParam("roughness", 0.0);
        return std::make_unique<Components::Channel>(compName, length, width, height, roughness, fluid);
    }
    
    if (typeKey == "liquid_filter") {
        double k = getParam("lossCoefficient", 1.5);
        double dia = getParam("diameter", 0.05);
        return std::make_unique<Components::LiquidFilter>(compName, k, dia, fluid);
    }
    
    if (typeKey == "gas_filter") {
        double k = getParam("lossCoefficient", 1.5);
        double dia = getParam("diameter", 0.05);
        return std::make_unique<Components::GasFilter>(compName, k, dia, fluid);
    }
    
    if (typeKey == "expander") {
        double diaIn = getParam("inletDiameter", 0.05);
        double diaOut = getParam("outletDiameter", 0.10);
        return std::make_unique<Components::Expander>(compName, diaIn, diaOut, fluid);
    }
    
    if (typeKey == "contraction") {
        double diaIn = getParam("inletDiameter", 0.10);
        double diaOut = getParam("outletDiameter", 0.05);
        return std::make_unique<Components::Contraction>(compName, diaIn, diaOut, fluid);
    }
    
    // Use component registry for other types
    auto comp = ComponentRegistry::instance().create(typeKey);
    if (comp) {
        return comp;
    }
    
    std::cerr << "Warning: Unknown component type '" << typeKey << "'\n";
    return nullptr;
}

// ============================================================================
// Node ID Mapping Helper
// ============================================================================

int nodeIdStringToInt(const std::string& id) {
    // Try direct integer conversion first
    int intId = -1;
    if (parseInt(id, intId) && intId >= 0) {
        return intId;
    }
    
    // Hash-based fallback for string IDs
    std::hash<std::string> hasher;
    return static_cast<int>(hasher(id) % 100000);
}

} // anonymous namespace

// ============================================================================
// Format Detection
// ============================================================================

FormatVersion detectFormat(const std::string& jsonContent) {
    // Check for v2 format indicators
    if (jsonContent.find("\"format_version\"") != std::string::npos) {
        const std::string version = JsonMini::readValue(jsonContent, "format_version");
        if (version.find("2.0") != std::string::npos) {
            return FormatVersion::V2_0;
        }
    }
    
    // Check for v2 top-level keys
    if (jsonContent.find("\"model\"") != std::string::npos &&
        jsonContent.find("\"controls\"") != std::string::npos) {
        return FormatVersion::V2_0;
    }
    
    return FormatVersion::V1_Legacy;
}

// ============================================================================
// V2 Input Loading
// ============================================================================

bool loadInputV2(const std::string& filePath, FluidNetworkSolver& solver, InputConfig* config) {
    std::ifstream ifs(filePath.c_str());
    if (!ifs.is_open()) {
        std::cerr << "Cannot open input file: " << filePath << std::endl;
        return false;
    }
    
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    return loadInputV2FromString(buffer.str(), solver, config);
}

bool loadInputV2FromString(const std::string& jsonContent, FluidNetworkSolver& solver, InputConfig* config) {
    solver.clearNetwork();
    
    InputConfig localConfig;
    InputConfig& cfg = config ? *config : localConfig;
    
    // Parse meta
    const std::string metaStr = JsonMini::readValue(jsonContent, "meta");
    if (!metaStr.empty()) {
        cfg.meta = parseMeta(metaStr);
    }
    
    // Parse model
    const std::string modelStr = JsonMini::readValue(jsonContent, "model");
    if (!modelStr.empty()) {
        cfg.model.dimension = stripOptionalQuotes(JsonMini::readValue(modelStr, "dimension"));
        cfg.model.topologyType = stripOptionalQuotes(JsonMini::readValue(modelStr, "topology_type"));
        
        // Parse nodes
        const std::string nodesArr = JsonMini::readValue(modelStr, "nodes");
        for (const std::string& nodeStr : JsonMini::readArrayItems(nodesArr)) {
            cfg.model.nodes.push_back(parseNodeDef(nodeStr));
        }
        
        // Parse materials
        const std::string matsArr = JsonMini::readValue(modelStr, "materials");
        for (const std::string& matStr : JsonMini::readArrayItems(matsArr)) {
            cfg.model.materials.push_back(parseMaterialDef(matStr));
        }
        
        // Parse components
        const std::string compsArr = JsonMini::readValue(modelStr, "components");
        for (const std::string& compStr : JsonMini::readArrayItems(compsArr)) {
            cfg.model.components.push_back(parseComponentDef(compStr));
        }
        
        // Parse boundary templates
        const std::string bcArr = JsonMini::readValue(modelStr, "boundary_templates");
        for (const std::string& bcStr : JsonMini::readArrayItems(bcArr)) {
            cfg.model.boundaryTemplates.push_back(parseBoundaryDef(bcStr));
        }
    }
    
    // Parse physics
    const std::string physicsStr = JsonMini::readValue(jsonContent, "physics");
    if (!physicsStr.empty()) {
        cfg.physics = parsePhysicsDef(physicsStr);
    }
    
    // Parse controls
    const std::string controlsStr = JsonMini::readValue(jsonContent, "controls");
    if (!controlsStr.empty()) {
        cfg.controls = parseControlsDef(controlsStr);
    }
    
    // Parse frames
    const std::string framesArr = JsonMini::readValue(jsonContent, "frames");
    for (const std::string& frameStr : JsonMini::readArrayItems(framesArr)) {
        cfg.frames.push_back(parseFrameDef(frameStr));
    }
    
    // Build solver network from parsed config
    
    // 1. Determine working fluid
    MaterialProperties workingFluid;
    if (!cfg.model.materials.empty()) {
        workingFluid = materialDefToProperties(cfg.model.materials[0]);
    }
    solver.setWorkingFluid(workingFluid);
    
    // 2. Create nodes
    std::map<std::string, int> nodeIdMapping;
    for (const auto& nodeDef : cfg.model.nodes) {
        const int intId = nodeIdStringToInt(nodeDef.id);
        nodeIdMapping[nodeDef.id] = intId;
        
        double pressure = nodeDef.variables.pressure.value > 0 ? nodeDef.variables.pressure.value : 101325.0;
        double temperature = nodeDef.variables.temperature.value > 0 ? nodeDef.variables.temperature.value : 293.15;
        
        auto node = std::make_unique<Node>(intId, nodeDef.name.empty() ? nodeDef.id : nodeDef.name, pressure, temperature);
        
        if (nodeDef.constraints.pressureFixed) {
            node->isPressureFixed = true;
            node->fixedPressure = pressure;
            solver.addBoundaryCondition(BoundaryCondition(intId, BoundaryConditionType::Pressure, pressure));
        }
        
        solver.addNode(std::move(node));
    }
    
    // 3. Create and connect components
    for (const auto& compDef : cfg.model.components) {
        auto comp = createComponentFromDef(compDef, workingFluid);
        if (!comp) continue;
        
        const int compIndex = static_cast<int>(solver.getComponents().size());
        solver.addComponent(std::move(comp));
        
        // Connect ports
        if (compDef.ports.size() >= 2) {
            const int fromId = nodeIdMapping.count(compDef.ports[0].nodeRef) ? 
                               nodeIdMapping[compDef.ports[0].nodeRef] : nodeIdStringToInt(compDef.ports[0].nodeRef);
            const int toId = nodeIdMapping.count(compDef.ports[1].nodeRef) ? 
                             nodeIdMapping[compDef.ports[1].nodeRef] : nodeIdStringToInt(compDef.ports[1].nodeRef);
            
            if (fromId >= 0 && toId >= 0) {
                solver.connectComponent(compIndex, fromId, toId);
            }
        }
    }
    
    // 4. Apply boundary templates
    for (const auto& bcDef : cfg.model.boundaryTemplates) {
        if (bcDef.target.kind == "node") {
            const int nodeId = nodeIdMapping.count(bcDef.target.id) ? 
                              nodeIdMapping[bcDef.target.id] : nodeIdStringToInt(bcDef.target.id);
            
            if (bcDef.variable == "pressure" && bcDef.type == "dirichlet") {
                solver.addBoundaryCondition(BoundaryCondition(nodeId, BoundaryConditionType::Pressure, bcDef.value.value));
                Node* node = solver.getNode(nodeId);
                if (node) {
                    node->isPressureFixed = true;
                    node->fixedPressure = bcDef.value.value;
                    node->pressure = bcDef.value.value;
                }
            } else if (bcDef.variable == "temperature") {
                solver.addBoundaryCondition(BoundaryCondition(nodeId, BoundaryConditionType::Temperature, bcDef.value.value));
            }
        }
    }
    
    // 5. Configure solver parameters
    const auto& solverCtrl = cfg.controls.solver;
    solver.setCalculationParameters(cfg.controls.time.timeStep,
                                    solverCtrl.tolerance,
                                    solverCtrl.maxIterations,
                                    solverCtrl.relaxationFactor);
    
    if (!solverCtrl.steadyMethod.empty()) {
        solver.setSteadySolverTypeByName(solverCtrl.steadyMethod);
    }
    
    if (solverCtrl.cpuThreads > 0) {
        solver.setCpuThreads(solverCtrl.cpuThreads);
    }
    
    if (!solverCtrl.backend.empty()) {
        solver.setComputeBackendByName(solverCtrl.backend);
    }
    
    // 6. Apply initial frame controls if present
    if (!cfg.frames.empty()) {
        for (const auto& update : cfg.frames[0].controlUpdates) {
            if (update.target.kind == "component") {
                // Find component and apply control
                for (std::size_t i = 0; i < solver.getComponents().size(); ++i) {
                    Component* comp = solver.getComponent(static_cast<int>(i));
                    if (comp && comp->getName() == update.target.id) {
                        // Apply control update (e.g., valve opening)
                        if (update.variable == "opening") {
                            if (auto* valve = dynamic_cast<Valve*>(comp)) {
                                valve->setOpening(update.value);
                            }
                        }
                    }
                }
            }
        }
    }
    
    return true;
}

// ============================================================================
// V2 Output Generation
// ============================================================================

std::string getCurrentTimestamp() {
    auto now = std::chrono::system_clock::now();
    auto time = std::chrono::system_clock::to_time_t(now);
    std::stringstream ss;
    ss << std::put_time(std::gmtime(&time), "%Y-%m-%dT%H:%M:%SZ");
    return ss.str();
}

bool dumpOutputV2(const FluidNetworkSolver& solver, const OutputConfig& extraConfig, const std::string& filePath) {
    std::ofstream out(filePath.c_str());
    if (!out.is_open()) {
        std::cerr << "Cannot open output file: " << filePath << std::endl;
        return false;
    }
    
    out << std::setprecision(12);
    out << "{\n";
    
    // meta
    out << "  \"meta\": {\n";
    out << "    \"format_name\": \"FluidSystemOutput\",\n";
    out << "    \"format_version\": \"2.0.0\",\n";
    out << "    \"generated_at\": \"" << getCurrentTimestamp() << "\",\n";
    out << "    \"case_id\": \"" << extraConfig.meta.caseId << "\"\n";
    out << "  },\n";
    
    // execution
    out << "  \"execution\": {\n";
    out << "    \"success\": true,\n";
    out << "    \"analysis_type\": \"" << extraConfig.controls.analysisType << "\",\n";
    out << "    \"elapsed_seconds\": " << extraConfig.execution.elapsedSeconds << ",\n";
    out << "    \"backend\": \"" << extraConfig.execution.backend << "\"\n";
    out << "  },\n";
    
    // summary
    out << "  \"summary\": {\n";
    out << "    \"converged\": " << (solver.getLastIterationHistory().converged ? "true" : "false") << ",\n";
    out << "    \"final_iteration_count\": " << solver.getLastIterationHistory().totalIterations << ",\n";
    out << "    \"max_residual\": ";
    const auto& residuals = solver.getLastIterationHistory().residuals;
    if (!residuals.empty()) {
        out << residuals.back();
    } else {
        out << "0.0";
    }
    out << "\n  },\n";
    
    // results.final_state
    out << "  \"results\": {\n";
    out << "    \"final_state\": {\n";
    out << "      \"time\": " << solver.getCurrentTime() << ",\n";
    
    // nodes
    out << "      \"nodes\": {\n";
    const auto& nodes = solver.getNodes();
    for (std::size_t i = 0; i < nodes.size(); ++i) {
        const Node* node = nodes[i].get();
        if (!node) continue;
        
        out << "        \"" << node->id << "\": {\n";
        out << "          \"pressure\": { \"value\": " << node->pressure << ", \"unit\": \"Pa\" },\n";
        out << "          \"temperature\": { \"value\": " << node->temperature << ", \"unit\": \"K\" }\n";
        out << "        }" << (i + 1 < nodes.size() ? "," : "") << "\n";
    }
    out << "      },\n";
    
    // components
    out << "      \"components\": {\n";
    const auto& conns = solver.getComponentConnections();
    const auto& components = solver.getComponents();
    const auto& flows = solver.getLatestConnectionFlows();
    
    for (std::size_t i = 0; i < conns.size(); ++i) {
        const auto& conn = conns[i];
        const Component* comp = nullptr;
        if (conn.componentIndex >= 0 && conn.componentIndex < static_cast<int>(components.size())) {
            comp = components[conn.componentIndex].get();
        }
        if (!comp) continue;
        
        const double flow = (i < flows.size()) ? flows[i] : 0.0;
        const Node* fromNode = solver.getNode(conn.fromNodeId);
        const Node* toNode = solver.getNode(conn.toNodeId);
        const double dp = (fromNode && toNode) ? (fromNode->pressure - toNode->pressure) : 0.0;
        
        out << "        \"" << comp->getName() << "\": {\n";
        out << "          \"mass_flow_rate\": { \"value\": " << flow << ", \"unit\": \"kg/s\" },\n";
        out << "          \"pressure_drop\": { \"value\": " << dp << ", \"unit\": \"Pa\" }\n";
        out << "        }" << (i + 1 < conns.size() ? "," : "") << "\n";
    }
    out << "      }\n";
    out << "    }\n";
    out << "  },\n";
    
    // restart
    out << "  \"restart\": {\n";
    out << "    \"available\": true,\n";
    out << "    \"time\": " << solver.getCurrentTime() << "\n";
    out << "  }\n";
    
    out << "}\n";
    return true;
}

bool dumpOutputV2SteadyState(const FluidNetworkSolver& solver, const std::string& filePath) {
    OutputConfig config;
    config.meta.caseId = "steady_state";
    config.controls.analysisType = "steady";
    config.execution.backend = "cpu";
    return dumpOutputV2(solver, config, filePath);
}

bool dumpOutputV2Transient(const FluidNetworkSolver& solver, const std::string& filePath) {
    OutputConfig config;
    config.meta.caseId = "transient";
    config.controls.analysisType = "transient";
    config.execution.backend = "cpu";
    return dumpOutputV2(solver, config, filePath);
}

// ============================================================================
// Restart Support
// ============================================================================

bool exportRestartFile(const OutputConfig& output, const std::string& filePath) {
    std::ofstream out(filePath.c_str());
    if (!out.is_open()) {
        std::cerr << "Cannot open restart file: " << filePath << std::endl;
        return false;
    }
    
    out << "{\n";
    out << "  \"meta\": {\n";
    out << "    \"format_name\": \"FluidSystemInput\",\n";
    out << "    \"format_version\": \"2.0.0\",\n";
    out << "    \"description\": \"Restart file from previous simulation\"\n";
    out << "  },\n";
    out << "  \"model\": {\n";
    out << "    \"initial_state\": {\n";
    out << "      \"time\": " << output.summary.finalTime << "\n";
    out << "    }\n";
    out << "  }\n";
    out << "}\n";
    return true;
}

bool loadRestartFile(const std::string& filePath, InitialStateDef& initialState, double& time) {
    std::ifstream ifs(filePath.c_str());
    if (!ifs.is_open()) {
        return false;
    }
    
    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string content = buffer.str();
    
    // Parse time
    const std::string modelStr = JsonMini::readValue(content, "model");
    if (!modelStr.empty()) {
        const std::string initStateStr = JsonMini::readValue(modelStr, "initial_state");
        if (!initStateStr.empty()) {
            parseDouble(JsonMini::readValue(initStateStr, "time"), time);
        }
    }
    
    return true;
}

// ============================================================================
// Utility Functions
// ============================================================================

MaterialProperties materialDefToProperties(const MaterialDef& def) {
    return MaterialProperties(
        def.name.empty() ? "Default" : def.name,
        def.fluidType == "compressible" ? FluidType::Compressible : FluidType::Incompressible,
        def.properties.density.value > 0 ? def.properties.density.value : 998.0,
        def.properties.dynamicViscosity.value > 0 ? def.properties.dynamicViscosity.value : 0.001,
        1e-6,
        def.properties.specificHeat.value > 0 ? def.properties.specificHeat.value : 4180.0,
        def.properties.thermalConductivity.value > 0 ? def.properties.thermalConductivity.value : 0.6
    );
}

MaterialDef propertiesToMaterialDef(const MaterialProperties& props, const std::string& id) {
    MaterialDef def;
    def.id = id;
    def.name = props.name;
    def.fluidType = props.fluidType == FluidType::Compressible ? "compressible" : "incompressible";
    def.properties.density = ValueUnit(props.density, "kg/m3");
    def.properties.dynamicViscosity = ValueUnit(props.dynamicViscosity, "Pa*s");
    def.properties.specificHeat = ValueUnit(props.specificHeat, "J/(kg*K)");
    def.properties.thermalConductivity = ValueUnit(props.thermalConductivity, "W/(m*K)");
    return def;
}

} // namespace NetworkIOv2
