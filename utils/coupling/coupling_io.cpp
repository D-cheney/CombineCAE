#include "coupling_io.h"

#include "core/components.h"
#include "core/coupling_types.h"
#include "solver/fluid_solver/solver_context_utils.h"

#include <algorithm>
#include <cctype>
#include <fstream>
#include <iomanip>
#include <map>
#include <sstream>
#include <vector>

namespace {

std::string trim(const std::string& str) {
    const std::size_t start = str.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) {
        return "";
    }
    const std::size_t end = str.find_last_not_of(" \t\n\r");
    return str.substr(start, end - start + 1);
}

std::string toLowerCopy(const std::string& text) {
    std::string out = text;
    std::transform(out.begin(), out.end(), out.begin(),
                   [](unsigned char c) { return static_cast<char>(std::tolower(c)); });
    return out;
}

bool parseDouble(const std::string& text, double& value) {
    try {
        value = std::stod(text);
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

std::string readValue(const std::string& content, const std::string& key) {
    std::size_t pos = content.find("\"" + key + "\"");
    if (pos == std::string::npos) {
        return "";
    }

    pos = content.find(':', pos);
    if (pos == std::string::npos) {
        return "";
    }

    ++pos;
    while (pos < content.size() && std::isspace(static_cast<unsigned char>(content[pos]))) {
        ++pos;
    }
    if (pos >= content.size()) {
        return "";
    }

    if (content[pos] == '"') {
        ++pos;
        const std::size_t end = content.find('"', pos);
        if (end == std::string::npos) {
            return "";
        }
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
            if (c == '"' && content[end - 1] != '\\') {
                inString = !inString;
            }
            if (!inString) {
                if (c == open) {
                    ++depth;
                } else if (c == close) {
                    --depth;
                }
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
            if (c == '{' || c == '[') {
                ++depth;
            } else if (c == '}' || c == ']') {
                --depth;
            } else if (c == ',' && depth == 0) {
                const std::string trimmed = trim(item);
                if (!trimmed.empty()) {
                    items.push_back(trimmed);
                }
                item.clear();
                continue;
            }
        }
        item.push_back(c);
    }

    const std::string trimmed = trim(item);
    if (!trimmed.empty()) {
        items.push_back(trimmed);
    }
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
                    if (objectStr[keyEnd] == '"' && objectStr[keyEnd - 1] != '\\') {
                        break;
                    }
                    ++keyEnd;
                }
                if (keyEnd < right) {
                    std::size_t pos = keyEnd + 1;
                    while (pos < right &&
                           std::isspace(static_cast<unsigned char>(objectStr[pos]))) {
                        ++pos;
                    }
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
            if (c == '{' || c == '[') {
                ++depth;
            } else if (c == '}' || c == ']') {
                --depth;
            }
        }
        ++i;
    }
    return keys;
}

struct CouplingInputRecord {
    CouplingInput input;
};

bool parseCouplingInputs(const std::string& content,
                         std::map<std::string, CouplingInputRecord>& out) {
    out.clear();
    const std::string componentsObj = readValue(content, "components");
    if (componentsObj.empty()) {
        return false;
    }

    if (componentsObj.find('{') != std::string::npos &&
        componentsObj.find('[') == std::string::npos) {
        const std::vector<std::string> keys = readObjectKeys(componentsObj);
        for (const std::string& key : keys) {
            const std::string payload = readValue(componentsObj, key);
            if (payload.empty()) {
                continue;
            }
            CouplingInputRecord record;
            const std::string mode = toLowerCopy(stripOptionalQuotes(readValue(payload, "mode")));
            if (!mode.empty()) {
                record.input.hasMode = true;
                record.input.mode = couplingModeFromString(mode);
            }
            double value = 0.0;
            if (parseDouble(readValue(payload, "resistance"), value)) {
                record.input.hasResistance = true;
                record.input.resistance = value;
            }
            if (parseDouble(readValue(payload, "pressureDrop"), value)) {
                record.input.hasPressureDrop = true;
                record.input.pressureDrop = value;
            }
            if (parseDouble(readValue(payload, "heatTransfer"), value)) {
                record.input.hasHeatTransfer = true;
                record.input.heatTransfer = value;
            }
            out[key] = record;
        }
        return !out.empty();
    }

    for (const std::string& item : readArrayItems(componentsObj)) {
        const std::string idRaw = readValue(item, "id");
        const std::string id = stripOptionalQuotes(idRaw);
        if (id.empty()) {
            continue;
        }
        CouplingInputRecord record;
        const std::string mode = toLowerCopy(stripOptionalQuotes(readValue(item, "mode")));
        if (!mode.empty()) {
            record.input.hasMode = true;
            record.input.mode = couplingModeFromString(mode);
        }
        double value = 0.0;
        if (parseDouble(readValue(item, "resistance"), value)) {
            record.input.hasResistance = true;
            record.input.resistance = value;
        }
        if (parseDouble(readValue(item, "pressureDrop"), value)) {
            record.input.hasPressureDrop = true;
            record.input.pressureDrop = value;
        }
        if (parseDouble(readValue(item, "heatTransfer"), value)) {
            record.input.hasHeatTransfer = true;
            record.input.heatTransfer = value;
        }
        out[id] = record;
    }

    return !out.empty();
}

} // namespace

namespace CouplingIO {

bool applyCouplingInputs(SolverContext& context,
                         const CouplingConfig& config,
                         double time,
                         int iteration) {
    (void)time;
    (void)iteration;
    if (!config.enabled || config.inputFile.empty()) {
        return false;
    }

    std::ifstream ifs(config.inputFile.c_str());
    if (!ifs.is_open()) {
        return false;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string content = buffer.str();

    std::map<std::string, CouplingInputRecord> inputs;
    if (!parseCouplingInputs(content, inputs)) {
        return false;
    }

    bool applied = false;
    for (auto& compPtr : context.components) {
        if (!compPtr) {
            continue;
        }
        auto* coupling = dynamic_cast<CouplingLink*>(compPtr.get());
        if (coupling == nullptr) {
            continue;
        }

        const auto it = inputs.find(coupling->getCouplingId());
        if (it == inputs.end()) {
            continue;
        }
        coupling->applyCouplingInput(it->second.input);
        applied = true;
    }

    return applied;
}

bool writeCouplingOutputs(const SolverContext& context,
                          const CouplingConfig& config,
                          double time,
                          int iteration) {
    if (!config.enabled || config.outputFile.empty()) {
        return false;
    }

    std::ofstream out(config.outputFile.c_str());
    if (!out.is_open()) {
        return false;
    }

    out << "{\n";
    out << "  \"version\": 1,\n";
    out << "  \"time\": " << std::setprecision(12) << time << ",\n";
    out << "  \"iteration\": " << iteration << ",\n";
    out << "  \"components\": [\n";

    bool wroteAny = false;
    for (std::size_t i = 0; i < context.componentConnections.size(); ++i) {
        const ComponentConnection& conn = context.componentConnections[i];
        if (conn.componentIndex < 0 ||
            conn.componentIndex >= static_cast<int>(context.components.size())) {
            continue;
        }

        Component* component =
            context.components[static_cast<std::size_t>(conn.componentIndex)].get();
        auto* coupling = dynamic_cast<CouplingLink*>(component);
        if (coupling == nullptr) {
            continue;
        }

        Node* fromNode = findNodeById(const_cast<SolverContext&>(context), conn.fromNodeId);
        Node* toNode = findNodeById(const_cast<SolverContext&>(context), conn.toNodeId);
        if (fromNode == nullptr || toNode == nullptr) {
            continue;
        }

        const double flow = estimateConnectionFlow(const_cast<SolverContext&>(context), conn);
        std::vector<double> flows(1, flow);
        std::vector<Node*> nodes;
        nodes.push_back(fromNode);
        nodes.push_back(toNode);
        const double pressureDrop =
            component->calculatePressureDropWithPorts(flows,
                                                      nodes,
                                                      conn.fromPortIndex,
                                                      conn.toPortIndex);
        double resistance = 0.0;
        if (std::abs(flow) > 1e-12) {
            resistance = pressureDrop / flow;
        } else {
            resistance =
                estimateConnectionResistance(const_cast<SolverContext&>(context), conn);
        }
        const double heatTransfer = component->calculateHeatTransfer(nodes);

        out << "    {\n";
        out << "      \"id\": \"" << coupling->getCouplingId() << "\",\n";
        out << "      \"component_index\": " << conn.componentIndex << ",\n";
        out << "      \"connection_index\": " << i << ",\n";
        out << "      \"type\": \"" << component->getType() << "\",\n";
        out << "      \"name\": \"" << component->getName() << "\",\n";
        out << "      \"from_node\": " << conn.fromNodeId << ",\n";
        out << "      \"to_node\": " << conn.toNodeId << ",\n";
        out << "      \"from_port\": " << conn.fromPortIndex << ",\n";
        out << "      \"to_port\": " << conn.toPortIndex << ",\n";
        out << "      \"flow\": " << std::setprecision(12) << flow << ",\n";
        out << "      \"pressureDrop\": " << std::setprecision(12) << pressureDrop << ",\n";
        out << "      \"resistance\": " << std::setprecision(12) << resistance << ",\n";
        out << "      \"heatTransfer\": " << std::setprecision(12) << heatTransfer << ",\n";
        out << "      \"inlet\": {\n";
        out << "        \"pressure\": " << std::setprecision(12) << fromNode->pressure << ",\n";
        out << "        \"temperature\": " << std::setprecision(8) << fromNode->temperature << "\n";
        out << "      },\n";
        out << "      \"outlet\": {\n";
        out << "        \"pressure\": " << std::setprecision(12) << toNode->pressure << ",\n";
        out << "        \"temperature\": " << std::setprecision(8) << toNode->temperature << "\n";
        out << "      },\n";
        out << "      \"inputs\": {\n";
        out << "        \"hydraulicMode\": \"" << couplingModeToString(coupling->getHydraulicMode())
            << "\",\n";
        out << "        \"resistance\": " << std::setprecision(12)
            << coupling->getAppliedResistance() << ",\n";
        out << "        \"pressureDrop\": " << std::setprecision(12)
            << coupling->getAppliedPressureDrop() << ",\n";
        out << "        \"heatTransfer\": " << std::setprecision(12)
            << coupling->getAppliedHeatTransfer() << "\n";
        out << "      }\n";
        out << "    },\n";
        wroteAny = true;
    }

    if (wroteAny) {
        out.seekp(-2, std::ios_base::cur);
        out << "\n";
    }

    out << "  ]\n";
    out << "}\n";
    return true;
}

} // namespace CouplingIO
