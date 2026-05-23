#ifndef JSON_MINI_H
#define JSON_MINI_H

#include <string>
#include <vector>
#include <sstream>
#include <algorithm>

namespace JsonMini {

inline std::string trim(const std::string& str) {
    const std::size_t start = str.find_first_not_of(" \t\n\r");
    if (start == std::string::npos) return "";
    const std::size_t end = str.find_last_not_of(" \t\n\r");
    return str.substr(start, end - start + 1);
}

inline std::string readValue(const std::string& content, const std::string& key) {
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

inline std::vector<std::string> readArrayItems(const std::string& arrayStr) {
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

inline std::vector<std::string> readObjectKeys(const std::string& objectStr) {
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

inline std::vector<int> readIntArray(const std::string& arrayStr) {
    std::vector<int> values;
    for (const std::string& item : readArrayItems(arrayStr)) {
        try {
            values.push_back(std::stoi(trim(item)));
        } catch (...) {}
    }
    return values;
}

inline std::vector<std::string> readStringArray(const std::string& arrayStr) {
    std::vector<std::string> values;
    for (const std::string& item : readArrayItems(arrayStr)) {
        std::string val = trim(item);
        if (val.size() >= 2 && val.front() == '"' && val.back() == '"') {
            val = val.substr(1, val.size() - 2);
        }
        values.push_back(val);
    }
    return values;
}

} // namespace JsonMini

#endif // JSON_MINI_H
