/*
 * OpenCNC Configuration Parser
 * 
 * TOML/YAML configuration file parser for machine setup.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef CNC_CONFIG_H
#define CNC_CONFIG_H

#include <string>
#include <vector>
#include <map>
#include <variant>
#include <optional>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace cnc {
namespace config {

// ============================================================================
// Configuration Value Types
// ============================================================================

class ConfigValue;

using ConfigArray = std::vector<ConfigValue>;
using ConfigTable = std::map<std::string, ConfigValue>;

class ConfigValue {
public:
    using Variant = std::variant<
        std::monostate,
        bool,
        int64_t,
        double,
        std::string,
        ConfigArray,
        ConfigTable
    >;
    
    ConfigValue() : value_(std::monostate{}) {}
    ConfigValue(bool v) : value_(v) {}
    ConfigValue(int v) : value_(static_cast<int64_t>(v)) {}
    ConfigValue(int64_t v) : value_(v) {}
    ConfigValue(double v) : value_(v) {}
    ConfigValue(const char* v) : value_(std::string(v)) {}
    ConfigValue(std::string v) : value_(std::move(v)) {}
    ConfigValue(ConfigArray v) : value_(std::move(v)) {}
    ConfigValue(ConfigTable v) : value_(std::move(v)) {}
    
    bool isNull() const { return std::holds_alternative<std::monostate>(value_); }
    bool isBool() const { return std::holds_alternative<bool>(value_); }
    bool isInt() const { return std::holds_alternative<int64_t>(value_); }
    bool isDouble() const { return std::holds_alternative<double>(value_); }
    bool isString() const { return std::holds_alternative<std::string>(value_); }
    bool isArray() const { return std::holds_alternative<ConfigArray>(value_); }
    bool isTable() const { return std::holds_alternative<ConfigTable>(value_); }
    
    bool asBool() const { return std::get<bool>(value_); }
    int64_t asInt() const { return std::get<int64_t>(value_); }
    double asDouble() const {
        if (isInt()) return static_cast<double>(asInt());
        return std::get<double>(value_);
    }
    const std::string& asString() const { return std::get<std::string>(value_); }
    const ConfigArray& asArray() const { return std::get<ConfigArray>(value_); }
    const ConfigTable& asTable() const { return std::get<ConfigTable>(value_); }
    
    ConfigArray& asArray() { return std::get<ConfigArray>(value_); }
    ConfigTable& asTable() { return std::get<ConfigTable>(value_); }
    
    ConfigValue& operator[](const std::string& key) {
        return asTable()[key];
    }
    
    const ConfigValue& operator[](const std::string& key) const {
        return asTable().at(key);
    }
    
    ConfigValue& operator[](size_t index) {
        return asArray()[index];
    }
    
    const ConfigValue& operator[](size_t index) const {
        return asArray()[index];
    }
    
    bool contains(const std::string& key) const {
        if (!isTable()) return false;
        return asTable().find(key) != asTable().end();
    }
    
    template<typename T>
    T get(const T& defaultValue = T{}) const {
        if constexpr (std::is_same_v<T, bool>) {
            return isBool() ? asBool() : defaultValue;
        } else if constexpr (std::is_integral_v<T>) {
            return isInt() ? static_cast<T>(asInt()) : defaultValue;
        } else if constexpr (std::is_floating_point_v<T>) {
            return (isDouble() || isInt()) ? static_cast<T>(asDouble()) : defaultValue;
        } else if constexpr (std::is_same_v<T, std::string>) {
            return isString() ? asString() : defaultValue;
        } else {
            return defaultValue;
        }
    }
    
private:
    Variant value_;
};

// ============================================================================
// TOML Parser
// ============================================================================

class TomlParser {
public:
    ConfigTable parse(const std::string& content) {
        lines_ = splitLines(content);
        lineIndex_ = 0;
        root_ = ConfigTable{};
        currentTable_ = &root_;
        
        while (lineIndex_ < lines_.size()) {
            parseLine();
            lineIndex_++;
        }
        
        return root_;
    }
    
    ConfigTable parseFile(const std::string& filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Cannot open config file: " + filename);
        }
        
        std::stringstream buffer;
        buffer << file.rdbuf();
        return parse(buffer.str());
    }
    
private:
    std::vector<std::string> lines_;
    size_t lineIndex_ = 0;
    ConfigTable root_;
    ConfigTable* currentTable_ = nullptr;
    
    std::vector<std::string> splitLines(const std::string& content) {
        std::vector<std::string> result;
        std::istringstream stream(content);
        std::string line;
        while (std::getline(stream, line)) {
            result.push_back(line);
        }
        return result;
    }
    
    void parseLine() {
        std::string line = trim(lines_[lineIndex_]);
        
        // Skip empty lines and comments
        if (line.empty() || line[0] == '#') {
            return;
        }
        
        // Table header
        if (line[0] == '[') {
            parseTableHeader(line);
            return;
        }
        
        // Key-value pair
        size_t eqPos = line.find('=');
        if (eqPos != std::string::npos) {
            std::string key = trim(line.substr(0, eqPos));
            std::string value = trim(line.substr(eqPos + 1));
            (*currentTable_)[key] = parseValue(value);
        }
    }
    
    void parseTableHeader(const std::string& line) {
        bool isArrayOfTables = (line.size() > 2 && line[1] == '[');
        
        size_t start = isArrayOfTables ? 2 : 1;
        size_t end = isArrayOfTables ? line.rfind("]]") : line.rfind(']');
        
        if (end == std::string::npos) {
            throw std::runtime_error("Invalid table header at line " + 
                                     std::to_string(lineIndex_ + 1));
        }
        
        std::string tablePath = trim(line.substr(start, end - start));
        std::vector<std::string> keys = splitPath(tablePath);
        
        // Navigate to or create the table
        currentTable_ = &root_;
        for (size_t i = 0; i < keys.size(); i++) {
            const std::string& key = keys[i];
            
            if (!currentTable_->contains(key)) {
                if (isArrayOfTables && i == keys.size() - 1) {
                    (*currentTable_)[key] = ConfigArray{};
                } else {
                    (*currentTable_)[key] = ConfigTable{};
                }
            }
            
            ConfigValue& val = (*currentTable_)[key];
            
            if (isArrayOfTables && i == keys.size() - 1) {
                // Add new table to array
                val.asArray().push_back(ConfigTable{});
                currentTable_ = &val.asArray().back().asTable();
            } else if (val.isTable()) {
                currentTable_ = &val.asTable();
            } else if (val.isArray() && !val.asArray().empty()) {
                currentTable_ = &val.asArray().back().asTable();
            }
        }
    }
    
    std::vector<std::string> splitPath(const std::string& path) {
        std::vector<std::string> result;
        std::string current;
        
        for (char c : path) {
            if (c == '.') {
                if (!current.empty()) {
                    result.push_back(current);
                    current.clear();
                }
            } else {
                current += c;
            }
        }
        
        if (!current.empty()) {
            result.push_back(current);
        }
        
        return result;
    }
    
    ConfigValue parseValue(const std::string& value) {
        if (value.empty()) {
            return ConfigValue{};
        }
        
        // String
        if (value[0] == '"' || value[0] == '\'') {
            return parseString(value);
        }
        
        // Boolean
        if (value == "true") return ConfigValue{true};
        if (value == "false") return ConfigValue{false};
        
        // Array
        if (value[0] == '[') {
            return parseArray(value);
        }
        
        // Inline table
        if (value[0] == '{') {
            return parseInlineTable(value);
        }
        
        // Number
        return parseNumber(value);
    }
    
    ConfigValue parseString(const std::string& value) {
        char quote = value[0];
        size_t end = value.rfind(quote);
        if (end <= 0) {
            throw std::runtime_error("Unterminated string");
        }
        return ConfigValue{value.substr(1, end - 1)};
    }
    
    ConfigValue parseNumber(const std::string& value) {
        // Remove underscores (TOML allows 1_000_000)
        std::string cleaned;
        for (char c : value) {
            if (c != '_') cleaned += c;
        }
        
        // Check for float
        if (cleaned.find('.') != std::string::npos ||
            cleaned.find('e') != std::string::npos ||
            cleaned.find('E') != std::string::npos) {
            return ConfigValue{std::stod(cleaned)};
        }
        
        // Integer
        return ConfigValue{std::stoll(cleaned)};
    }
    
    ConfigValue parseArray(const std::string& value) {
        ConfigArray arr;
        std::string inner = trim(value.substr(1, value.size() - 2));
        
        if (inner.empty()) {
            return ConfigValue{arr};
        }
        
        // Simple comma-separated parsing (doesn't handle nested arrays well)
        size_t pos = 0;
        while (pos < inner.size()) {
            // Skip whitespace
            while (pos < inner.size() && std::isspace(inner[pos])) pos++;
            
            if (pos >= inner.size()) break;
            
            // Find end of value
            size_t end = pos;
            int depth = 0;
            bool inString = false;
            
            while (end < inner.size()) {
                char c = inner[end];
                if (c == '"' || c == '\'') inString = !inString;
                if (!inString) {
                    if (c == '[' || c == '{') depth++;
                    if (c == ']' || c == '}') depth--;
                    if (c == ',' && depth == 0) break;
                }
                end++;
            }
            
            std::string element = trim(inner.substr(pos, end - pos));
            if (!element.empty()) {
                arr.push_back(parseValue(element));
            }
            
            pos = end + 1;
        }
        
        return ConfigValue{arr};
    }
    
    ConfigValue parseInlineTable(const std::string& value) {
        ConfigTable table;
        std::string inner = trim(value.substr(1, value.size() - 2));
        
        // Parse key = value pairs
        size_t pos = 0;
        while (pos < inner.size()) {
            // Find key
            size_t eqPos = inner.find('=', pos);
            if (eqPos == std::string::npos) break;
            
            std::string key = trim(inner.substr(pos, eqPos - pos));
            
            // Find value end
            size_t start = eqPos + 1;
            size_t end = inner.find(',', start);
            if (end == std::string::npos) end = inner.size();
            
            std::string val = trim(inner.substr(start, end - start));
            table[key] = parseValue(val);
            
            pos = end + 1;
        }
        
        return ConfigValue{table};
    }
    
    static std::string trim(const std::string& str) {
        size_t start = 0;
        size_t end = str.size();
        
        while (start < end && std::isspace(str[start])) start++;
        while (end > start && std::isspace(str[end - 1])) end--;
        
        return str.substr(start, end - start);
    }
};

// ============================================================================
// Machine Configuration Structures
// ============================================================================

struct AxisConfig {
    std::string name;
    double stepsPerMm = 800.0;
    double maxVelocity = 5000.0;      // mm/min
    double maxAcceleration = 500.0;   // mm/s²
    double maxTravel = 200.0;         // mm
    double homePosition = 0.0;
    int homeDirection = -1;           // -1 or +1
    double homeFeedrate = 500.0;      // mm/min
    double homePulloff = 2.0;         // mm
    bool stepInvert = false;
    bool dirInvert = false;
    bool limitInvert = false;
    bool softLimitsEnabled = true;
};

struct SpindleConfig {
    uint32_t minRpm = 0;
    uint32_t maxRpm = 24000;
    uint32_t pwmFrequency = 5000;
    bool enabled = true;
    bool invertEnable = false;
    bool invertDirection = false;
};

struct CoolantConfig {
    bool mistEnabled = true;
    bool floodEnabled = true;
    bool mistInvert = false;
    bool floodInvert = false;
};

struct CommunicationConfig {
    std::string port;
    uint32_t baudRate = 115200;
    uint32_t timeout = 1000;
    uint32_t statusPollRate = 50;  // Hz
};

struct KinematicsConfig {
    std::string type = "cartesian";  // cartesian, corexy, delta
    double junctionDeviation = 0.02;
    double arcTolerance = 0.002;
    bool softLimitsEnabled = true;
    bool hardLimitsEnabled = true;
};

struct MachineConfig {
    std::string name;
    std::string version;
    
    std::vector<AxisConfig> axes;
    SpindleConfig spindle;
    CoolantConfig coolant;
    CommunicationConfig communication;
    KinematicsConfig kinematics;
    
    // Load from TOML file
    static MachineConfig loadFromFile(const std::string& filename) {
        TomlParser parser;
        ConfigTable config = parser.parseFile(filename);
        return fromConfigTable(config);
    }
    
    // Load from TOML string
    static MachineConfig loadFromString(const std::string& content) {
        TomlParser parser;
        ConfigTable config = parser.parse(content);
        return fromConfigTable(config);
    }
    
private:
    static MachineConfig fromConfigTable(const ConfigTable& config) {
        MachineConfig mc;
        
        // Machine info
        if (config.count("machine")) {
            const auto& machine = config.at("machine").asTable();
            if (machine.count("name")) mc.name = machine.at("name").asString();
            if (machine.count("version")) mc.version = machine.at("version").asString();
        }
        
        // Axes
        if (config.count("axis")) {
            const auto& axisArray = config.at("axis").asArray();
            for (const auto& axisVal : axisArray) {
                const auto& axis = axisVal.asTable();
                AxisConfig ac;
                
                if (axis.count("name")) ac.name = axis.at("name").asString();
                if (axis.count("steps_per_mm")) ac.stepsPerMm = axis.at("steps_per_mm").asDouble();
                if (axis.count("max_velocity")) ac.maxVelocity = axis.at("max_velocity").asDouble();
                if (axis.count("max_acceleration")) ac.maxAcceleration = axis.at("max_acceleration").asDouble();
                if (axis.count("max_travel")) ac.maxTravel = axis.at("max_travel").asDouble();
                if (axis.count("home_position")) ac.homePosition = axis.at("home_position").asDouble();
                if (axis.count("home_direction")) ac.homeDirection = axis.at("home_direction").get<int>();
                if (axis.count("home_feedrate")) ac.homeFeedrate = axis.at("home_feedrate").asDouble();
                if (axis.count("home_pulloff")) ac.homePulloff = axis.at("home_pulloff").asDouble();
                if (axis.count("step_invert")) ac.stepInvert = axis.at("step_invert").asBool();
                if (axis.count("dir_invert")) ac.dirInvert = axis.at("dir_invert").asBool();
                if (axis.count("limit_invert")) ac.limitInvert = axis.at("limit_invert").asBool();
                
                mc.axes.push_back(ac);
            }
        }
        
        // Spindle
        if (config.count("spindle")) {
            const auto& spindle = config.at("spindle").asTable();
            if (spindle.count("min_rpm")) mc.spindle.minRpm = spindle.at("min_rpm").get<uint32_t>();
            if (spindle.count("max_rpm")) mc.spindle.maxRpm = spindle.at("max_rpm").get<uint32_t>();
            if (spindle.count("pwm_frequency")) mc.spindle.pwmFrequency = spindle.at("pwm_frequency").get<uint32_t>();
            if (spindle.count("enabled")) mc.spindle.enabled = spindle.at("enabled").asBool();
        }
        
        // Coolant
        if (config.count("coolant")) {
            const auto& coolant = config.at("coolant").asTable();
            if (coolant.count("mist_enabled")) mc.coolant.mistEnabled = coolant.at("mist_enabled").asBool();
            if (coolant.count("flood_enabled")) mc.coolant.floodEnabled = coolant.at("flood_enabled").asBool();
        }
        
        // Communication
        if (config.count("communication")) {
            const auto& comm = config.at("communication").asTable();
            if (comm.count("port")) mc.communication.port = comm.at("port").asString();
            if (comm.count("baud_rate")) mc.communication.baudRate = comm.at("baud_rate").get<uint32_t>();
            if (comm.count("timeout")) mc.communication.timeout = comm.at("timeout").get<uint32_t>();
            if (comm.count("status_poll_rate")) mc.communication.statusPollRate = comm.at("status_poll_rate").get<uint32_t>();
        }
        
        // Kinematics
        if (config.count("kinematics")) {
            const auto& kin = config.at("kinematics").asTable();
            if (kin.count("type")) mc.kinematics.type = kin.at("type").asString();
            if (kin.count("junction_deviation")) mc.kinematics.junctionDeviation = kin.at("junction_deviation").asDouble();
            if (kin.count("arc_tolerance")) mc.kinematics.arcTolerance = kin.at("arc_tolerance").asDouble();
            if (kin.count("soft_limits")) mc.kinematics.softLimitsEnabled = kin.at("soft_limits").asBool();
            if (kin.count("hard_limits")) mc.kinematics.hardLimitsEnabled = kin.at("hard_limits").asBool();
        }
        
        return mc;
    }
};

} // namespace config
} // namespace cnc

#endif // CNC_CONFIG_H
