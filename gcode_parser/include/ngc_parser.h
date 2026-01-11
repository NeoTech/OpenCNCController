/*
 * NGC G-Code Parser - RS274/NGC Compatible
 * 
 * An NGC (NIST RS274/NGC) compatible G-code parser inspired by LinuxCNC.
 * Parses standard G-code and outputs canonical commands for motion control.
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#ifndef NGC_PARSER_H
#define NGC_PARSER_H

#include <cstdint>
#include <string>
#include <vector>
#include <array>
#include <optional>
#include <variant>
#include <functional>
#include <memory>
#include <unordered_map>

namespace ngc_parser {

// ============================================================================
// Constants
// ============================================================================

constexpr int MAX_AXES = 9;
constexpr int MAX_PARAMETERS = 5400;  // NGC parameter range
constexpr int MAX_STACK_DEPTH = 10;   // Subroutine call depth

// Parameter ranges (RS274/NGC standard)
constexpr int PARAM_PROBE_X = 5061;
constexpr int PARAM_PROBE_Y = 5062;
constexpr int PARAM_PROBE_Z = 5063;
constexpr int PARAM_G28_X = 5161;
constexpr int PARAM_G28_Y = 5162;
constexpr int PARAM_G28_Z = 5163;
constexpr int PARAM_G30_X = 5181;
constexpr int PARAM_G30_Y = 5182;
constexpr int PARAM_G30_Z = 5183;
constexpr int PARAM_G54_X = 5221;  // G54 offset start
constexpr int PARAM_CURRENT_X = 5420;
constexpr int PARAM_CURRENT_Y = 5421;
constexpr int PARAM_CURRENT_Z = 5422;

// ============================================================================
// Enumerations - Modal Groups
// ============================================================================

// Modal Group 1: Motion
enum class MotionMode : uint8_t {
    RAPID = 0,              // G0
    LINEAR = 1,             // G1
    ARC_CW = 2,             // G2
    ARC_CCW = 3,            // G3
    DWELL = 4,              // G4
    CUBIC_SPLINE = 5,       // G5 (optional)
    PROBE_TOWARD = 38,      // G38.2
    PROBE_TOWARD_NO_ERR = 382, // G38.3
    PROBE_AWAY = 383,       // G38.4
    PROBE_AWAY_NO_ERR = 384,   // G38.5
    CANNED_CYCLE_81 = 81,   // G81
    CANNED_CYCLE_82 = 82,   // G82
    CANNED_CYCLE_83 = 83,   // G83 peck drilling
    CANNED_CYCLE_84 = 84,   // G84 tapping
    CANNED_CYCLE_85 = 85,   // G85 boring
    CANNED_CYCLE_86 = 86,   // G86
    CANNED_CYCLE_87 = 87,   // G87
    CANNED_CYCLE_88 = 88,   // G88
    CANNED_CYCLE_89 = 89,   // G89
    CANCEL_MOTION = 80      // G80
};

// Modal Group 2: Plane Selection
enum class PlaneSelect : uint8_t {
    XY = 17,  // G17
    XZ = 18,  // G18
    YZ = 19   // G19
};

// Modal Group 3: Distance Mode
enum class DistanceMode : uint8_t {
    ABSOLUTE = 90,      // G90
    INCREMENTAL = 91    // G91
};

// Modal Group 4: Stopping (non-modal, but tracked)
enum class StopMode : uint8_t {
    NONE = 0,
    PROGRAM_STOP = 0,       // M0
    OPTIONAL_STOP = 1,      // M1
    PROGRAM_END = 2,        // M2
    PROGRAM_END_RESET = 30  // M30
};

// Modal Group 5: Feed Rate Mode
enum class FeedRateMode : uint8_t {
    INVERSE_TIME = 93,    // G93
    UNITS_PER_MIN = 94,   // G94
    UNITS_PER_REV = 95    // G95
};

// Modal Group 6: Units
enum class Units : uint8_t {
    INCHES = 20,   // G20
    MM = 21        // G21
};

// Modal Group 7: Cutter Compensation
enum class CutterComp : uint8_t {
    OFF = 40,     // G40
    LEFT = 41,    // G41
    RIGHT = 42    // G42
};

// Modal Group 8: Tool Length Offset
enum class ToolLengthOffset : uint8_t {
    OFF = 49,           // G49
    POSITIVE = 43,      // G43
    NEGATIVE = 44,      // G44 (less common)
    DYNAMIC = 432       // G43.2 (optional)
};

// Modal Group 10: Return Mode (canned cycles)
enum class ReturnMode : uint8_t {
    TO_R = 99,    // G99 - return to R level
    TO_Z = 98     // G98 - return to initial Z
};

// Modal Group 12: Coordinate System
enum class CoordSystem : uint8_t {
    G54 = 54,
    G55 = 55,
    G56 = 56,
    G57 = 57,
    G58 = 58,
    G59 = 59,
    G59_1 = 59,   // G59.1
    G59_2 = 59,   // G59.2
    G59_3 = 59    // G59.3
};

// Modal Group 13: Path Control
enum class PathControl : uint8_t {
    EXACT_PATH = 61,     // G61
    EXACT_STOP = 611,    // G61.1
    BLENDING = 64        // G64
};

// Spindle Control
enum class SpindleMode : uint8_t {
    OFF = 5,
    CW = 3,
    CCW = 4
};

// Coolant Control
enum class CoolantMode : uint8_t {
    OFF = 9,
    MIST = 7,
    FLOOD = 8
};

// ============================================================================
// Data Structures
// ============================================================================

struct Position {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;
    double a = 0.0;
    double b = 0.0;
    double c = 0.0;
    double u = 0.0;
    double v = 0.0;
    double w = 0.0;
    
    double& operator[](int idx);
    const double& operator[](int idx) const;
};

// Word value (letter + number, e.g., X10.5)
struct Word {
    char letter = 0;
    double value = 0.0;
    int line = 0;
    int column = 0;
};

// Parsed block (one line of G-code)
struct Block {
    int line_number = -1;        // N-word value
    int source_line = 0;         // Actual line in file
    std::string original_text;   // Original line text
    std::string comment;         // Comment text
    bool block_delete = false;   // Line starts with /
    
    // Words found in this block
    std::vector<int> g_codes;    // G codes (x10, so G1 = 10, G0 = 0, G38.2 = 382)
    std::vector<int> m_codes;    // M codes
    
    // Axis values (NaN if not specified)
    std::array<std::optional<double>, MAX_AXES> axes;
    
    // Other words
    std::optional<double> f_word;   // Feed rate
    std::optional<double> s_word;   // Spindle speed
    std::optional<int> t_word;      // Tool number
    std::optional<double> p_word;   // Dwell time / parameter
    std::optional<double> q_word;   // Peck depth / parameter
    std::optional<double> r_word;   // Arc radius / canned cycle
    std::optional<int> l_word;      // Loop count
    std::optional<int> h_word;      // Tool length offset index
    std::optional<int> d_word;      // Cutter comp index
    
    // Arc center (I, J, K)
    std::optional<double> i_word;
    std::optional<double> j_word;
    std::optional<double> k_word;
};

// ============================================================================
// Modal State
// ============================================================================

struct ModalState {
    // Current modal settings
    MotionMode motion = MotionMode::RAPID;
    PlaneSelect plane = PlaneSelect::XY;
    DistanceMode distance = DistanceMode::ABSOLUTE;
    DistanceMode arc_distance = DistanceMode::INCREMENTAL;  // G90.1/G91.1
    FeedRateMode feed_mode = FeedRateMode::UNITS_PER_MIN;
    Units units = Units::MM;
    CutterComp cutter_comp = CutterComp::OFF;
    ToolLengthOffset tool_offset = ToolLengthOffset::OFF;
    ReturnMode return_mode = ReturnMode::TO_Z;
    CoordSystem coord_system = CoordSystem::G54;
    PathControl path_control = PathControl::EXACT_PATH;
    SpindleMode spindle = SpindleMode::OFF;
    CoolantMode coolant = CoolantMode::OFF;
    
    // Current values
    double feed_rate = 0.0;
    double spindle_speed = 0.0;
    int current_tool = 0;
    int tool_offset_index = 0;
    int cutter_comp_index = 0;
    
    // Canned cycle retained values
    double cycle_r = 0.0;
    double cycle_z = 0.0;
    double cycle_p = 0.0;
    double cycle_q = 0.0;
    int cycle_l = 1;
};

// ============================================================================
// Canonical Commands (Parser Output)
// ============================================================================

// These are the canonical commands output by the parser, similar to LinuxCNC

struct CanonPosition {
    double x, y, z, a, b, c, u, v, w;
};

enum class CanonCommandType {
    // Motion commands
    STRAIGHT_TRAVERSE,      // Rapid move (G0)
    STRAIGHT_FEED,          // Linear feed (G1)
    ARC_FEED,               // Arc move (G2/G3)
    
    // Dwelling
    DWELL,
    
    // Spindle
    SET_SPINDLE_SPEED,
    START_SPINDLE_CW,
    START_SPINDLE_CCW,
    STOP_SPINDLE,
    ORIENT_SPINDLE,
    
    // Tools
    SELECT_TOOL,
    CHANGE_TOOL,
    USE_TOOL_LENGTH_OFFSET,
    
    // Coolant
    MIST_ON,
    FLOOD_ON,
    COOLANT_OFF,
    
    // Overrides
    ENABLE_FEED_OVERRIDE,
    DISABLE_FEED_OVERRIDE,
    ENABLE_SPEED_OVERRIDE,
    DISABLE_SPEED_OVERRIDE,
    
    // Program control
    OPTIONAL_STOP,
    PROGRAM_STOP,
    PROGRAM_END,
    
    // Comments and messages
    COMMENT,
    MESSAGE,
    
    // Probing
    STRAIGHT_PROBE,
    
    // Coordinate system
    SET_ORIGIN_OFFSETS,
    RESET_ORIGIN_OFFSETS,
    SET_G92_OFFSET,
    RESET_G92_OFFSET,
    
    // Plane selection
    SELECT_PLANE,
    
    // Path control
    SET_PATH_CONTROL_MODE,
    
    // Feed modes
    SET_FEED_RATE,
    SET_FEED_REFERENCE,
    SET_MOTION_CONTROL_MODE,
    
    // Special
    PALLET_SHUTTLE,
    TURN_PROBE_ON,
    TURN_PROBE_OFF
};

struct CanonCommand {
    CanonCommandType type;
    int line_number = 0;
    
    // Motion parameters
    CanonPosition end_point{};
    CanonPosition center{};     // For arcs
    double feed_rate = 0.0;
    int rotation = 0;           // Arc rotation count
    int plane = 0;              // 1=XY, 2=XZ, 3=YZ
    
    // Tool parameters
    int tool_number = 0;
    double tool_offset = 0.0;
    
    // Spindle parameters
    double spindle_speed = 0.0;
    int spindle_direction = 0;
    
    // Time parameters
    double dwell_seconds = 0.0;
    
    // Text parameters
    std::string text;
};

// ============================================================================
// Error Handling
// ============================================================================

enum class ParseError {
    NONE = 0,
    UNEXPECTED_CHAR,
    INVALID_NUMBER,
    MISSING_VALUE,
    DUPLICATE_WORD,
    CONFLICTING_GCODES,
    UNKNOWN_GCODE,
    UNKNOWN_MCODE,
    MISSING_AXIS,
    MISSING_FEED_RATE,
    RADIUS_ARC_ERROR,
    PARAMETER_OUT_OF_RANGE,
    SUBROUTINE_ERROR,
    EXPRESSION_ERROR,
    FILE_ERROR
};

struct ParseResult {
    bool success = true;
    ParseError error = ParseError::NONE;
    int line = 0;
    int column = 0;
    std::string message;
};

// ============================================================================
// Parser Interface
// ============================================================================

class Parser {
public:
    Parser();
    ~Parser();
    
    // Parse a single line, output canonical commands
    ParseResult parseLine(const std::string& line, 
                          std::vector<CanonCommand>& commands);
    
    // Parse entire program
    ParseResult parseProgram(const std::string& program,
                             std::vector<CanonCommand>& commands);
    
    // Parse from file
    ParseResult parseFile(const std::string& filename,
                          std::vector<CanonCommand>& commands);
    
    // State access
    const ModalState& getModalState() const;
    void setModalState(const ModalState& state);
    void resetModalState();
    
    // Position access
    Position getCurrentPosition() const;
    void setCurrentPosition(const Position& pos);
    
    // Parameter access (NGC #variables)
    double getParameter(int index) const;
    void setParameter(int index, double value);
    
    // Configuration
    void setBlockDeleteEnabled(bool enabled);
    void setOptionalStopEnabled(bool enabled);
    
    // Reset parser state
    void reset();
    
private:
    struct Impl;
    std::unique_ptr<Impl> pImpl;
};

// ============================================================================
// Utility Functions
// ============================================================================

std::string parseErrorToString(ParseError error);
std::string motionModeToString(MotionMode mode);
std::string canonCommandTypeToString(CanonCommandType type);

} // namespace ngc_parser

#endif // NGC_PARSER_H
