/*
 * NGC G-Code Parser Implementation
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#include "ngc_parser.h"
#include <cmath>
#include <cctype>
#include <sstream>
#include <fstream>
#include <algorithm>
#include <cstring>

namespace ngc_parser {

// ============================================================================
// Position Implementation
// ============================================================================

double& Position::operator[](int idx) {
    switch(idx) {
        case 0: return x; case 1: return y; case 2: return z;
        case 3: return a; case 4: return b; case 5: return c;
        case 6: return u; case 7: return v; case 8: return w;
        default: return x;
    }
}

const double& Position::operator[](int idx) const {
    switch(idx) {
        case 0: return x; case 1: return y; case 2: return z;
        case 3: return a; case 4: return b; case 5: return c;
        case 6: return u; case 7: return v; case 8: return w;
        default: return x;
    }
}

// ============================================================================
// Parser Implementation
// ============================================================================

struct Parser::Impl {
    ModalState modal_state;
    Position current_position;
    std::array<double, MAX_PARAMETERS> parameters{};
    
    bool block_delete_enabled = true;
    bool optional_stop_enabled = true;
    int current_line = 0;
    
    // Parse a word (letter + number) from the line
    bool parseWord(const std::string& line, size_t& pos, Word& word) {
        // Skip whitespace
        while (pos < line.size() && std::isspace(line[pos])) {
            pos++;
        }
        
        if (pos >= line.size()) return false;
        
        char c = std::toupper(line[pos]);
        if (!std::isalpha(c)) return false;
        
        word.letter = c;
        word.column = static_cast<int>(pos);
        pos++;
        
        // Skip whitespace between letter and number
        while (pos < line.size() && std::isspace(line[pos])) {
            pos++;
        }
        
        // Parse number
        size_t start = pos;
        bool has_decimal = false;
        bool has_sign = false;
        
        if (pos < line.size() && (line[pos] == '+' || line[pos] == '-')) {
            has_sign = true;
            pos++;
        }
        
        while (pos < line.size()) {
            if (std::isdigit(line[pos])) {
                pos++;
            } else if (line[pos] == '.' && !has_decimal) {
                has_decimal = true;
                pos++;
            } else {
                break;
            }
        }
        
        if (pos == start || (has_sign && pos == start + 1)) {
            return false;  // No number found
        }
        
        word.value = std::stod(line.substr(start, pos - start));
        return true;
    }
    
    // Parse a block (line of G-code)
    ParseResult parseBlock(const std::string& line, Block& block) {
        block.original_text = line;
        block.source_line = current_line;
        
        size_t pos = 0;
        
        // Skip leading whitespace
        while (pos < line.size() && std::isspace(line[pos])) {
            pos++;
        }
        
        // Check for block delete
        if (pos < line.size() && line[pos] == '/') {
            block.block_delete = true;
            pos++;
        }
        
        // Parse words
        Word word;
        while (pos < line.size()) {
            // Check for comment
            if (line[pos] == '(' || line[pos] == ';') {
                if (line[pos] == '(') {
                    size_t end = line.find(')', pos);
                    if (end != std::string::npos) {
                        block.comment = line.substr(pos + 1, end - pos - 1);
                        pos = end + 1;
                        continue;
                    }
                }
                // Rest of line is comment
                block.comment = line.substr(pos + 1);
                break;
            }
            
            // Skip whitespace
            if (std::isspace(line[pos])) {
                pos++;
                continue;
            }
            
            word.line = current_line;
            if (!parseWord(line, pos, word)) {
                if (pos < line.size() && !std::isspace(line[pos])) {
                    return {false, ParseError::UNEXPECTED_CHAR, current_line, 
                            static_cast<int>(pos), "Unexpected character"};
                }
                continue;
            }
            
            // Process word based on letter
            switch (word.letter) {
                case 'N':
                    block.line_number = static_cast<int>(word.value);
                    break;
                    
                case 'G': {
                    // Convert to integer code (G38.2 -> 382)
                    int code = static_cast<int>(word.value * 10 + 0.5);
                    if (std::fabs(word.value - std::round(word.value)) < 0.001) {
                        code = static_cast<int>(word.value + 0.5) * 10;
                    }
                    block.g_codes.push_back(code);
                    break;
                }
                    
                case 'M':
                    block.m_codes.push_back(static_cast<int>(word.value + 0.5));
                    break;
                    
                case 'X': block.axes[0] = word.value; break;
                case 'Y': block.axes[1] = word.value; break;
                case 'Z': block.axes[2] = word.value; break;
                case 'A': block.axes[3] = word.value; break;
                case 'B': block.axes[4] = word.value; break;
                case 'C': block.axes[5] = word.value; break;
                case 'U': block.axes[6] = word.value; break;
                case 'V': block.axes[7] = word.value; break;
                case 'W': block.axes[8] = word.value; break;
                    
                case 'F': block.f_word = word.value; break;
                case 'S': block.s_word = word.value; break;
                case 'T': block.t_word = static_cast<int>(word.value + 0.5); break;
                case 'P': block.p_word = word.value; break;
                case 'Q': block.q_word = word.value; break;
                case 'R': block.r_word = word.value; break;
                case 'L': block.l_word = static_cast<int>(word.value + 0.5); break;
                case 'H': block.h_word = static_cast<int>(word.value + 0.5); break;
                case 'D': block.d_word = static_cast<int>(word.value + 0.5); break;
                    
                case 'I': block.i_word = word.value; break;
                case 'J': block.j_word = word.value; break;
                case 'K': block.k_word = word.value; break;
                    
                default:
                    // Unknown word letter - ignore or warn
                    break;
            }
        }
        
        return {true, ParseError::NONE, 0, 0, ""};
    }
    
    // Execute block and generate canonical commands
    ParseResult executeBlock(const Block& block, std::vector<CanonCommand>& commands) {
        // Skip block if block delete is active
        if (block.block_delete && block_delete_enabled) {
            return {true, ParseError::NONE, 0, 0, ""};
        }
        
        // Process G-codes first (modal settings)
        for (int gcode : block.g_codes) {
            processGCode(gcode, block, commands);
        }
        
        // Update feed rate if specified
        if (block.f_word.has_value()) {
            modal_state.feed_rate = block.f_word.value();
            
            CanonCommand cmd;
            cmd.type = CanonCommandType::SET_FEED_RATE;
            cmd.line_number = block.source_line;
            cmd.feed_rate = modal_state.feed_rate;
            commands.push_back(cmd);
        }
        
        // Update spindle speed if specified
        if (block.s_word.has_value()) {
            modal_state.spindle_speed = block.s_word.value();
            
            CanonCommand cmd;
            cmd.type = CanonCommandType::SET_SPINDLE_SPEED;
            cmd.line_number = block.source_line;
            cmd.spindle_speed = modal_state.spindle_speed;
            commands.push_back(cmd);
        }
        
        // Tool selection
        if (block.t_word.has_value()) {
            CanonCommand cmd;
            cmd.type = CanonCommandType::SELECT_TOOL;
            cmd.line_number = block.source_line;
            cmd.tool_number = block.t_word.value();
            commands.push_back(cmd);
        }
        
        // Process M-codes
        for (int mcode : block.m_codes) {
            processMCode(mcode, block, commands);
        }
        
        // Generate motion command if axes are specified
        bool has_motion = false;
        for (const auto& axis : block.axes) {
            if (axis.has_value()) {
                has_motion = true;
                break;
            }
        }
        
        if (has_motion || block.i_word || block.j_word || block.k_word) {
            generateMotion(block, commands);
        }
        
        // Handle comment
        if (!block.comment.empty()) {
            CanonCommand cmd;
            cmd.type = CanonCommandType::COMMENT;
            cmd.line_number = block.source_line;
            cmd.text = block.comment;
            commands.push_back(cmd);
        }
        
        return {true, ParseError::NONE, 0, 0, ""};
    }
    
    void processGCode(int code, const Block& block, std::vector<CanonCommand>& commands) {
        (void)commands;  // Some codes don't generate commands
        
        switch (code) {
            // Motion modes (Group 1)
            case 0:   modal_state.motion = MotionMode::RAPID; break;
            case 10:  modal_state.motion = MotionMode::LINEAR; break;
            case 20:  modal_state.motion = MotionMode::ARC_CW; break;
            case 30:  modal_state.motion = MotionMode::ARC_CCW; break;
            case 40:  // G4 - Dwell
                if (block.p_word.has_value()) {
                    CanonCommand cmd;
                    cmd.type = CanonCommandType::DWELL;
                    cmd.line_number = block.source_line;
                    cmd.dwell_seconds = block.p_word.value();
                    commands.push_back(cmd);
                }
                break;
            case 800: modal_state.motion = MotionMode::CANCEL_MOTION; break;
            case 810: modal_state.motion = MotionMode::CANNED_CYCLE_81; break;
            case 820: modal_state.motion = MotionMode::CANNED_CYCLE_82; break;
            case 830: modal_state.motion = MotionMode::CANNED_CYCLE_83; break;
            
            // Plane selection (Group 2)
            case 170: modal_state.plane = PlaneSelect::XY; break;
            case 180: modal_state.plane = PlaneSelect::XZ; break;
            case 190: modal_state.plane = PlaneSelect::YZ; break;
            
            // Distance mode (Group 3)
            case 900: modal_state.distance = DistanceMode::ABSOLUTE; break;
            case 910: modal_state.distance = DistanceMode::INCREMENTAL; break;
            
            // Feed rate mode (Group 5)
            case 930: modal_state.feed_mode = FeedRateMode::INVERSE_TIME; break;
            case 940: modal_state.feed_mode = FeedRateMode::UNITS_PER_MIN; break;
            case 950: modal_state.feed_mode = FeedRateMode::UNITS_PER_REV; break;
            
            // Units (Group 6)
            case 200: modal_state.units = Units::INCHES; break;
            case 210: modal_state.units = Units::MM; break;
            
            // Cutter compensation (Group 7)
            case 400: modal_state.cutter_comp = CutterComp::OFF; break;
            case 410: modal_state.cutter_comp = CutterComp::LEFT; break;
            case 420: modal_state.cutter_comp = CutterComp::RIGHT; break;
            
            // Tool length offset (Group 8)
            case 430: 
                modal_state.tool_offset = ToolLengthOffset::POSITIVE;
                if (block.h_word.has_value()) {
                    modal_state.tool_offset_index = block.h_word.value();
                }
                break;
            case 490: modal_state.tool_offset = ToolLengthOffset::OFF; break;
            
            // Coordinate systems (Group 12)
            case 540: modal_state.coord_system = CoordSystem::G54; break;
            case 550: modal_state.coord_system = CoordSystem::G55; break;
            case 560: modal_state.coord_system = CoordSystem::G56; break;
            case 570: modal_state.coord_system = CoordSystem::G57; break;
            case 580: modal_state.coord_system = CoordSystem::G58; break;
            case 590: modal_state.coord_system = CoordSystem::G59; break;
            
            // Path control (Group 13)
            case 610: modal_state.path_control = PathControl::EXACT_PATH; break;
            case 611: modal_state.path_control = PathControl::EXACT_STOP; break;
            case 640: modal_state.path_control = PathControl::BLENDING; break;
            
            // Return mode (Group 10)
            case 980: modal_state.return_mode = ReturnMode::TO_Z; break;
            case 990: modal_state.return_mode = ReturnMode::TO_R; break;
            
            default:
                // Unknown G-code
                break;
        }
    }
    
    void processMCode(int code, const Block& block, std::vector<CanonCommand>& commands) {
        (void)block;
        
        CanonCommand cmd;
        cmd.line_number = block.source_line;
        
        switch (code) {
            case 0:  // Program stop
                cmd.type = CanonCommandType::PROGRAM_STOP;
                commands.push_back(cmd);
                break;
                
            case 1:  // Optional stop
                cmd.type = CanonCommandType::OPTIONAL_STOP;
                commands.push_back(cmd);
                break;
                
            case 2:  // Program end
            case 30: // Program end and reset
                cmd.type = CanonCommandType::PROGRAM_END;
                commands.push_back(cmd);
                break;
                
            case 3:  // Spindle CW
                modal_state.spindle = SpindleMode::CW;
                cmd.type = CanonCommandType::START_SPINDLE_CW;
                cmd.spindle_speed = modal_state.spindle_speed;
                commands.push_back(cmd);
                break;
                
            case 4:  // Spindle CCW
                modal_state.spindle = SpindleMode::CCW;
                cmd.type = CanonCommandType::START_SPINDLE_CCW;
                cmd.spindle_speed = modal_state.spindle_speed;
                commands.push_back(cmd);
                break;
                
            case 5:  // Spindle stop
                modal_state.spindle = SpindleMode::OFF;
                cmd.type = CanonCommandType::STOP_SPINDLE;
                commands.push_back(cmd);
                break;
                
            case 6:  // Tool change
                cmd.type = CanonCommandType::CHANGE_TOOL;
                cmd.tool_number = modal_state.current_tool;
                commands.push_back(cmd);
                break;
                
            case 7:  // Mist coolant on
                modal_state.coolant = CoolantMode::MIST;
                cmd.type = CanonCommandType::MIST_ON;
                commands.push_back(cmd);
                break;
                
            case 8:  // Flood coolant on
                modal_state.coolant = CoolantMode::FLOOD;
                cmd.type = CanonCommandType::FLOOD_ON;
                commands.push_back(cmd);
                break;
                
            case 9:  // Coolant off
                modal_state.coolant = CoolantMode::OFF;
                cmd.type = CanonCommandType::COOLANT_OFF;
                commands.push_back(cmd);
                break;
                
            case 48: // Enable overrides
                cmd.type = CanonCommandType::ENABLE_FEED_OVERRIDE;
                commands.push_back(cmd);
                break;
                
            case 49: // Disable overrides
                cmd.type = CanonCommandType::DISABLE_FEED_OVERRIDE;
                commands.push_back(cmd);
                break;
                
            default:
                // Unknown M-code
                break;
        }
    }
    
    void generateMotion(const Block& block, std::vector<CanonCommand>& commands) {
        // Calculate target position
        Position target = current_position;
        
        for (int i = 0; i < MAX_AXES; i++) {
            if (block.axes[i].has_value()) {
                if (modal_state.distance == DistanceMode::INCREMENTAL) {
                    target[i] = current_position[i] + block.axes[i].value();
                } else {
                    target[i] = block.axes[i].value();
                }
            }
        }
        
        CanonCommand cmd;
        cmd.line_number = block.source_line;
        cmd.end_point = {target.x, target.y, target.z,
                         target.a, target.b, target.c,
                         target.u, target.v, target.w};
        cmd.feed_rate = modal_state.feed_rate;
        
        switch (modal_state.motion) {
            case MotionMode::RAPID:
                cmd.type = CanonCommandType::STRAIGHT_TRAVERSE;
                break;
                
            case MotionMode::LINEAR:
                cmd.type = CanonCommandType::STRAIGHT_FEED;
                break;
                
            case MotionMode::ARC_CW:
            case MotionMode::ARC_CCW: {
                cmd.type = CanonCommandType::ARC_FEED;
                cmd.rotation = (modal_state.motion == MotionMode::ARC_CCW) ? 1 : -1;
                
                // Calculate center from I, J, K
                double cx = current_position.x + (block.i_word.value_or(0.0));
                double cy = current_position.y + (block.j_word.value_or(0.0));
                double cz = current_position.z + (block.k_word.value_or(0.0));
                
                cmd.center = {cx, cy, cz, 0, 0, 0, 0, 0, 0};
                cmd.plane = static_cast<int>(modal_state.plane);
                break;
            }
                
            default:
                cmd.type = CanonCommandType::STRAIGHT_FEED;
                break;
        }
        
        commands.push_back(cmd);
        current_position = target;
    }
};

// ============================================================================
// Parser Public Interface
// ============================================================================

Parser::Parser() : pImpl(std::make_unique<Impl>()) {}
Parser::~Parser() = default;

ParseResult Parser::parseLine(const std::string& line, 
                              std::vector<CanonCommand>& commands) {
    Block block;
    pImpl->current_line++;
    
    auto result = pImpl->parseBlock(line, block);
    if (!result.success) {
        return result;
    }
    
    return pImpl->executeBlock(block, commands);
}

ParseResult Parser::parseProgram(const std::string& program,
                                 std::vector<CanonCommand>& commands) {
    std::istringstream stream(program);
    std::string line;
    
    pImpl->current_line = 0;
    
    while (std::getline(stream, line)) {
        auto result = parseLine(line, commands);
        if (!result.success) {
            return result;
        }
    }
    
    return {true, ParseError::NONE, 0, 0, ""};
}

ParseResult Parser::parseFile(const std::string& filename,
                              std::vector<CanonCommand>& commands) {
    std::ifstream file(filename);
    if (!file.is_open()) {
        return {false, ParseError::FILE_ERROR, 0, 0, 
                "Could not open file: " + filename};
    }
    
    std::string content((std::istreambuf_iterator<char>(file)),
                        std::istreambuf_iterator<char>());
    
    return parseProgram(content, commands);
}

const ModalState& Parser::getModalState() const {
    return pImpl->modal_state;
}

void Parser::setModalState(const ModalState& state) {
    pImpl->modal_state = state;
}

void Parser::resetModalState() {
    pImpl->modal_state = ModalState{};
}

Position Parser::getCurrentPosition() const {
    return pImpl->current_position;
}

void Parser::setCurrentPosition(const Position& pos) {
    pImpl->current_position = pos;
}

double Parser::getParameter(int index) const {
    if (index >= 0 && index < MAX_PARAMETERS) {
        return pImpl->parameters[index];
    }
    return 0.0;
}

void Parser::setParameter(int index, double value) {
    if (index >= 0 && index < MAX_PARAMETERS) {
        pImpl->parameters[index] = value;
    }
}

void Parser::setBlockDeleteEnabled(bool enabled) {
    pImpl->block_delete_enabled = enabled;
}

void Parser::setOptionalStopEnabled(bool enabled) {
    pImpl->optional_stop_enabled = enabled;
}

void Parser::reset() {
    pImpl->modal_state = ModalState{};
    pImpl->current_position = Position{};
    pImpl->parameters.fill(0.0);
    pImpl->current_line = 0;
}

// ============================================================================
// Utility Functions
// ============================================================================

std::string parseErrorToString(ParseError error) {
    switch (error) {
        case ParseError::NONE: return "No error";
        case ParseError::UNEXPECTED_CHAR: return "Unexpected character";
        case ParseError::INVALID_NUMBER: return "Invalid number format";
        case ParseError::MISSING_VALUE: return "Missing value";
        case ParseError::DUPLICATE_WORD: return "Duplicate word";
        case ParseError::CONFLICTING_GCODES: return "Conflicting G-codes";
        case ParseError::UNKNOWN_GCODE: return "Unknown G-code";
        case ParseError::UNKNOWN_MCODE: return "Unknown M-code";
        case ParseError::MISSING_AXIS: return "Missing axis value";
        case ParseError::MISSING_FEED_RATE: return "Missing feed rate";
        case ParseError::RADIUS_ARC_ERROR: return "Arc radius error";
        case ParseError::PARAMETER_OUT_OF_RANGE: return "Parameter out of range";
        case ParseError::SUBROUTINE_ERROR: return "Subroutine error";
        case ParseError::EXPRESSION_ERROR: return "Expression error";
        case ParseError::FILE_ERROR: return "File error";
        default: return "Unknown error";
    }
}

std::string motionModeToString(MotionMode mode) {
    switch (mode) {
        case MotionMode::RAPID: return "G0 (Rapid)";
        case MotionMode::LINEAR: return "G1 (Linear)";
        case MotionMode::ARC_CW: return "G2 (Arc CW)";
        case MotionMode::ARC_CCW: return "G3 (Arc CCW)";
        case MotionMode::DWELL: return "G4 (Dwell)";
        case MotionMode::CANCEL_MOTION: return "G80 (Cancel)";
        default: return "Unknown";
    }
}

std::string canonCommandTypeToString(CanonCommandType type) {
    switch (type) {
        case CanonCommandType::STRAIGHT_TRAVERSE: return "STRAIGHT_TRAVERSE";
        case CanonCommandType::STRAIGHT_FEED: return "STRAIGHT_FEED";
        case CanonCommandType::ARC_FEED: return "ARC_FEED";
        case CanonCommandType::DWELL: return "DWELL";
        case CanonCommandType::SET_SPINDLE_SPEED: return "SET_SPINDLE_SPEED";
        case CanonCommandType::START_SPINDLE_CW: return "START_SPINDLE_CW";
        case CanonCommandType::START_SPINDLE_CCW: return "START_SPINDLE_CCW";
        case CanonCommandType::STOP_SPINDLE: return "STOP_SPINDLE";
        case CanonCommandType::SELECT_TOOL: return "SELECT_TOOL";
        case CanonCommandType::CHANGE_TOOL: return "CHANGE_TOOL";
        case CanonCommandType::MIST_ON: return "MIST_ON";
        case CanonCommandType::FLOOD_ON: return "FLOOD_ON";
        case CanonCommandType::COOLANT_OFF: return "COOLANT_OFF";
        case CanonCommandType::SET_FEED_RATE: return "SET_FEED_RATE";
        case CanonCommandType::PROGRAM_STOP: return "PROGRAM_STOP";
        case CanonCommandType::PROGRAM_END: return "PROGRAM_END";
        case CanonCommandType::COMMENT: return "COMMENT";
        default: return "UNKNOWN";
    }
}

} // namespace ngc_parser
