# NGC G-Code Parser

An RS274/NGC compatible G-code parser inspired by LinuxCNC's interpreter.

## Features

- Full NGC G-code dialect support
- Modal group management
- Canonical command output
- Line-by-line or program parsing
- Expression evaluation (planned)
- Subroutine support (planned)

## Supported G-Codes

### Motion (Group 1)
- G0 - Rapid positioning
- G1 - Linear interpolation
- G2 - Circular/Arc CW
- G3 - Circular/Arc CCW
- G4 - Dwell
- G38.2-G38.5 - Probing
- G80 - Cancel canned cycle
- G81-G89 - Canned cycles

### Plane Selection (Group 2)
- G17 - XY plane
- G18 - XZ plane
- G19 - YZ plane

### Distance Mode (Group 3)
- G90 - Absolute
- G91 - Incremental

### Feed Rate Mode (Group 5)
- G93 - Inverse time
- G94 - Units per minute
- G95 - Units per revolution

### Units (Group 6)
- G20 - Inches
- G21 - Millimeters

### Cutter Compensation (Group 7)
- G40 - Cancel
- G41 - Left
- G42 - Right

### Tool Length Offset (Group 8)
- G43 - Positive offset
- G49 - Cancel

### Coordinate Systems (Group 12)
- G54-G59 - Work offsets
- G59.1, G59.2, G59.3 - Extended offsets

### Path Control (Group 13)
- G61 - Exact path
- G61.1 - Exact stop
- G64 - Path blending

## Supported M-Codes

- M0 - Program stop
- M1 - Optional stop
- M2 - Program end
- M3 - Spindle CW
- M4 - Spindle CCW
- M5 - Spindle stop
- M6 - Tool change
- M7 - Mist coolant
- M8 - Flood coolant
- M9 - Coolant off
- M30 - Program end and reset
- M48 - Enable overrides
- M49 - Disable overrides

## Usage

```cpp
#include <ngc_parser.h>

ngc_parser::Parser parser;
std::vector<ngc_parser::CanonCommand> commands;

// Parse a single line
auto result = parser.parseLine("G0 X10 Y20", commands);
if (!result.success) {
    std::cerr << "Error: " << result.message << std::endl;
}

// Parse a file
result = parser.parseFile("part.ngc", commands);

// Process commands
for (const auto& cmd : commands) {
    switch (cmd.type) {
        case ngc_parser::CanonCommandType::STRAIGHT_TRAVERSE:
            // Handle rapid move
            break;
        case ngc_parser::CanonCommandType::STRAIGHT_FEED:
            // Handle linear feed
            break;
        // ...
    }
}
```

## Architecture

```
G-Code Text
     │
     ▼
┌─────────────┐
│   Lexer     │  Tokenize into words (G1, X10.5, etc.)
└─────────────┘
     │
     ▼
┌─────────────┐
│   Parser    │  Build block structure, validate
└─────────────┘
     │
     ▼
┌─────────────┐
│ Interpreter │  Apply modal state, calculate positions
└─────────────┘
     │
     ▼
Canonical Commands
```

The parser outputs canonical commands (STRAIGHT_TRAVERSE, STRAIGHT_FEED, ARC_FEED, etc.)
that can be consumed by a trajectory planner or motion controller.
