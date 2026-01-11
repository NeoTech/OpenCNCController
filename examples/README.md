# Examples

This directory contains example applications demonstrating how to use the OpenCNC libraries.

## Win32 Native

Location: `win32/`

A minimal Windows application using only the Win32 API. No external dependencies required.

```bash
cd win32
mkdir build && cd build
cmake -G Ninja ..
cmake --build .
```

## Qt5

Location: `qt5/`

A more complete example using Qt5 for the user interface.

```bash
cd qt5
mkdir build && cd build
cmake -G Ninja -DCMAKE_PREFIX_PATH=/path/to/qt5 ..
cmake --build .
```

## Ultralight (Coming Soon)

Location: `ultralight/`

A modern web-based UI using Ultralight HTML renderer.

## Console

Location: `console/`

A command-line interface for testing and automation.

## Features Demonstrated

- Connecting to CNC controller
- Real-time status updates
- Position display
- Jog controls
- Program loading and execution
- Feed/spindle overrides
- Error handling
- Logging

## Building All Examples

From the project root:

```bash
mkdir build && cd build
cmake -G Ninja -DBUILD_EXAMPLES=ON ..
cmake --build .
```
