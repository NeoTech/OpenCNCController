# OpenCNC HMI - CNC Controller Interface

A cross-platform CNC control library with Qt6 as the primary UI framework. Runs on embedded Linux (Wayland) for dedicated control panels or Windows for development/desktop use.

## Platform Support

| Platform | Status | Use Case |
|----------|--------|----------|
| **Linux + Qt6 + Wayland** | Primary | Embedded control panels (LattePanda, RPi5) |
| **Windows + Qt6** | Supported | Desktop development, operator stations |
| **Windows + Win32** | Supported | Lightweight native apps |

## Architecture

```
┌─────────────────────────────────────────────────────────────┐
│  Qt6 HMI Application (Linux Wayland / Windows)              │
│  ┌──────────────┐  ┌───────────────┐  ┌──────────────────┐ │
│  │ QML UI       │  │ opencnc_hmi.h │  │ Trajectory       │ │
│  │ Touch/Mouse  │◄─┤ Controller    │◄─┤ Planner          │ │
│  │ Visualization│  │ Interface     │  │ ngc_parser       │ │
│  └──────────────┘  └───────────────┘  └──────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
                        Ethernet (TCP/UDP)
                              │
                              ▼
                    ESP32-P4 Controller
```

## Quick Start

### Linux Embedded (Recommended for Production)

See [linux/README.md](linux/README.md) for:
- Buildroot minimal image (~100MB, 3s boot)
- Yocto enterprise build
- Arch Linux quick setup
- PREEMPT_RT kernel configuration
- Wayland kiosk mode setup

### Windows Development

```cpp
// In ONE source file:
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"

// In other files, just:
#include "opencnc_hmi.h"
```

## Features

- **Header-Only Core**: `opencnc_hmi.h` - just include and go
- **Framework Agnostic**: Works with Qt6, Qt5, Win32, or any C++ framework
- **Thread-Safe**: All public APIs are thread-safe
- **Ethernet Communication**: TCP/UDP to ESP32-P4 controller
- **Real-Time Feedback**: 50Hz+ status updates via callbacks or polling

## Integration Examples

### Qt6 (Primary)

```cpp
// main.cpp
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"
#include <QGuiApplication>
#include <QQmlApplicationEngine>
#include <QQmlContext>

class CNCController : public QObject {
    Q_OBJECT
    Q_PROPERTY(double xPos READ xPos NOTIFY positionChanged)
    Q_PROPERTY(double yPos READ yPos NOTIFY positionChanged)
    Q_PROPERTY(double zPos READ zPos NOTIFY positionChanged)
    Q_PROPERTY(QString state READ state NOTIFY stateChanged)
    
    opencnc::Controller m_cnc;
    
public:
    CNCController() {
        m_cnc.setStatusCallback([this](const opencnc::MachineStatus& s) {
            emit positionChanged();
            emit stateChanged();
        });
    }
    
    Q_INVOKABLE bool connect(const QString& address, int port) {
        return m_cnc.connect(address.toStdString(), port);
    }
    
    Q_INVOKABLE void jogStart(int axis, double velocity) {
        m_cnc.jogStart(axis, velocity);
    }
    
    Q_INVOKABLE void jogStop(int axis) {
        m_cnc.jogStop(axis);
    }
    
    Q_INVOKABLE void emergencyStop() {
        m_cnc.emergencyStop();
    }
    
    double xPos() const { return m_cnc.getCurrentPosition().x; }
    double yPos() const { return m_cnc.getCurrentPosition().y; }
    double zPos() const { return m_cnc.getCurrentPosition().z; }
    QString state() const { 
        return QString::fromStdString(opencnc::stateToString(m_cnc.getState())); 
    }
    
signals:
    void positionChanged();
    void stateChanged();
};

int main(int argc, char *argv[]) {
    QGuiApplication app(argc, argv);
    QQmlApplicationEngine engine;
    
    CNCController cnc;
    engine.rootContext()->setContextProperty("cnc", &cnc);
    engine.load(QUrl("qrc:/main.qml"));
    
    return app.exec();
}
```

```qml
// main.qml
import QtQuick 2.15
import QtQuick.Controls 2.15

ApplicationWindow {
    visible: true
    width: 1024
    height: 600
    title: "OpenCNC"
    
    Column {
        anchors.centerIn: parent
        spacing: 20
        
        // DRO Display
        Row {
            spacing: 40
            DroDisplay { axis: "X"; value: cnc.xPos }
            DroDisplay { axis: "Y"; value: cnc.yPos }
            DroDisplay { axis: "Z"; value: cnc.zPos }
        }
        
        // Jog Buttons
        Grid {
            columns: 3
            spacing: 10
            
            Button { text: "Y+"; onPressed: cnc.jogStart(1, 1000); onReleased: cnc.jogStop(1) }
            Button { }
            Button { text: "Z+"; onPressed: cnc.jogStart(2, 500); onReleased: cnc.jogStop(2) }
            
            Button { text: "X-"; onPressed: cnc.jogStart(0, -1000); onReleased: cnc.jogStop(0) }
            Button { text: "X+"; onPressed: cnc.jogStart(0, 1000); onReleased: cnc.jogStop(0) }
            Button { text: "Z-"; onPressed: cnc.jogStart(2, -500); onReleased: cnc.jogStop(2) }
            
            Button { }
            Button { text: "Y-"; onPressed: cnc.jogStart(1, -1000); onReleased: cnc.jogStop(1) }
            Button { }
        }
        
        // E-Stop
        Button {
            text: "EMERGENCY STOP"
            palette.button: "red"
            onClicked: cnc.emergencyStop()
        }
    }
    
    Component.onCompleted: {
        cnc.connect("192.168.4.10", 5000)
    }
}
```

### Win32 Native

```cpp
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"
#include <windows.h>

opencnc::Controller g_cnc;

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE:
        g_cnc.connect("192.168.4.10", 5000);
        g_cnc.setStatusCallback([hwnd](const opencnc::MachineStatus& status) {
            PostMessage(hwnd, WM_USER + 1, 0, 0);
        });
        break;
        
    case WM_USER + 1: {
        auto pos = g_cnc.getCurrentPosition();
        // Update DRO display...
        break;
    }
    
    case WM_KEYDOWN:
        if (wParam == VK_ESCAPE) {
            g_cnc.emergencyStop();
        }
        break;
    }
    return DefWindowProc(hwnd, msg, wParam, lParam);
}
```


## API Reference

### Connection

```cpp
bool connect(const std::string& address, int port = 5000);
void disconnect();
bool isConnected() const;
```

### Program Control

```cpp
bool loadProgram(const std::string& filename);
bool start();
bool pause();
bool resume();
bool stop();
bool reset();
```

### Manual Control

```cpp
bool jogStart(int axis, double velocity, JogMode mode = JogMode::CONTINUOUS);
bool jogStop(int axis);
bool jogIncrement(int axis, double distance, double velocity);
bool homeAxis(int axis);
bool homeAll();
```

### Status

```cpp
MachineState getState() const;
MachineStatus getStatus() const;
Position getCurrentPosition() const;
Position getMachinePosition() const;
```

### Callbacks

```cpp
void setStatusCallback(StatusCallback callback);
void setErrorCallback(ErrorCallback callback);
void setLogCallback(LogCallback callback);
```

## Thread Safety

All public methods are thread-safe. The library uses internal mutexes to protect shared state.

Callbacks are invoked from the network thread - marshal to the UI thread:
- **Qt6**: `QMetaObject::invokeMethod()` with `Qt::QueuedConnection`
- **Win32**: `PostMessage()`

## Dependencies

### Linux (Embedded)
- PREEMPT_RT kernel (recommended)
- Qt6 with Wayland support
- Wayland compositor (Weston, Sway, or custom)
- See [linux/README.md](linux/README.md) for full setup

### Windows
- C++17 compiler (MSVC 2019+, MinGW-w64)
- Qt6 (optional, for Qt integration)
- Winsock2 for networking

## Directory Structure

```
hmi/
├── include/
│   └── opencnc_hmi.h       # Header-only library
├── linux/
│   ├── README.md           # Linux embedded setup guide
│   ├── buildroot/          # Buildroot external tree
│   ├── yocto/              # Yocto meta-layer
│   └── arch/               # Arch Linux setup scripts
└── README.md               # This file
```
