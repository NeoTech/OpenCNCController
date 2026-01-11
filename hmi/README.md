# OpenCNC HMI - Header-Only CNC Controller Interface

A single-header library for integrating CNC machine control into Windows applications.

## Features

- **Header-Only**: Just include and go
- **Framework Agnostic**: Works with Win32, Qt5, Ultralight, or any C++ framework
- **Thread-Safe**: All public APIs are thread-safe
- **Comprehensive API**: Full CNC control including jog, MDI, program execution
- **Real-Time Feedback**: Callbacks or polling for machine status

## Quick Start

```cpp
// In ONE source file:
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"

// In other files, just:
#include "opencnc_hmi.h"
```

## Integration Examples

### Win32 Native

```cpp
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"
#include <windows.h>

opencnc::Controller g_cnc;

LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
    case WM_CREATE:
        g_cnc.connect("COM3");
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

### Qt5

```cpp
// mainwindow.cpp
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"
#include <QTimer>
#include <QMainWindow>

class MainWindow : public QMainWindow {
    opencnc::Controller m_cnc;
    QTimer* m_statusTimer;
    
public:
    MainWindow() {
        m_statusTimer = new QTimer(this);
        connect(m_statusTimer, &QTimer::timeout, this, &MainWindow::updateStatus);
        
        if (m_cnc.connect("COM3")) {
            m_statusTimer->start(20); // 50Hz update
        }
    }
    
private slots:
    void updateStatus() {
        auto pos = m_cnc.getCurrentPosition();
        ui->xDro->setText(QString::number(pos.x, 'f', 3));
        ui->yDro->setText(QString::number(pos.y, 'f', 3));
        ui->zDro->setText(QString::number(pos.z, 'f', 3));
    }
    
    void onJogXPlus() {
        m_cnc.jogStart(0, 1000.0);
    }
    
    void onJogXStop() {
        m_cnc.jogStop(0);
    }
};
```

### Ultralight (WebView)

```cpp
#define OPENCNC_HMI_IMPLEMENTATION
#include "opencnc_hmi.h"
#include <Ultralight/Ultralight.h>
#include <JavaScriptCore/JavaScript.h>

class CNCBridge {
    opencnc::Controller m_cnc;
    
public:
    // Expose to JavaScript
    JSValueRef connect(JSContextRef ctx, JSValueRef* args, size_t argc) {
        std::string port = JSStringToStdString(args[0]);
        bool result = m_cnc.connect(port);
        return JSValueMakeBoolean(ctx, result);
    }
    
    JSValueRef getPosition(JSContextRef ctx) {
        auto pos = m_cnc.getCurrentPosition();
        // Return as JS object
        JSObjectRef obj = JSObjectMake(ctx, nullptr, nullptr);
        JSObjectSetProperty(ctx, obj, "x", JSValueMakeNumber(ctx, pos.x), ...);
        return obj;
    }
};
```

## API Reference

### Connection

```cpp
bool connect(const std::string& connection_string);
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

Callbacks are invoked from the polling thread - if updating UI, marshal to the UI thread:
- Win32: `PostMessage()`
- Qt: `QMetaObject::invokeMethod()` with `Qt::QueuedConnection`
- Ultralight: Use the main update loop

## Dependencies

- C++17 compiler (GCC 7+, Clang 5+, MSVC 2017+)
- Windows: `setupapi.lib` for COM port enumeration
- Threads library (pthreads or Windows threads)
