/*
 * OpenCNC Example - Minimal Win32 Application
 * 
 * Demonstrates using the OpenCNC HMI library with native Win32 API.
 * 
 * Build with MinGW64:
 *   g++ -o example_win32.exe example_win32.cpp -mwindows -lsetupapi
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <commctrl.h>

// Include the OpenCNC HMI header
#include "../hmi/include/opencnc_hmi.h"

// Using the namespace for convenience
using namespace cnc::hmi;

// Global controller instance
static Controller* g_controller = nullptr;

// UI Controls
static HWND g_hwndStatus = nullptr;
static HWND g_hwndPosition = nullptr;
static HWND g_hwndFeedrate = nullptr;
static HWND g_hwndSpindle = nullptr;
static HWND g_hwndLog = nullptr;

// Update UI with current status
void UpdateStatus() {
    if (!g_controller) return;
    
    Controller::Status status = g_controller->getStatus();
    
    // State
    const char* stateStr = "Unknown";
    switch (status.state) {
        case MachineState::Disconnected: stateStr = "Disconnected"; break;
        case MachineState::Idle: stateStr = "Idle"; break;
        case MachineState::Running: stateStr = "Running"; break;
        case MachineState::Paused: stateStr = "Paused"; break;
        case MachineState::Homing: stateStr = "Homing"; break;
        case MachineState::Jogging: stateStr = "Jogging"; break;
        case MachineState::Alarm: stateStr = "ALARM"; break;
        case MachineState::Probing: stateStr = "Probing"; break;
    }
    SetWindowTextA(g_hwndStatus, stateStr);
    
    // Position
    char posStr[128];
    snprintf(posStr, sizeof(posStr), "X: %.3f  Y: %.3f  Z: %.3f",
             status.position.x, status.position.y, status.position.z);
    SetWindowTextA(g_hwndPosition, posStr);
    
    // Feedrate
    char feedStr[64];
    snprintf(feedStr, sizeof(feedStr), "%.0f mm/min (%d%%)",
             status.feedrate, status.feedOverride);
    SetWindowTextA(g_hwndFeedrate, feedStr);
    
    // Spindle
    char spindleStr[64];
    snprintf(spindleStr, sizeof(spindleStr), "%d RPM (%s)",
             status.spindleSpeed, status.spindleOn ? "ON" : "OFF");
    SetWindowTextA(g_hwndSpindle, spindleStr);
}

// Callbacks
void OnStatusUpdate(const Controller::Status& status) {
    // Post message to main thread for UI update
    PostMessage(GetActiveWindow(), WM_USER + 1, 0, 0);
}

void OnError(int code, const std::string& message) {
    MessageBoxA(nullptr, message.c_str(), "CNC Error", MB_OK | MB_ICONERROR);
}

void OnLog(const std::string& message) {
    if (g_hwndLog) {
        // Append to log
        int len = GetWindowTextLength(g_hwndLog);
        SendMessage(g_hwndLog, EM_SETSEL, len, len);
        SendMessageA(g_hwndLog, EM_REPLACESEL, FALSE, (LPARAM)message.c_str());
        SendMessageA(g_hwndLog, EM_REPLACESEL, FALSE, (LPARAM)"\r\n");
    }
}

// Window procedure
LRESULT CALLBACK WndProc(HWND hwnd, UINT msg, WPARAM wParam, LPARAM lParam) {
    switch (msg) {
        case WM_CREATE: {
            // Create status labels
            CreateWindowA("STATIC", "Status:", WS_CHILD | WS_VISIBLE,
                          10, 10, 60, 20, hwnd, nullptr, nullptr, nullptr);
            g_hwndStatus = CreateWindowA("STATIC", "Disconnected",
                          WS_CHILD | WS_VISIBLE | SS_SUNKEN,
                          80, 10, 100, 20, hwnd, nullptr, nullptr, nullptr);
            
            CreateWindowA("STATIC", "Position:", WS_CHILD | WS_VISIBLE,
                          10, 40, 60, 20, hwnd, nullptr, nullptr, nullptr);
            g_hwndPosition = CreateWindowA("STATIC", "X: 0.000  Y: 0.000  Z: 0.000",
                          WS_CHILD | WS_VISIBLE | SS_SUNKEN,
                          80, 40, 250, 20, hwnd, nullptr, nullptr, nullptr);
            
            CreateWindowA("STATIC", "Feed:", WS_CHILD | WS_VISIBLE,
                          10, 70, 60, 20, hwnd, nullptr, nullptr, nullptr);
            g_hwndFeedrate = CreateWindowA("STATIC", "0 mm/min",
                          WS_CHILD | WS_VISIBLE | SS_SUNKEN,
                          80, 70, 150, 20, hwnd, nullptr, nullptr, nullptr);
            
            CreateWindowA("STATIC", "Spindle:", WS_CHILD | WS_VISIBLE,
                          10, 100, 60, 20, hwnd, nullptr, nullptr, nullptr);
            g_hwndSpindle = CreateWindowA("STATIC", "0 RPM",
                          WS_CHILD | WS_VISIBLE | SS_SUNKEN,
                          80, 100, 150, 20, hwnd, nullptr, nullptr, nullptr);
            
            // Control buttons
            CreateWindowA("BUTTON", "Connect", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          10, 140, 80, 30, hwnd, (HMENU)1001, nullptr, nullptr);
            CreateWindowA("BUTTON", "Disconnect", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          100, 140, 80, 30, hwnd, (HMENU)1002, nullptr, nullptr);
            CreateWindowA("BUTTON", "Home", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          190, 140, 80, 30, hwnd, (HMENU)1003, nullptr, nullptr);
            
            CreateWindowA("BUTTON", "Start", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          10, 180, 80, 30, hwnd, (HMENU)1004, nullptr, nullptr);
            CreateWindowA("BUTTON", "Pause", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          100, 180, 80, 30, hwnd, (HMENU)1005, nullptr, nullptr);
            CreateWindowA("BUTTON", "Stop", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          190, 180, 80, 30, hwnd, (HMENU)1006, nullptr, nullptr);
            
            // Jog buttons
            CreateWindowA("BUTTON", "X+", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          340, 100, 40, 30, hwnd, (HMENU)2001, nullptr, nullptr);
            CreateWindowA("BUTTON", "X-", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          340, 140, 40, 30, hwnd, (HMENU)2002, nullptr, nullptr);
            CreateWindowA("BUTTON", "Y+", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          390, 100, 40, 30, hwnd, (HMENU)2003, nullptr, nullptr);
            CreateWindowA("BUTTON", "Y-", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          390, 140, 40, 30, hwnd, (HMENU)2004, nullptr, nullptr);
            CreateWindowA("BUTTON", "Z+", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          440, 100, 40, 30, hwnd, (HMENU)2005, nullptr, nullptr);
            CreateWindowA("BUTTON", "Z-", WS_CHILD | WS_VISIBLE | BS_PUSHBUTTON,
                          440, 140, 40, 30, hwnd, (HMENU)2006, nullptr, nullptr);
            
            // Log window
            CreateWindowA("STATIC", "Log:", WS_CHILD | WS_VISIBLE,
                          10, 220, 60, 20, hwnd, nullptr, nullptr, nullptr);
            g_hwndLog = CreateWindowExA(WS_EX_CLIENTEDGE, "EDIT", "",
                          WS_CHILD | WS_VISIBLE | WS_VSCROLL | ES_MULTILINE | ES_READONLY,
                          10, 240, 480, 150, hwnd, nullptr, nullptr, nullptr);
            
            // Create controller
            g_controller = new Controller();
            g_controller->setStatusCallback(OnStatusUpdate);
            g_controller->setErrorCallback(OnError);
            g_controller->setLogCallback(OnLog);
            
            // Start status timer
            SetTimer(hwnd, 1, 100, nullptr);
            
            return 0;
        }
        
        case WM_TIMER:
            if (wParam == 1) {
                UpdateStatus();
            }
            return 0;
        
        case WM_USER + 1:
            UpdateStatus();
            return 0;
        
        case WM_COMMAND: {
            int id = LOWORD(wParam);
            
            switch (id) {
                case 1001: // Connect
                    g_controller->connect("COM3");
                    break;
                case 1002: // Disconnect
                    g_controller->disconnect();
                    break;
                case 1003: // Home
                    g_controller->home();
                    break;
                case 1004: // Start
                    g_controller->start();
                    break;
                case 1005: // Pause
                    g_controller->pause();
                    break;
                case 1006: // Stop
                    g_controller->stop();
                    break;
                
                // Jog buttons
                case 2001: g_controller->jogStart(0, 1000.0); break;
                case 2002: g_controller->jogStart(0, -1000.0); break;
                case 2003: g_controller->jogStart(1, 1000.0); break;
                case 2004: g_controller->jogStart(1, -1000.0); break;
                case 2005: g_controller->jogStart(2, 500.0); break;
                case 2006: g_controller->jogStart(2, -500.0); break;
            }
            
            return 0;
        }
        
        case WM_DESTROY:
            KillTimer(hwnd, 1);
            if (g_controller) {
                g_controller->disconnect();
                delete g_controller;
                g_controller = nullptr;
            }
            PostQuitMessage(0);
            return 0;
    }
    
    return DefWindowProcA(hwnd, msg, wParam, lParam);
}

int WINAPI WinMain(HINSTANCE hInstance, HINSTANCE hPrevInstance,
                   LPSTR lpCmdLine, int nCmdShow) {
    (void)hPrevInstance;
    (void)lpCmdLine;
    
    // Initialize common controls
    INITCOMMONCONTROLSEX icc = { sizeof(icc), ICC_WIN95_CLASSES };
    InitCommonControlsEx(&icc);
    
    // Register window class
    WNDCLASSA wc = {};
    wc.lpfnWndProc = WndProc;
    wc.hInstance = hInstance;
    wc.hCursor = LoadCursor(nullptr, IDC_ARROW);
    wc.hbrBackground = (HBRUSH)(COLOR_WINDOW + 1);
    wc.lpszClassName = "OpenCNCWin32";
    RegisterClassA(&wc);
    
    // Create window
    HWND hwnd = CreateWindowA("OpenCNCWin32", "OpenCNC - Win32 Example",
                              WS_OVERLAPPEDWINDOW,
                              CW_USEDEFAULT, CW_USEDEFAULT, 520, 440,
                              nullptr, nullptr, hInstance, nullptr);
    
    ShowWindow(hwnd, nCmdShow);
    UpdateWindow(hwnd);
    
    // Message loop
    MSG msg;
    while (GetMessage(&msg, nullptr, 0, 0)) {
        TranslateMessage(&msg);
        DispatchMessage(&msg);
    }
    
    return (int)msg.wParam;
}
