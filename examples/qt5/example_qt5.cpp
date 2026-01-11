/*
 * OpenCNC Example - Qt5 Application
 * 
 * Demonstrates using the OpenCNC HMI library with Qt5.
 * 
 * Build with CMake + Qt5:
 *   cmake -DCMAKE_PREFIX_PATH=/path/to/qt5 ..
 *   cmake --build .
 * 
 * Copyright (c) 2024 OpenCNC Project
 * MIT License
 */

#include <QApplication>
#include <QMainWindow>
#include <QWidget>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QGridLayout>
#include <QLabel>
#include <QPushButton>
#include <QTextEdit>
#include <QGroupBox>
#include <QTimer>
#include <QMessageBox>
#include <QFileDialog>
#include <QSlider>
#include <QSpinBox>

// Include the OpenCNC HMI header
#include "../../hmi/include/opencnc_hmi.h"

using namespace cnc::hmi;

class CNCMainWindow : public QMainWindow {
    Q_OBJECT

public:
    CNCMainWindow(QWidget* parent = nullptr) : QMainWindow(parent) {
        setWindowTitle("OpenCNC - Qt5 Example");
        setMinimumSize(800, 600);
        
        createUI();
        setupController();
        
        // Update timer
        updateTimer_ = new QTimer(this);
        connect(updateTimer_, &QTimer::timeout, this, &CNCMainWindow::updateStatus);
        updateTimer_->start(50);  // 20 Hz update
    }
    
    ~CNCMainWindow() {
        controller_.disconnect();
    }

private slots:
    void onConnect() {
        QString port = "COM3";  // TODO: Port selector
        if (controller_.connect(port.toStdString())) {
            log("Connected to " + port);
        }
    }
    
    void onDisconnect() {
        controller_.disconnect();
        log("Disconnected");
    }
    
    void onHome() {
        controller_.home();
        log("Homing started");
    }
    
    void onStart() {
        controller_.start();
        log("Program started");
    }
    
    void onPause() {
        controller_.pause();
        log("Program paused");
    }
    
    void onStop() {
        controller_.stop();
        log("Program stopped");
    }
    
    void onLoadFile() {
        QString filename = QFileDialog::getOpenFileName(
            this, "Open G-code File", QString(),
            "G-code Files (*.nc *.ngc *.gcode);;All Files (*)");
        
        if (!filename.isEmpty()) {
            if (controller_.loadProgram(filename.toStdString())) {
                log("Loaded: " + filename);
            } else {
                QMessageBox::warning(this, "Error", "Failed to load file");
            }
        }
    }
    
    void onFeedOverrideChanged(int value) {
        controller_.setFeedOverride(value);
        feedOverrideLabel_->setText(QString("%1%").arg(value));
    }
    
    void onSpindleOverrideChanged(int value) {
        controller_.setSpindleOverride(value);
        spindleOverrideLabel_->setText(QString("%1%").arg(value));
    }
    
    void onJogXPlus() { controller_.jogStart(0, 1000.0); }
    void onJogXMinus() { controller_.jogStart(0, -1000.0); }
    void onJogYPlus() { controller_.jogStart(1, 1000.0); }
    void onJogYMinus() { controller_.jogStart(1, -1000.0); }
    void onJogZPlus() { controller_.jogStart(2, 500.0); }
    void onJogZMinus() { controller_.jogStart(2, -500.0); }
    void onJogStop() { controller_.jogStop(); }
    
    void updateStatus() {
        Controller::Status status = controller_.getStatus();
        
        // State
        QString stateStr;
        QString stateColor;
        switch (status.state) {
            case MachineState::Disconnected:
                stateStr = "Disconnected"; stateColor = "gray"; break;
            case MachineState::Idle:
                stateStr = "Idle"; stateColor = "green"; break;
            case MachineState::Running:
                stateStr = "Running"; stateColor = "blue"; break;
            case MachineState::Paused:
                stateStr = "Paused"; stateColor = "orange"; break;
            case MachineState::Homing:
                stateStr = "Homing"; stateColor = "blue"; break;
            case MachineState::Jogging:
                stateStr = "Jogging"; stateColor = "cyan"; break;
            case MachineState::Alarm:
                stateStr = "ALARM"; stateColor = "red"; break;
            case MachineState::Probing:
                stateStr = "Probing"; stateColor = "purple"; break;
        }
        stateLabel_->setText(stateStr);
        stateLabel_->setStyleSheet(QString("QLabel { color: %1; font-weight: bold; }").arg(stateColor));
        
        // Position
        posXLabel_->setText(QString::number(status.position.x, 'f', 3));
        posYLabel_->setText(QString::number(status.position.y, 'f', 3));
        posZLabel_->setText(QString::number(status.position.z, 'f', 3));
        
        // Feedrate
        feedrateLabel_->setText(QString("%1 mm/min").arg(status.feedrate, 0, 'f', 0));
        
        // Spindle
        spindleLabel_->setText(QString("%1 RPM (%2)")
            .arg(status.spindleSpeed)
            .arg(status.spindleOn ? "ON" : "OFF"));
    }

private:
    void createUI() {
        QWidget* central = new QWidget(this);
        setCentralWidget(central);
        
        QHBoxLayout* mainLayout = new QHBoxLayout(central);
        
        // Left panel - Status and controls
        QVBoxLayout* leftPanel = new QVBoxLayout();
        
        // Status group
        QGroupBox* statusGroup = new QGroupBox("Status");
        QGridLayout* statusLayout = new QGridLayout(statusGroup);
        
        statusLayout->addWidget(new QLabel("State:"), 0, 0);
        stateLabel_ = new QLabel("Disconnected");
        statusLayout->addWidget(stateLabel_, 0, 1);
        
        statusLayout->addWidget(new QLabel("X:"), 1, 0);
        posXLabel_ = new QLabel("0.000");
        statusLayout->addWidget(posXLabel_, 1, 1);
        
        statusLayout->addWidget(new QLabel("Y:"), 2, 0);
        posYLabel_ = new QLabel("0.000");
        statusLayout->addWidget(posYLabel_, 2, 1);
        
        statusLayout->addWidget(new QLabel("Z:"), 3, 0);
        posZLabel_ = new QLabel("0.000");
        statusLayout->addWidget(posZLabel_, 3, 1);
        
        statusLayout->addWidget(new QLabel("Feed:"), 4, 0);
        feedrateLabel_ = new QLabel("0 mm/min");
        statusLayout->addWidget(feedrateLabel_, 4, 1);
        
        statusLayout->addWidget(new QLabel("Spindle:"), 5, 0);
        spindleLabel_ = new QLabel("0 RPM");
        statusLayout->addWidget(spindleLabel_, 5, 1);
        
        leftPanel->addWidget(statusGroup);
        
        // Control buttons
        QGroupBox* controlGroup = new QGroupBox("Control");
        QGridLayout* controlLayout = new QGridLayout(controlGroup);
        
        QPushButton* connectBtn = new QPushButton("Connect");
        QPushButton* disconnectBtn = new QPushButton("Disconnect");
        QPushButton* homeBtn = new QPushButton("Home");
        QPushButton* loadBtn = new QPushButton("Load File");
        QPushButton* startBtn = new QPushButton("Start");
        QPushButton* pauseBtn = new QPushButton("Pause");
        QPushButton* stopBtn = new QPushButton("Stop");
        
        controlLayout->addWidget(connectBtn, 0, 0);
        controlLayout->addWidget(disconnectBtn, 0, 1);
        controlLayout->addWidget(homeBtn, 1, 0);
        controlLayout->addWidget(loadBtn, 1, 1);
        controlLayout->addWidget(startBtn, 2, 0);
        controlLayout->addWidget(pauseBtn, 2, 1);
        controlLayout->addWidget(stopBtn, 3, 0, 1, 2);
        
        connect(connectBtn, &QPushButton::clicked, this, &CNCMainWindow::onConnect);
        connect(disconnectBtn, &QPushButton::clicked, this, &CNCMainWindow::onDisconnect);
        connect(homeBtn, &QPushButton::clicked, this, &CNCMainWindow::onHome);
        connect(loadBtn, &QPushButton::clicked, this, &CNCMainWindow::onLoadFile);
        connect(startBtn, &QPushButton::clicked, this, &CNCMainWindow::onStart);
        connect(pauseBtn, &QPushButton::clicked, this, &CNCMainWindow::onPause);
        connect(stopBtn, &QPushButton::clicked, this, &CNCMainWindow::onStop);
        
        leftPanel->addWidget(controlGroup);
        
        // Overrides
        QGroupBox* overrideGroup = new QGroupBox("Overrides");
        QGridLayout* overrideLayout = new QGridLayout(overrideGroup);
        
        overrideLayout->addWidget(new QLabel("Feed:"), 0, 0);
        QSlider* feedSlider = new QSlider(Qt::Horizontal);
        feedSlider->setRange(0, 200);
        feedSlider->setValue(100);
        overrideLayout->addWidget(feedSlider, 0, 1);
        feedOverrideLabel_ = new QLabel("100%");
        overrideLayout->addWidget(feedOverrideLabel_, 0, 2);
        
        overrideLayout->addWidget(new QLabel("Spindle:"), 1, 0);
        QSlider* spindleSlider = new QSlider(Qt::Horizontal);
        spindleSlider->setRange(0, 200);
        spindleSlider->setValue(100);
        overrideLayout->addWidget(spindleSlider, 1, 1);
        spindleOverrideLabel_ = new QLabel("100%");
        overrideLayout->addWidget(spindleOverrideLabel_, 1, 2);
        
        connect(feedSlider, &QSlider::valueChanged, this, &CNCMainWindow::onFeedOverrideChanged);
        connect(spindleSlider, &QSlider::valueChanged, this, &CNCMainWindow::onSpindleOverrideChanged);
        
        leftPanel->addWidget(overrideGroup);
        
        // Jog controls
        QGroupBox* jogGroup = new QGroupBox("Jog");
        QGridLayout* jogLayout = new QGridLayout(jogGroup);
        
        QPushButton* jogXPlus = new QPushButton("X+");
        QPushButton* jogXMinus = new QPushButton("X-");
        QPushButton* jogYPlus = new QPushButton("Y+");
        QPushButton* jogYMinus = new QPushButton("Y-");
        QPushButton* jogZPlus = new QPushButton("Z+");
        QPushButton* jogZMinus = new QPushButton("Z-");
        
        jogLayout->addWidget(jogYPlus, 0, 1);
        jogLayout->addWidget(jogXMinus, 1, 0);
        jogLayout->addWidget(jogXPlus, 1, 2);
        jogLayout->addWidget(jogYMinus, 2, 1);
        jogLayout->addWidget(jogZPlus, 0, 3);
        jogLayout->addWidget(jogZMinus, 2, 3);
        
        // Use press/release for continuous jog
        connect(jogXPlus, &QPushButton::pressed, this, &CNCMainWindow::onJogXPlus);
        connect(jogXPlus, &QPushButton::released, this, &CNCMainWindow::onJogStop);
        connect(jogXMinus, &QPushButton::pressed, this, &CNCMainWindow::onJogXMinus);
        connect(jogXMinus, &QPushButton::released, this, &CNCMainWindow::onJogStop);
        connect(jogYPlus, &QPushButton::pressed, this, &CNCMainWindow::onJogYPlus);
        connect(jogYPlus, &QPushButton::released, this, &CNCMainWindow::onJogStop);
        connect(jogYMinus, &QPushButton::pressed, this, &CNCMainWindow::onJogYMinus);
        connect(jogYMinus, &QPushButton::released, this, &CNCMainWindow::onJogStop);
        connect(jogZPlus, &QPushButton::pressed, this, &CNCMainWindow::onJogZPlus);
        connect(jogZPlus, &QPushButton::released, this, &CNCMainWindow::onJogStop);
        connect(jogZMinus, &QPushButton::pressed, this, &CNCMainWindow::onJogZMinus);
        connect(jogZMinus, &QPushButton::released, this, &CNCMainWindow::onJogStop);
        
        leftPanel->addWidget(jogGroup);
        leftPanel->addStretch();
        
        mainLayout->addLayout(leftPanel);
        
        // Right panel - Log
        QVBoxLayout* rightPanel = new QVBoxLayout();
        
        QLabel* logLabel = new QLabel("Log");
        rightPanel->addWidget(logLabel);
        
        logText_ = new QTextEdit();
        logText_->setReadOnly(true);
        rightPanel->addWidget(logText_);
        
        mainLayout->addLayout(rightPanel, 1);
    }
    
    void setupController() {
        controller_.setErrorCallback([this](int code, const std::string& msg) {
            QMessageBox::warning(this, "Error", QString::fromStdString(msg));
        });
        
        controller_.setLogCallback([this](const std::string& msg) {
            log(QString::fromStdString(msg));
        });
    }
    
    void log(const QString& message) {
        logText_->append(message);
    }
    
    Controller controller_;
    QTimer* updateTimer_;
    
    QLabel* stateLabel_;
    QLabel* posXLabel_;
    QLabel* posYLabel_;
    QLabel* posZLabel_;
    QLabel* feedrateLabel_;
    QLabel* spindleLabel_;
    QLabel* feedOverrideLabel_;
    QLabel* spindleOverrideLabel_;
    QTextEdit* logText_;
};

int main(int argc, char* argv[]) {
    QApplication app(argc, argv);
    
    CNCMainWindow window;
    window.show();
    
    return app.exec();
}

#include "example_qt5.moc"
