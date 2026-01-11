/*
 * OpenCNC Configuration Test
 * 
 * Test program for configuration parsing.
 */

#include "cnc_config.h"
#include <iostream>
#include <iomanip>

using namespace cnc::config;

int main(int argc, char* argv[]) {
    std::string configFile = "machine.toml";
    
    if (argc > 1) {
        configFile = argv[1];
    }
    
    std::cout << "OpenCNC Configuration Parser Test\n";
    std::cout << "==================================\n\n";
    
    try {
        MachineConfig config = MachineConfig::loadFromFile(configFile);
        
        std::cout << "Machine: " << config.name << " v" << config.version << "\n\n";
        
        std::cout << "Kinematics:\n";
        std::cout << "  Type: " << config.kinematics.type << "\n";
        std::cout << "  Junction Deviation: " << config.kinematics.junctionDeviation << " mm\n";
        std::cout << "  Arc Tolerance: " << config.kinematics.arcTolerance << " mm\n";
        std::cout << "  Soft Limits: " << (config.kinematics.softLimitsEnabled ? "yes" : "no") << "\n";
        std::cout << "  Hard Limits: " << (config.kinematics.hardLimitsEnabled ? "yes" : "no") << "\n\n";
        
        std::cout << "Axes (" << config.axes.size() << "):\n";
        for (const auto& axis : config.axes) {
            std::cout << "  " << axis.name << ":\n";
            std::cout << "    Steps/mm: " << axis.stepsPerMm << "\n";
            std::cout << "    Max Velocity: " << axis.maxVelocity << " mm/min\n";
            std::cout << "    Max Acceleration: " << axis.maxAcceleration << " mm/s²\n";
            std::cout << "    Max Travel: " << axis.maxTravel << " mm\n";
            std::cout << "    Home Direction: " << (axis.homeDirection > 0 ? "+" : "-") << "\n";
        }
        std::cout << "\n";
        
        std::cout << "Spindle:\n";
        std::cout << "  Enabled: " << (config.spindle.enabled ? "yes" : "no") << "\n";
        std::cout << "  RPM Range: " << config.spindle.minRpm << " - " << config.spindle.maxRpm << "\n";
        std::cout << "  PWM Frequency: " << config.spindle.pwmFrequency << " Hz\n\n";
        
        std::cout << "Coolant:\n";
        std::cout << "  Mist: " << (config.coolant.mistEnabled ? "enabled" : "disabled") << "\n";
        std::cout << "  Flood: " << (config.coolant.floodEnabled ? "enabled" : "disabled") << "\n\n";
        
        std::cout << "Communication:\n";
        std::cout << "  Port: " << config.communication.port << "\n";
        std::cout << "  Baud Rate: " << config.communication.baudRate << "\n";
        std::cout << "  Status Poll Rate: " << config.communication.statusPollRate << " Hz\n\n";
        
        std::cout << "Configuration loaded successfully!\n";
        
    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 1;
    }
    
    return 0;
}
