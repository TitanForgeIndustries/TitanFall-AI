#ifndef TITANFALL_AI_MAIN_CONTROLLER_HPP
#define TITANFALL_AI_MAIN_CONTROLLER_HPP

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>

// Include all subsystem headers
#include "sensor_fusion.hpp"
#include "intent_model.hpp"
#include "torque_controller.hpp"
#include "energy_manager.hpp"
#include "fault_detection.hpp"
#include "thermal_manager.hpp"

namespace titanfall_ai {

enum class SystemState {
    INITIALIZING,
    READY,
    RUNNING,
    PAUSED,
    ERROR,
    EMERGENCY_STOP,
    SHUTDOWN
};

enum class ControlMode {
    MANUAL,
    SEMI_AUTONOMOUS,
    FULLY_AUTONOMOUS,
    LEARNING_MODE
};

class MainController {
public:
    MainController();
    ~MainController() = default;

    // Initialize the entire system
    void initialize();
    
    // Main control loop
    void run();
    
    // System control
    void start();
    void stop();
    void pause();
    void resume();
    void emergencyStop();
    void shutdown();
    
    // State management
    SystemState getCurrentState() const;
    ControlMode getCurrentControlMode() const;
    void setControlMode(ControlMode mode);
    
    // Subsystem access
    std::shared_ptr<SensorFusion> getSensorFusion();
    std::shared_ptr<IntentModel> getIntentModel();
    std::shared_ptr<TorqueController> getTorqueController();
    std::shared_ptr<EnergyManager> getEnergyManager();
    std::shared_ptr<FaultDetection> getFaultDetection();
    std::shared_ptr<ThermalManager> getThermalManager();
    
    // System monitoring
    bool isSystemHealthy() const;
    std::vector<std::string> getActiveWarnings() const;
    std::vector<std::string> getActiveErrors() const;
    
    // Configuration
    void loadConfiguration(const std::string& config_path);
    void saveConfiguration(const std::string& config_path);
    void setParameter(const std::string& subsystem, const std::string& parameter, const std::string& value);

private:
    rclcpp::Node::SharedPtr node_;
    
    // Subsystems
    std::shared_ptr<SensorFusion> sensor_fusion_;
    std::shared_ptr<IntentModel> intent_model_;
    std::shared_ptr<TorqueController> torque_controller_;
    std::shared_ptr<EnergyManager> energy_manager_;
    std::shared_ptr<FaultDetection> fault_detection_;
    std::shared_ptr<ThermalManager> thermal_manager_;
    
    // System state
    SystemState current_state_;
    ControlMode current_control_mode_;
    bool system_initialized_;
    bool system_running_;
    
    // Configuration
    std::map<std::string, std::map<std::string, std::string>> configuration_;
    
    // Timing
    rclcpp::Time last_update_time_;
    double main_loop_rate_;
    
    // Internal methods
    void initializeSubsystems();
    void updateSystemState();
    void checkSystemHealth();
    void handleSystemErrors();
    void publishSystemStatus();
    void loadDefaultConfiguration();
    void validateConfiguration();
};

} // namespace titanfall_ai

#endif // TITANFALL_AI_MAIN_CONTROLLER_HPP
