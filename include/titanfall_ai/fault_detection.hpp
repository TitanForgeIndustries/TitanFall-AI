#ifndef TITANFALL_AI_FAULT_DETECTION_HPP
#define TITANFALL_AI_FAULT_DETECTION_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <queue>

namespace titanfall_ai {

enum class FaultType {
    NO_FAULT,
    SENSOR_FAILURE,
    ACTUATOR_FAILURE,
    COMMUNICATION_ERROR,
    POWER_FAULT,
    THERMAL_FAULT,
    MECHANICAL_FAILURE,
    SOFTWARE_ERROR,
    SAFETY_VIOLATION,
    CRITICAL_FAILURE
};

enum class FaultSeverity {
    INFO,
    WARNING,
    ERROR,
    CRITICAL,
    FATAL
};

struct FaultEvent {
    FaultType type;
    FaultSeverity severity;
    std::string component;
    std::string description;
    rclcpp::Time timestamp;
    std::map<std::string, double> parameters;
    bool is_resolved;
    double confidence;
};

class FaultDetection {
public:
    FaultDetection(rclcpp::Node::SharedPtr node);
    ~FaultDetection() = default;

    // Initialize fault detection system
    void initialize();
    
    // Main fault detection loop
    void update();
    
    // Input callbacks
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void sensorDataCallback(const std_msgs::msg::String::SharedPtr msg);
    void powerCallback(const std_msgs::msg::Float64::SharedPtr msg);
    
    // Fault detection methods
    void detectSensorFaults();
    void detectActuatorFaults();
    void detectCommunicationFaults();
    void detectPowerFaults();
    void detectThermalFaults();
    void detectMechanicalFaults();
    
    // Fault management
    std::vector<FaultEvent> getActiveFaults() const;
    std::vector<FaultEvent> getFaultHistory() const;
    bool hasActiveFaults() const;
    bool hasCriticalFaults() const;
    void acknowledgeFault(const std::string& fault_id);
    void resolveFault(const std::string& fault_id);
    
    // Configuration
    void setFaultThresholds(const std::string& component, const std::map<std::string, double>& thresholds);
    void setFaultSensitivity(double sensitivity);
    void enableFaultType(FaultType type, bool enable);

private:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr fault_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr fault_event_pub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr fault_count_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sensor_sub_;
    rclcpp::Subscription<std_msgs::msg::Float64>::SharedPtr power_sub_;
    
    // Fault state
    std::vector<FaultEvent> active_faults_;
    std::queue<FaultEvent> fault_history_;
    std::map<std::string, std::map<std::string, double>> fault_thresholds_;
    std::map<FaultType, bool> enabled_fault_types_;
    
    // Detection parameters
    double fault_sensitivity_;
    double detection_confidence_threshold_;
    int max_history_size_;
    
    // Component monitoring
    std::map<std::string, std::vector<double>> component_history_;
    std::map<std::string, rclcpp::Time> last_update_times_;
    
    // Timing
    rclcpp::Time last_detection_time_;
    double detection_rate_;
    
    // Internal methods
    void updateComponentHistory(const std::string& component, double value);
    bool checkThresholdViolation(const std::string& component, const std::string& parameter, double value);
    void addFaultEvent(const FaultEvent& fault);
    void removeResolvedFaults();
    void publishFaultStatus();
    void loadFaultConfiguration();
};

} // namespace titanfall_ai

#endif // TITANFALL_AI_FAULT_DETECTION_HPP
