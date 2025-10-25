#include "titanfall_ai/fault_detection.hpp"

namespace titanfall_ai {

FaultDetection::FaultDetection(rclcpp::Node::SharedPtr node)
    : node_(node), fault_sensitivity_(1.0), detection_confidence_threshold_(0.8),
      max_history_size_(1000), detection_rate_(10.0) {
    initialize();
}

void FaultDetection::initialize() {
    // Initialize publishers
    fault_status_pub_ = node_->create_publisher<std_msgs::msg::Bool>("fault_detected", 10);
    fault_event_pub_ = node_->create_publisher<std_msgs::msg::String>("fault_event", 10);
    fault_count_pub_ = node_->create_publisher<std_msgs::msg::Int32>("fault_count", 10);
    
    // Initialize subscribers
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&FaultDetection::jointStateCallback, this, std::placeholders::_1));
    
    sensor_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "sensor_data", 10,
        std::bind(&FaultDetection::sensorDataCallback, this, std::placeholders::_1));
    
    power_sub_ = node_->create_subscription<std_msgs::msg::Float64>(
        "power_data", 10,
        std::bind(&FaultDetection::powerCallback, this, std::placeholders::_1));
    
    // Initialize enabled fault types
    for (int i = static_cast<int>(FaultType::NO_FAULT); i <= static_cast<int>(FaultType::CRITICAL_FAILURE); ++i) {
        enabled_fault_types_[static_cast<FaultType>(i)] = true;
    }
    
    // Initialize timing
    last_detection_time_ = node_->now();
    
    loadFaultConfiguration();
    RCLCPP_INFO(node_->get_logger(), "Fault Detection initialized");
}

void FaultDetection::update() {
    auto current_time = node_->now();
    auto dt = (current_time - last_detection_time_).seconds();
    
    if (dt >= 1.0 / detection_rate_) {
        detectSensorFaults();
        detectActuatorFaults();
        detectCommunicationFaults();
        detectPowerFaults();
        detectThermalFaults();
        detectMechanicalFaults();
        
        removeResolvedFaults();
        publishFaultStatus();
        
        last_detection_time_ = current_time;
    }
}

void FaultDetection::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Monitor joint states for actuator faults
    for (size_t i = 0; i < msg->name.size(); ++i) {
        const std::string& joint_name = msg->name[i];
        
        // Update position history
        updateComponentHistory(joint_name + "_position", msg->position[i]);
        
        // Update velocity history
        updateComponentHistory(joint_name + "_velocity", msg->velocity[i]);
        
        // Update effort history
        updateComponentHistory(joint_name + "_effort", msg->effort[i]);
        
        // Check for position limits
        if (std::abs(msg->position[i]) > 3.14) {
            FaultEvent fault;
            fault.type = FaultType::MECHANICAL_FAILURE;
            fault.severity = FaultSeverity::WARNING;
            fault.component = joint_name;
            fault.description = "Joint position limit exceeded";
            fault.timestamp = node_->now();
            fault.is_resolved = false;
            fault.confidence = 0.9;
            addFaultEvent(fault);
        }
        
        // Check for velocity limits
        if (std::abs(msg->velocity[i]) > 10.0) {
            FaultEvent fault;
            fault.type = FaultType::ACTUATOR_FAILURE;
            fault.severity = FaultSeverity::ERROR;
            fault.component = joint_name;
            fault.description = "Joint velocity limit exceeded";
            fault.timestamp = node_->now();
            fault.is_resolved = false;
            fault.confidence = 0.8;
            addFaultEvent(fault);
        }
    }
}

void FaultDetection::sensorDataCallback(const std_msgs::msg::String::SharedPtr msg) {
    // Monitor sensor data for faults
    updateComponentHistory("sensor_data", 1.0); // Simplified monitoring
    
    // Check for sensor communication timeout
    auto current_time = node_->now();
    if (component_history_.find("sensor_data") != component_history_.end()) {
        auto& history = component_history_["sensor_data"];
        if (!history.empty()) {
            // Check if sensor data is stale
            if ((current_time - last_update_times_["sensor_data"]).seconds() > 1.0) {
                FaultEvent fault;
                fault.type = FaultType::SENSOR_FAILURE;
                fault.severity = FaultSeverity::ERROR;
                fault.component = "sensor_system";
                fault.description = "Sensor communication timeout";
                fault.timestamp = current_time;
                fault.is_resolved = false;
                fault.confidence = 0.9;
                addFaultEvent(fault);
            }
        }
    }
    
    last_update_times_["sensor_data"] = current_time;
}

void FaultDetection::powerCallback(const std_msgs::msg::Float64::SharedPtr msg) {
    // Monitor power system for faults
    updateComponentHistory("power_voltage", msg->data);
    
    // Check for power faults
    if (msg->data < 20.0) {
        FaultEvent fault;
        fault.type = FaultType::POWER_FAULT;
        fault.severity = FaultSeverity::CRITICAL;
        fault.component = "power_system";
        fault.description = "Low voltage detected";
        fault.timestamp = node_->now();
        fault.is_resolved = false;
        fault.confidence = 0.95;
        addFaultEvent(fault);
    }
    
    if (msg->data > 30.0) {
        FaultEvent fault;
        fault.type = FaultType::POWER_FAULT;
        fault.severity = FaultSeverity::CRITICAL;
        fault.component = "power_system";
        fault.description = "High voltage detected";
        fault.timestamp = node_->now();
        fault.is_resolved = false;
        fault.confidence = 0.95;
        addFaultEvent(fault);
    }
}

void FaultDetection::detectSensorFaults() {
    // Implement sensor fault detection logic
    // This would include checking sensor readings against expected ranges
    // and detecting communication failures
}

void FaultDetection::detectActuatorFaults() {
    // Implement actuator fault detection logic
    // This would include checking for stuck actuators, excessive current draw,
    // and position/velocity limit violations
}

void FaultDetection::detectCommunicationFaults() {
    // Implement communication fault detection logic
    // This would include checking for message timeouts and data corruption
}

void FaultDetection::detectPowerFaults() {
    // Implement power fault detection logic
    // This would include checking voltage levels, current draw, and battery health
}

void FaultDetection::detectThermalFaults() {
    // Implement thermal fault detection logic
    // This would include checking temperature sensors and thermal limits
}

void FaultDetection::detectMechanicalFaults() {
    // Implement mechanical fault detection logic
    // This would include checking for excessive vibration, unusual sounds,
    // and mechanical wear indicators
}

std::vector<FaultEvent> FaultDetection::getActiveFaults() const {
    std::vector<FaultEvent> active_faults;
    for (const auto& fault : active_faults_) {
        if (!fault.is_resolved) {
            active_faults.push_back(fault);
        }
    }
    return active_faults;
}

std::vector<FaultEvent> FaultDetection::getFaultHistory() const {
    std::vector<FaultEvent> history;
    auto temp_queue = fault_history_;
    
    while (!temp_queue.empty()) {
        history.push_back(temp_queue.front());
        temp_queue.pop();
    }
    
    return history;
}

bool FaultDetection::hasActiveFaults() const {
    return !active_faults_.empty();
}

bool FaultDetection::hasCriticalFaults() const {
    for (const auto& fault : active_faults_) {
        if (fault.severity == FaultSeverity::CRITICAL || fault.severity == FaultSeverity::FATAL) {
            return true;
        }
    }
    return false;
}

void FaultDetection::acknowledgeFault(const std::string& fault_id) {
    // Acknowledge a fault (implementation would depend on fault ID system)
    RCLCPP_INFO(node_->get_logger(), "Fault acknowledged: %s", fault_id.c_str());
}

void FaultDetection::resolveFault(const std::string& fault_id) {
    // Resolve a fault (implementation would depend on fault ID system)
    RCLCPP_INFO(node_->get_logger(), "Fault resolved: %s", fault_id.c_str());
}

void FaultDetection::setFaultThresholds(const std::string& component, const std::map<std::string, double>& thresholds) {
    fault_thresholds_[component] = thresholds;
}

void FaultDetection::setFaultSensitivity(double sensitivity) {
    fault_sensitivity_ = sensitivity;
}

void FaultDetection::enableFaultType(FaultType type, bool enable) {
    enabled_fault_types_[type] = enable;
}

void FaultDetection::updateComponentHistory(const std::string& component, double value) {
    component_history_[component].push_back(value);
    
    // Limit history size
    if (component_history_[component].size() > max_history_size_) {
        component_history_[component].erase(component_history_[component].begin());
    }
    
    last_update_times_[component] = node_->now();
}

bool FaultDetection::checkThresholdViolation(const std::string& component, const std::string& parameter, double value) {
    if (fault_thresholds_.find(component) != fault_thresholds_.end()) {
        const auto& thresholds = fault_thresholds_[component];
        if (thresholds.find(parameter) != thresholds.end()) {
            return std::abs(value) > thresholds.at(parameter);
        }
    }
    return false;
}

void FaultDetection::addFaultEvent(const FaultEvent& fault) {
    if (enabled_fault_types_[fault.type] && fault.confidence >= detection_confidence_threshold_) {
        active_faults_.push_back(fault);
        fault_history_.push(fault);
        
        if (fault_history_.size() > max_history_size_) {
            fault_history_.pop();
        }
        
        RCLCPP_WARN(node_->get_logger(), "Fault detected: %s - %s", 
                   fault.component.c_str(), fault.description.c_str());
    }
}

void FaultDetection::removeResolvedFaults() {
    active_faults_.erase(
        std::remove_if(active_faults_.begin(), active_faults_.end(),
                      [](const FaultEvent& fault) { return fault.is_resolved; }),
        active_faults_.end());
}

void FaultDetection::publishFaultStatus() {
    // Publish fault status
    std_msgs::msg::Bool status_msg;
    status_msg.data = hasActiveFaults();
    fault_status_pub_->publish(status_msg);
    
    // Publish fault count
    std_msgs::msg::Int32 count_msg;
    count_msg.data = static_cast<int>(active_faults_.size());
    fault_count_pub_->publish(count_msg);
    
    // Publish latest fault event
    if (!active_faults_.empty()) {
        const auto& latest_fault = active_faults_.back();
        std_msgs::msg::String event_msg;
        event_msg.data = latest_fault.component + ": " + latest_fault.description;
        fault_event_pub_->publish(event_msg);
    }
}

void FaultDetection::loadFaultConfiguration() {
    RCLCPP_INFO(node_->get_logger(), "Loading fault detection configuration");
}

} // namespace titanfall_ai
