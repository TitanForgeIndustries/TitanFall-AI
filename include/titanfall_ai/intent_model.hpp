#ifndef TITANFALL_AI_INTENT_MODEL_HPP
#define TITANFALL_AI_INTENT_MODEL_HPP

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <memory>
#include <vector>
#include <string>
#include <map>
#include <queue>

namespace titanfall_ai {

enum class IntentType {
    IDLE,
    WALK_FORWARD,
    WALK_BACKWARD,
    TURN_LEFT,
    TURN_RIGHT,
    JUMP,
    CROUCH,
    SPRINT,
    CLIMB,
    BALANCE,
    EMERGENCY_STOP
};

struct IntentCommand {
    IntentType type;
    double intensity;  // 0.0 to 1.0
    double duration;   // seconds
    std::map<std::string, double> parameters;
    rclcpp::Time timestamp;
};

class IntentModel {
public:
    IntentModel(rclcpp::Node::SharedPtr node);
    ~IntentModel() = default;

    // Initialize intent recognition system
    void initialize();
    
    // Main processing loop
    void process();
    
    // Input callbacks
    void userInputCallback(const std_msgs::msg::String::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg);
    
    // Intent prediction and recognition
    IntentCommand predictNextIntent() const;
    IntentCommand recognizeCurrentIntent() const;
    std::vector<IntentCommand> getIntentHistory() const;
    
    // Configuration
    void setPredictionHorizon(double horizon_seconds);
    void setConfidenceThreshold(double threshold);
    void enableIntentType(IntentType type, bool enable);
    
    // Learning and adaptation
    void updateModel(const IntentCommand& executed_intent, bool success);
    void loadModel(const std::string& model_path);
    void saveModel(const std::string& model_path);

private:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr intent_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr predicted_velocity_pub_;
    
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr user_input_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pose_sub_;
    
    // State variables
    std::queue<IntentCommand> intent_history_;
    std::map<IntentType, bool> enabled_intents_;
    IntentCommand current_intent_;
    IntentCommand predicted_intent_;
    
    // Model parameters
    double prediction_horizon_;
    double confidence_threshold_;
    double learning_rate_;
    
    // Pattern recognition
    std::vector<std::vector<double>> feature_history_;
    std::vector<IntentType> label_history_;
    
    // Timing
    rclcpp::Time last_prediction_time_;
    double prediction_rate_;
    
    // Internal methods
    std::vector<double> extractFeatures(const sensor_msgs::msg::JointState& joint_state,
                                      const geometry_msgs::msg::Pose& pose);
    IntentCommand classifyIntent(const std::vector<double>& features);
    double calculateConfidence(const std::vector<double>& features, IntentType intent);
    void updatePatternDatabase(const std::vector<double>& features, IntentType intent);
    IntentCommand parseUserInput(const std::string& input);
    void publishIntentPrediction();
};

} // namespace titanfall_ai

#endif // TITANFALL_AI_INTENT_MODEL_HPP
