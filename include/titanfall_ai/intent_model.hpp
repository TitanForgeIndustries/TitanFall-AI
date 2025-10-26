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
#include <deque>
#include <algorithm>
#include <numeric>
#include <cmath>
#include <fstream>

// TensorFlow Lite includes (optional - will use fallback if not available)
#ifdef USE_TFLITE
#include <tensorflow/lite/interpreter.h>
#include <tensorflow/lite/kernels/register.h>
#include <tensorflow/lite/model.h>
#include <tensorflow/lite/optional_debug_tools.h>
#endif

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

    // TensorFlow Lite model
#ifdef USE_TFLITE
    std::unique_ptr<tflite::FlatBufferModel> tflite_model_;
    std::unique_ptr<tflite::Interpreter> tflite_interpreter_;
    tflite::ops::builtin::BuiltinOpResolver tflite_resolver_;
#endif
    bool use_ml_model_;
    std::string model_path_;

    // Advanced feature extraction
    static constexpr size_t TEMPORAL_WINDOW_SIZE = 30;  // 3 seconds at 10Hz
    static constexpr size_t FEATURE_DIM = 20;  // Extended feature dimension
    std::deque<std::vector<double>> temporal_feature_buffer_;

    // Attention mechanism weights
    std::vector<double> attention_weights_;

    // Online learning
    struct ExperienceReplay {
        std::vector<double> features;
        IntentType intent;
        bool success;
        double reward;
        rclcpp::Time timestamp;
    };
    std::deque<ExperienceReplay> replay_buffer_;
    static constexpr size_t MAX_REPLAY_BUFFER_SIZE = 10000;

    // Q-learning parameters
    std::map<std::pair<IntentType, IntentType>, double> q_table_;
    double gamma_;  // Discount factor
    double epsilon_;  // Exploration rate

    // Internal methods
    std::vector<double> extractFeatures(const sensor_msgs::msg::JointState& joint_state,
                                      const geometry_msgs::msg::Pose& pose);
    std::vector<double> extractAdvancedFeatures(const sensor_msgs::msg::JointState& joint_state,
                                               const geometry_msgs::msg::Pose& pose);
    std::vector<double> applyTemporalConvolution(const std::deque<std::vector<double>>& temporal_data);
    std::vector<double> applyAttentionMechanism(const std::deque<std::vector<double>>& temporal_data);

    IntentCommand classifyIntent(const std::vector<double>& features);
    IntentCommand classifyIntentML(const std::vector<double>& features);
    IntentCommand classifyIntentRuleBased(const std::vector<double>& features);

    double calculateConfidence(const std::vector<double>& features, IntentType intent);
    void updatePatternDatabase(const std::vector<double>& features, IntentType intent);
    IntentCommand parseUserInput(const std::string& input);
    void publishIntentPrediction();

    // TensorFlow Lite helpers
    bool initializeTFLite();
    std::vector<float> preprocessFeaturesForML(const std::vector<double>& features);
    IntentType postprocessMLOutput(const std::vector<float>& output);

    // Online learning helpers
    void updateQTable(const ExperienceReplay& experience);
    double getQValue(IntentType current, IntentType next);
    void setQValue(IntentType current, IntentType next, double value);
    IntentType selectActionEpsilonGreedy(IntentType current_state);
    double calculateReward(const IntentCommand& intent, bool success);
};

} // namespace titanfall_ai

#endif // TITANFALL_AI_INTENT_MODEL_HPP
