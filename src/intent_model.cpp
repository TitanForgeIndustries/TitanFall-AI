#include "titanfall_ai/intent_model.hpp"

namespace titanfall_ai {

IntentModel::IntentModel(rclcpp::Node::SharedPtr node)
    : node_(node), prediction_horizon_(2.0), confidence_threshold_(0.7), 
      learning_rate_(0.01), prediction_rate_(10.0) {
    initialize();
}

void IntentModel::initialize() {
    // Initialize publishers
    intent_pub_ = node_->create_publisher<std_msgs::msg::String>("predicted_intent", 10);
    predicted_velocity_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("predicted_velocity", 10);
    
    // Initialize subscribers
    user_input_sub_ = node_->create_subscription<std_msgs::msg::String>(
        "user_input", 10,
        std::bind(&IntentModel::userInputCallback, this, std::placeholders::_1));
    
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&IntentModel::jointStateCallback, this, std::placeholders::_1));
    
    pose_sub_ = node_->create_subscription<geometry_msgs::msg::Pose>(
        "current_pose", 10,
        std::bind(&IntentModel::poseCallback, this, std::placeholders::_1));
    
    // Initialize enabled intents
    for (int i = static_cast<int>(IntentType::IDLE); i <= static_cast<int>(IntentType::EMERGENCY_STOP); ++i) {
        enabled_intents_[static_cast<IntentType>(i)] = true;
    }
    
    // Initialize timing
    last_prediction_time_ = node_->now();
    
    RCLCPP_INFO(node_->get_logger(), "Intent Model initialized");
}

void IntentModel::process() {
    auto current_time = node_->now();
    auto dt = (current_time - last_prediction_time_).seconds();
    
    if (dt >= 1.0 / prediction_rate_) {
        predicted_intent_ = predictNextIntent();
        publishIntentPrediction();
        last_prediction_time_ = current_time;
    }
}

void IntentModel::userInputCallback(const std_msgs::msg::String::SharedPtr msg) {
    IntentCommand parsed_intent = parseUserInput(msg->data);
    if (parsed_intent.type != IntentType::IDLE) {
        intent_history_.push(parsed_intent);
        if (intent_history_.size() > 100) {
            intent_history_.pop();
        }
    }
}

void IntentModel::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    // Extract features from joint state for pattern recognition
    std::vector<double> features = extractFeatures(*msg, geometry_msgs::msg::Pose());
    feature_history_.push_back(features);
    
    if (feature_history_.size() > 1000) {
        feature_history_.erase(feature_history_.begin());
    }
}

void IntentModel::poseCallback(const geometry_msgs::msg::Pose::SharedPtr msg) {
    // Update pose information for intent recognition
    // This would be used in feature extraction
}

IntentCommand IntentModel::predictNextIntent() const {
    IntentCommand prediction;
    prediction.type = IntentType::IDLE;
    prediction.intensity = 0.0;
    prediction.duration = 0.0;
    prediction.timestamp = node_->now();
    
    // Simple prediction based on recent history
    if (!intent_history_.empty()) {
        prediction = intent_history_.back();
        prediction.intensity *= 0.8; // Decay intensity over time
    }
    
    return prediction;
}

IntentCommand IntentModel::recognizeCurrentIntent() const {
    IntentCommand current;
    current.type = IntentType::IDLE;
    current.intensity = 0.0;
    current.duration = 0.0;
    current.timestamp = node_->now();
    
    // Simple recognition based on recent features
    if (!feature_history_.empty()) {
        const auto& latest_features = feature_history_.back();
        current = classifyIntent(latest_features);
    }
    
    return current;
}

std::vector<IntentCommand> IntentModel::getIntentHistory() const {
    std::vector<IntentCommand> history;
    auto temp_queue = intent_history_;
    
    while (!temp_queue.empty()) {
        history.push_back(temp_queue.front());
        temp_queue.pop();
    }
    
    return history;
}

void IntentModel::setPredictionHorizon(double horizon_seconds) {
    prediction_horizon_ = horizon_seconds;
}

void IntentModel::setConfidenceThreshold(double threshold) {
    confidence_threshold_ = threshold;
}

void IntentModel::enableIntentType(IntentType type, bool enable) {
    enabled_intents_[type] = enable;
}

void IntentModel::updateModel(const IntentCommand& executed_intent, bool success) {
    // Update learning model based on execution success
    // This would implement reinforcement learning in a real system
    RCLCPP_DEBUG(node_->get_logger(), "Updating model with executed intent: %d, success: %s",
                 static_cast<int>(executed_intent.type), success ? "true" : "false");
}

void IntentModel::loadModel(const std::string& model_path) {
    RCLCPP_INFO(node_->get_logger(), "Loading intent model from: %s", model_path.c_str());
    // Implement model loading
}

void IntentModel::saveModel(const std::string& model_path) {
    RCLCPP_INFO(node_->get_logger(), "Saving intent model to: %s", model_path.c_str());
    // Implement model saving
}

std::vector<double> IntentModel::extractFeatures(const sensor_msgs::msg::JointState& joint_state,
                                                const geometry_msgs::msg::Pose& pose) {
    std::vector<double> features;
    
    // Extract basic features from joint states
    for (const auto& position : joint_state.position) {
        features.push_back(position);
    }
    for (const auto& velocity : joint_state.velocity) {
        features.push_back(velocity);
    }
    
    // Add pose features
    features.push_back(pose.position.x);
    features.push_back(pose.position.y);
    features.push_back(pose.position.z);
    
    return features;
}

IntentCommand IntentModel::classifyIntent(const std::vector<double>& features) {
    IntentCommand intent;
    intent.type = IntentType::IDLE;
    intent.intensity = 0.0;
    intent.duration = 1.0;
    intent.timestamp = node_->now();
    
    // Simple classification based on feature patterns
    // In a real implementation, this would use machine learning
    if (features.size() > 0) {
        double avg_velocity = 0.0;
        for (size_t i = features.size() / 2; i < features.size(); ++i) {
            avg_velocity += std::abs(features[i]);
        }
        avg_velocity /= (features.size() / 2);
        
        if (avg_velocity > 0.1) {
            intent.type = IntentType::WALK_FORWARD;
            intent.intensity = std::min(avg_velocity, 1.0);
        }
    }
    
    return intent;
}

double IntentModel::calculateConfidence(const std::vector<double>& features, IntentType intent) {
    // Simple confidence calculation
    // In a real implementation, this would use trained models
    return 0.8; // Placeholder confidence
}

void IntentModel::updatePatternDatabase(const std::vector<double>& features, IntentType intent) {
    // Update pattern database for learning
    // This would implement pattern storage and retrieval
}

IntentCommand IntentModel::parseUserInput(const std::string& input) {
    IntentCommand intent;
    intent.type = IntentType::IDLE;
    intent.intensity = 1.0;
    intent.duration = 1.0;
    intent.timestamp = node_->now();
    
    // Simple text parsing
    std::string lower_input = input;
    std::transform(lower_input.begin(), lower_input.end(), lower_input.begin(), ::tolower);
    
    if (lower_input.find("walk") != std::string::npos) {
        intent.type = IntentType::WALK_FORWARD;
    } else if (lower_input.find("turn") != std::string::npos) {
        intent.type = IntentType::TURN_LEFT;
    } else if (lower_input.find("jump") != std::string::npos) {
        intent.type = IntentType::JUMP;
    } else if (lower_input.find("stop") != std::string::npos) {
        intent.type = IntentType::EMERGENCY_STOP;
    }
    
    return intent;
}

void IntentModel::publishIntentPrediction() {
    std_msgs::msg::String intent_msg;
    intent_msg.data = "Intent: " + std::to_string(static_cast<int>(predicted_intent_.type));
    intent_pub_->publish(intent_msg);
    
    geometry_msgs::msg::Twist velocity_msg;
    velocity_msg.linear.x = predicted_intent_.intensity;
    predicted_velocity_pub_->publish(velocity_msg);
}

} // namespace titanfall_ai
