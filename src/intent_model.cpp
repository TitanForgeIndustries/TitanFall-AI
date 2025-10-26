#include "titanfall_ai/intent_model.hpp"

namespace titanfall_ai {

IntentModel::IntentModel(rclcpp::Node::SharedPtr node)
    : node_(node), prediction_horizon_(2.0), confidence_threshold_(0.7),
      learning_rate_(0.01), prediction_rate_(10.0), use_ml_model_(false),
      gamma_(0.95), epsilon_(0.1) {
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
    // Extract advanced features from joint state for pattern recognition
    std::vector<double> features = extractAdvancedFeatures(*msg, geometry_msgs::msg::Pose());

    // Add to temporal buffer
    temporal_feature_buffer_.push_back(features);
    if (temporal_feature_buffer_.size() > TEMPORAL_WINDOW_SIZE) {
        temporal_feature_buffer_.pop_front();
    }

    // Apply temporal processing if we have enough history
    if (temporal_feature_buffer_.size() >= 3) {
        // Apply temporal convolution
        auto conv_features = applyTemporalConvolution(temporal_feature_buffer_);

        // Apply attention mechanism
        auto attended_features = applyAttentionMechanism(temporal_feature_buffer_);

        // Combine features (simple concatenation, could use more sophisticated fusion)
        std::vector<double> combined_features = features;
        combined_features.insert(combined_features.end(),
                                conv_features.begin(), conv_features.end());
        combined_features.insert(combined_features.end(),
                                attended_features.begin(), attended_features.end());

        feature_history_.push_back(combined_features);
    } else {
        feature_history_.push_back(features);
    }

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
    RCLCPP_DEBUG(node_->get_logger(), "Updating model with executed intent: %d, success: %s",
                 static_cast<int>(executed_intent.type), success ? "true" : "false");

    // Create experience replay entry
    if (!feature_history_.empty()) {
        ExperienceReplay experience;
        experience.features = feature_history_.back();
        experience.intent = executed_intent.type;
        experience.success = success;
        experience.reward = calculateReward(executed_intent, success);
        experience.timestamp = node_->now();

        // Add to replay buffer
        replay_buffer_.push_back(experience);
        if (replay_buffer_.size() > MAX_REPLAY_BUFFER_SIZE) {
            replay_buffer_.pop_front();
        }

        // Update Q-table
        updateQTable(experience);

        // Decay exploration rate
        epsilon_ = std::max(0.01, epsilon_ * 0.995);
    }
}

void IntentModel::loadModel(const std::string& model_path) {
    RCLCPP_INFO(node_->get_logger(), "Loading intent model from: %s", model_path.c_str());
    model_path_ = model_path;

#ifdef USE_TFLITE
    if (initializeTFLite()) {
        use_ml_model_ = true;
        RCLCPP_INFO(node_->get_logger(), "TensorFlow Lite model loaded successfully");
    } else {
        use_ml_model_ = false;
        RCLCPP_WARN(node_->get_logger(), "Failed to load TFLite model, using rule-based fallback");
    }
#else
    RCLCPP_WARN(node_->get_logger(), "TensorFlow Lite not available, using rule-based classification");
    use_ml_model_ = false;
#endif
}

void IntentModel::saveModel(const std::string& model_path) {
    RCLCPP_INFO(node_->get_logger(), "Saving intent model to: %s", model_path.c_str());

    // Save Q-table and experience replay buffer
    std::ofstream file(model_path + "_qtable.dat", std::ios::binary);
    if (file.is_open()) {
        size_t q_table_size = q_table_.size();
        file.write(reinterpret_cast<const char*>(&q_table_size), sizeof(q_table_size));

        for (const auto& entry : q_table_) {
            int current = static_cast<int>(entry.first.first);
            int next = static_cast<int>(entry.first.second);
            double value = entry.second;
            file.write(reinterpret_cast<const char*>(&current), sizeof(current));
            file.write(reinterpret_cast<const char*>(&next), sizeof(next));
            file.write(reinterpret_cast<const char*>(&value), sizeof(value));
        }
        file.close();
        RCLCPP_INFO(node_->get_logger(), "Q-table saved successfully");
    }
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
    // Use ML model if available, otherwise fall back to rule-based
    if (use_ml_model_) {
        return classifyIntentML(features);
    } else {
        return classifyIntentRuleBased(features);
    }
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

// ============================================================================
// ADVANCED FEATURE EXTRACTION
// ============================================================================

std::vector<double> IntentModel::extractAdvancedFeatures(
    const sensor_msgs::msg::JointState& joint_state,
    const geometry_msgs::msg::Pose& pose) {

    std::vector<double> features;
    features.reserve(FEATURE_DIM);

    // 1. Joint positions (normalized)
    for (const auto& position : joint_state.position) {
        features.push_back(std::tanh(position));  // Normalize to [-1, 1]
    }

    // 2. Joint velocities (normalized)
    for (const auto& velocity : joint_state.velocity) {
        features.push_back(std::tanh(velocity / 10.0));  // Scale and normalize
    }

    // 3. Joint accelerations (computed from velocity history)
    if (feature_history_.size() >= 2) {
        const auto& prev_features = feature_history_[feature_history_.size() - 2];
        size_t vel_start = joint_state.position.size();
        for (size_t i = 0; i < joint_state.velocity.size(); ++i) {
            double accel = joint_state.velocity[i] - prev_features[vel_start + i];
            features.push_back(std::tanh(accel / 100.0));
        }
    } else {
        // Pad with zeros if not enough history
        for (size_t i = 0; i < joint_state.velocity.size(); ++i) {
            features.push_back(0.0);
        }
    }

    // 4. Pose features
    features.push_back(std::tanh(pose.position.x));
    features.push_back(std::tanh(pose.position.y));
    features.push_back(std::tanh(pose.position.z));

    // 5. Orientation (quaternion)
    features.push_back(pose.orientation.x);
    features.push_back(pose.orientation.y);
    features.push_back(pose.orientation.z);
    features.push_back(pose.orientation.w);

    // 6. Center of mass velocity (estimated from pose history)
    if (feature_history_.size() >= 2) {
        const auto& prev_features = feature_history_.back();
        size_t pose_idx = prev_features.size() - 7;  // Position is near end
        double vx = pose.position.x - prev_features[pose_idx];
        double vy = pose.position.y - prev_features[pose_idx + 1];
        double vz = pose.position.z - prev_features[pose_idx + 2];
        features.push_back(std::tanh(vx * 10.0));
        features.push_back(std::tanh(vy * 10.0));
        features.push_back(std::tanh(vz * 10.0));
    } else {
        features.push_back(0.0);
        features.push_back(0.0);
        features.push_back(0.0);
    }

    // Ensure consistent feature dimension
    while (features.size() < FEATURE_DIM) {
        features.push_back(0.0);
    }
    if (features.size() > FEATURE_DIM) {
        features.resize(FEATURE_DIM);
    }

    return features;
}

std::vector<double> IntentModel::applyTemporalConvolution(
    const std::deque<std::vector<double>>& temporal_data) {

    if (temporal_data.empty()) {
        return std::vector<double>(FEATURE_DIM, 0.0);
    }

    std::vector<double> conv_features(FEATURE_DIM, 0.0);

    // Simple 1D convolution with kernel size 3
    std::vector<double> kernel = {0.25, 0.5, 0.25};  // Gaussian-like kernel

    for (size_t feat_idx = 0; feat_idx < FEATURE_DIM; ++feat_idx) {
        double sum = 0.0;
        double weight_sum = 0.0;

        for (size_t t = 0; t < std::min(temporal_data.size(), size_t(3)); ++t) {
            if (temporal_data[temporal_data.size() - 1 - t].size() > feat_idx) {
                sum += temporal_data[temporal_data.size() - 1 - t][feat_idx] * kernel[t];
                weight_sum += kernel[t];
            }
        }

        conv_features[feat_idx] = weight_sum > 0 ? sum / weight_sum : 0.0;
    }

    return conv_features;
}

std::vector<double> IntentModel::applyAttentionMechanism(
    const std::deque<std::vector<double>>& temporal_data) {

    if (temporal_data.empty()) {
        return std::vector<double>(FEATURE_DIM, 0.0);
    }

    // Compute attention weights based on recency and variance
    std::vector<double> weights(temporal_data.size());
    double weight_sum = 0.0;

    for (size_t t = 0; t < temporal_data.size(); ++t) {
        // Exponential decay for older timesteps
        double recency_weight = std::exp(-0.1 * (temporal_data.size() - 1 - t));

        // Variance-based importance (higher variance = more important)
        double variance = 0.0;
        if (temporal_data[t].size() > 0) {
            double mean = std::accumulate(temporal_data[t].begin(),
                                         temporal_data[t].end(), 0.0) / temporal_data[t].size();
            for (double val : temporal_data[t]) {
                variance += (val - mean) * (val - mean);
            }
            variance /= temporal_data[t].size();
        }
        double importance_weight = 1.0 + variance;

        weights[t] = recency_weight * importance_weight;
        weight_sum += weights[t];
    }

    // Normalize weights
    for (auto& w : weights) {
        w /= weight_sum;
    }

    // Apply attention
    std::vector<double> attended_features(FEATURE_DIM, 0.0);
    for (size_t t = 0; t < temporal_data.size(); ++t) {
        for (size_t feat_idx = 0; feat_idx < std::min(temporal_data[t].size(), FEATURE_DIM); ++feat_idx) {
            attended_features[feat_idx] += weights[t] * temporal_data[t][feat_idx];
        }
    }

    attention_weights_ = weights;  // Store for visualization/debugging

    return attended_features;
}

// ============================================================================
// TENSORFLOW LITE INTEGRATION
// ============================================================================

bool IntentModel::initializeTFLite() {
#ifdef USE_TFLITE
    // Load the model
    tflite_model_ = tflite::FlatBufferModel::BuildFromFile(model_path_.c_str());
    if (!tflite_model_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to load TFLite model from: %s", model_path_.c_str());
        return false;
    }

    // Build the interpreter
    tflite::InterpreterBuilder builder(*tflite_model_, tflite_resolver_);
    builder(&tflite_interpreter_);
    if (!tflite_interpreter_) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to build TFLite interpreter");
        return false;
    }

    // Allocate tensors
    if (tflite_interpreter_->AllocateTensors() != kTfLiteOk) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to allocate TFLite tensors");
        return false;
    }

    // Set number of threads
    tflite_interpreter_->SetNumThreads(4);

    RCLCPP_INFO(node_->get_logger(), "TFLite model initialized successfully");
    return true;
#else
    return false;
#endif
}

std::vector<float> IntentModel::preprocessFeaturesForML(const std::vector<double>& features) {
    std::vector<float> processed(features.size());
    for (size_t i = 0; i < features.size(); ++i) {
        processed[i] = static_cast<float>(features[i]);
    }
    return processed;
}

IntentType IntentModel::postprocessMLOutput(const std::vector<float>& output) {
    // Find the class with highest probability
    auto max_it = std::max_element(output.begin(), output.end());
    int predicted_class = std::distance(output.begin(), max_it);

    // Map to IntentType (assumes output classes match IntentType enum order)
    if (predicted_class >= static_cast<int>(IntentType::IDLE) &&
        predicted_class <= static_cast<int>(IntentType::EMERGENCY_STOP)) {
        return static_cast<IntentType>(predicted_class);
    }

    return IntentType::IDLE;
}

IntentCommand IntentModel::classifyIntentML(const std::vector<double>& features) {
    IntentCommand intent;
    intent.type = IntentType::IDLE;
    intent.intensity = 0.0;
    intent.duration = 1.0;
    intent.timestamp = node_->now();

#ifdef USE_TFLITE
    if (!tflite_interpreter_) {
        return classifyIntentRuleBased(features);
    }

    // Preprocess features
    auto processed_features = preprocessFeaturesForML(features);

    // Get input tensor
    int input_idx = tflite_interpreter_->inputs()[0];
    TfLiteTensor* input_tensor = tflite_interpreter_->tensor(input_idx);

    // Copy features to input tensor
    float* input_data = input_tensor->data.f;
    std::copy(processed_features.begin(), processed_features.end(), input_data);

    // Run inference
    if (tflite_interpreter_->Invoke() != kTfLiteOk) {
        RCLCPP_ERROR(node_->get_logger(), "Failed to invoke TFLite interpreter");
        return classifyIntentRuleBased(features);
    }

    // Get output tensor
    int output_idx = tflite_interpreter_->outputs()[0];
    TfLiteTensor* output_tensor = tflite_interpreter_->tensor(output_idx);
    float* output_data = output_tensor->data.f;

    // Convert to vector
    std::vector<float> output(output_data, output_data + output_tensor->bytes / sizeof(float));

    // Postprocess
    intent.type = postprocessMLOutput(output);

    // Set intensity based on confidence
    auto max_it = std::max_element(output.begin(), output.end());
    intent.intensity = std::min(1.0, static_cast<double>(*max_it));

    RCLCPP_DEBUG(node_->get_logger(), "ML predicted intent: %d with confidence: %.2f",
                 static_cast<int>(intent.type), intent.intensity);
#else
    return classifyIntentRuleBased(features);
#endif

    return intent;
}

IntentCommand IntentModel::classifyIntentRuleBased(const std::vector<double>& features) {
    IntentCommand intent;
    intent.type = IntentType::IDLE;
    intent.intensity = 0.0;
    intent.duration = 1.0;
    intent.timestamp = node_->now();

    // Enhanced rule-based classification
    if (features.size() > 0) {
        double avg_velocity = 0.0;
        size_t vel_count = 0;

        // Calculate average velocity magnitude
        for (size_t i = features.size() / 2; i < features.size() && i < features.size() - 7; ++i) {
            avg_velocity += std::abs(features[i]);
            vel_count++;
        }
        if (vel_count > 0) {
            avg_velocity /= vel_count;
        }

        // Classify based on velocity patterns
        if (avg_velocity > 0.5) {
            intent.type = IntentType::SPRINT;
            intent.intensity = std::min(avg_velocity, 1.0);
        } else if (avg_velocity > 0.2) {
            intent.type = IntentType::WALK_FORWARD;
            intent.intensity = std::min(avg_velocity * 2.0, 1.0);
        } else if (avg_velocity > 0.05) {
            intent.type = IntentType::BALANCE;
            intent.intensity = 0.3;
        }
    }

    return intent;
}

// ============================================================================
// ONLINE LEARNING (Q-LEARNING)
// ============================================================================

void IntentModel::updateQTable(const ExperienceReplay& experience) {
    // Get current and next intent
    IntentType current_intent = experience.intent;

    // Predict next intent based on current features
    IntentCommand next_intent_cmd = classifyIntent(experience.features);
    IntentType next_intent = next_intent_cmd.type;

    // Get current Q-value
    double current_q = getQValue(current_intent, next_intent);

    // Calculate max Q-value for next state
    double max_next_q = 0.0;
    for (int i = static_cast<int>(IntentType::IDLE);
         i <= static_cast<int>(IntentType::EMERGENCY_STOP); ++i) {
        IntentType next_state = static_cast<IntentType>(i);
        double q = getQValue(next_intent, next_state);
        max_next_q = std::max(max_next_q, q);
    }

    // Q-learning update: Q(s,a) = Q(s,a) + α[r + γ*max(Q(s',a')) - Q(s,a)]
    double new_q = current_q + learning_rate_ *
                   (experience.reward + gamma_ * max_next_q - current_q);

    setQValue(current_intent, next_intent, new_q);

    RCLCPP_DEBUG(node_->get_logger(),
                 "Q-table updated: (%d,%d) = %.3f (reward: %.3f)",
                 static_cast<int>(current_intent),
                 static_cast<int>(next_intent),
                 new_q, experience.reward);
}

double IntentModel::getQValue(IntentType current, IntentType next) {
    auto key = std::make_pair(current, next);
    auto it = q_table_.find(key);
    if (it != q_table_.end()) {
        return it->second;
    }
    return 0.0;  // Default Q-value for unseen state-action pairs
}

void IntentModel::setQValue(IntentType current, IntentType next, double value) {
    auto key = std::make_pair(current, next);
    q_table_[key] = value;
}

IntentType IntentModel::selectActionEpsilonGreedy(IntentType current_state) {
    // Epsilon-greedy exploration
    double rand_val = static_cast<double>(rand()) / RAND_MAX;

    if (rand_val < epsilon_) {
        // Explore: random action
        int random_action = rand() % (static_cast<int>(IntentType::EMERGENCY_STOP) + 1);
        return static_cast<IntentType>(random_action);
    } else {
        // Exploit: choose best action based on Q-table
        IntentType best_action = IntentType::IDLE;
        double best_q = -std::numeric_limits<double>::infinity();

        for (int i = static_cast<int>(IntentType::IDLE);
             i <= static_cast<int>(IntentType::EMERGENCY_STOP); ++i) {
            IntentType action = static_cast<IntentType>(i);
            double q = getQValue(current_state, action);
            if (q > best_q) {
                best_q = q;
                best_action = action;
            }
        }

        return best_action;
    }
}

double IntentModel::calculateReward(const IntentCommand& intent, bool success) {
    // Reward function design
    double reward = 0.0;

    if (success) {
        // Base reward for successful execution
        reward = 1.0;

        // Bonus for efficient movements
        if (intent.type == IntentType::WALK_FORWARD ||
            intent.type == IntentType::SPRINT) {
            reward += 0.5 * intent.intensity;
        }

        // Bonus for maintaining balance
        if (intent.type == IntentType::BALANCE) {
            reward += 0.3;
        }
    } else {
        // Penalty for failure
        reward = -1.0;

        // Extra penalty for dangerous failures
        if (intent.type == IntentType::JUMP ||
            intent.type == IntentType::SPRINT) {
            reward -= 0.5;
        }
    }

    // Small penalty for emergency stops (indicates poor planning)
    if (intent.type == IntentType::EMERGENCY_STOP) {
        reward -= 0.2;
    }

    return reward;
}

} // namespace titanfall_ai
