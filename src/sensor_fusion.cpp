#include "titanfall_ai/sensor_fusion.hpp"
#include <tf2/LinearMath/Transform.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace titanfall_ai {

SensorFusion::SensorFusion(rclcpp::Node::SharedPtr node)
    : node_(node), alpha_(0.98), beta_(0.02), gamma_(0.01), update_rate_(100.0) {
    initialize();
}

void SensorFusion::initialize() {
    // Initialize publishers
    odom_pub_ = node_->create_publisher<nav_msgs::msg::Odometry>("fused_odometry", 10);
    pose_pub_ = node_->create_publisher<geometry_msgs::msg::Pose>("current_pose", 10);
    velocity_pub_ = node_->create_publisher<geometry_msgs::msg::Twist>("current_velocity", 10);
    
    // Initialize subscribers
    imu_sub_ = node_->create_subscription<sensor_msgs::msg::Imu>(
        "imu_data", 10,
        std::bind(&SensorFusion::imuCallback, this, std::placeholders::_1));
    
    joint_sub_ = node_->create_subscription<sensor_msgs::msg::JointState>(
        "joint_states", 10,
        std::bind(&SensorFusion::jointStateCallback, this, std::placeholders::_1));
    
    // Initialize transform broadcaster
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(node_);
    
    // Initialize sensor enabled map
    sensor_enabled_["imu"] = true;
    sensor_enabled_["joint_states"] = true;
    
    // Initialize timing
    last_update_time_ = node_->now();
    
    RCLCPP_INFO(node_->get_logger(), "Sensor Fusion initialized");
}

void SensorFusion::process() {
    auto current_time = node_->now();
    auto dt = (current_time - last_update_time_).seconds();
    
    if (dt >= 1.0 / update_rate_) {
        updateFusedState();
        publishTransforms();
        last_update_time_ = current_time;
    }
}

void SensorFusion::imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg) {
    latest_imu_ = *msg;
}

void SensorFusion::jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg) {
    latest_joint_state_ = *msg;
}

nav_msgs::msg::Odometry SensorFusion::getFusedOdometry() const {
    return fused_odometry_;
}

geometry_msgs::msg::Pose SensorFusion::getCurrentPose() const {
    return current_pose_;
}

geometry_msgs::msg::Twist SensorFusion::getCurrentVelocity() const {
    return current_velocity_;
}

void SensorFusion::setFusionParameters(double alpha, double beta, double gamma) {
    alpha_ = alpha;
    beta_ = beta;
    gamma_ = gamma;
}

void SensorFusion::enableSensor(const std::string& sensor_name, bool enable) {
    sensor_enabled_[sensor_name] = enable;
}

void SensorFusion::updateFusedState() {
    // Implement sensor fusion algorithm (simplified version)
    if (sensor_enabled_["imu"] && sensor_enabled_["joint_states"]) {
        // Use complementary filter for orientation
        complementaryFilterUpdate();
        
        // Use Kalman filter for position
        kalmanFilterUpdate();
    }
    
    // Publish fused state
    fused_odometry_.header.stamp = node_->now();
    fused_odometry_.header.frame_id = "odom";
    fused_odometry_.child_frame_id = "base_link";
    
    fused_odometry_.pose.pose = current_pose_;
    fused_odometry_.twist.twist = current_velocity_;
    
    odom_pub_->publish(fused_odometry_);
    pose_pub_->publish(current_pose_);
    velocity_pub_->publish(current_velocity_);
}

void SensorFusion::publishTransforms() {
    geometry_msgs::msg::TransformStamped transform;
    transform.header.stamp = node_->now();
    transform.header.frame_id = "odom";
    transform.child_frame_id = "base_link";
    
    transform.transform.translation.x = current_pose_.position.x;
    transform.transform.translation.y = current_pose_.position.y;
    transform.transform.translation.z = current_pose_.position.z;
    
    transform.transform.rotation = current_pose_.orientation;
    
    tf_broadcaster_->sendTransform(transform);
}

void SensorFusion::kalmanFilterUpdate() {
    // Simplified Kalman filter implementation
    // In a real implementation, this would include proper state estimation
    // For now, just use IMU data directly
    if (sensor_enabled_["imu"]) {
        // Update position based on velocity integration
        auto dt = (node_->now() - last_update_time_).seconds();
        current_pose_.position.x += current_velocity_.linear.x * dt;
        current_pose_.position.y += current_velocity_.linear.y * dt;
        current_pose_.position.z += current_velocity_.linear.z * dt;
    }
}

void SensorFusion::complementaryFilterUpdate() {
    // Simplified complementary filter for orientation
    if (sensor_enabled_["imu"]) {
        // Use IMU orientation directly (in real implementation, would fuse with other sensors)
        current_pose_.orientation = latest_imu_.orientation;
        
        // Update angular velocity
        current_velocity_.angular.x = latest_imu_.angular_velocity.x;
        current_velocity_.angular.y = latest_imu_.angular_velocity.y;
        current_velocity_.angular.z = latest_imu_.angular_velocity.z;
    }
}

} // namespace titanfall_ai
