#ifndef TITANFALL_AI_SENSOR_FUSION_HPP
#define TITANFALL_AI_SENSOR_FUSION_HPP

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <memory>
#include <vector>
#include <string>

namespace titanfall_ai {

class SensorFusion {
public:
    SensorFusion(rclcpp::Node::SharedPtr node);
    ~SensorFusion() = default;

    // Initialize sensor fusion system
    void initialize();
    
    // Main processing loop
    void process();
    
    // Sensor data callbacks
    void imuCallback(const sensor_msgs::msg::Imu::SharedPtr msg);
    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg);
    
    // Get fused state
    nav_msgs::msg::Odometry getFusedOdometry() const;
    geometry_msgs::msg::Pose getCurrentPose() const;
    geometry_msgs::msg::Twist getCurrentVelocity() const;
    
    // Configuration
    void setFusionParameters(double alpha, double beta, double gamma);
    void enableSensor(const std::string& sensor_name, bool enable);

private:
    rclcpp::Node::SharedPtr node_;
    
    // Publishers
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Pose>::SharedPtr pose_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_pub_;
    
    // Subscribers
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
    
    // Transform broadcaster
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    
    // State variables
    nav_msgs::msg::Odometry fused_odometry_;
    geometry_msgs::msg::Pose current_pose_;
    geometry_msgs::msg::Twist current_velocity_;
    
    // Sensor data
    sensor_msgs::msg::Imu latest_imu_;
    sensor_msgs::msg::JointState latest_joint_state_;
    
    // Fusion parameters
    double alpha_, beta_, gamma_;
    std::map<std::string, bool> sensor_enabled_;
    
    // Timing
    rclcpp::Time last_update_time_;
    double update_rate_;
    
    // Internal methods
    void updateFusedState();
    void publishTransforms();
    void kalmanFilterUpdate();
    void complementaryFilterUpdate();
};

} // namespace titanfall_ai

#endif // TITANFALL_AI_SENSOR_FUSION_HPP
