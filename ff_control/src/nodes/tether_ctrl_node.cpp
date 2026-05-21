// // MIT License
// //
// // Copyright (c) 2024 Stanford Autonomous Systems Lab

// #include "ff_control/tether_ctrl.hpp"
// #include <chrono>
// #include <cmath>

// using namespace std::chrono_literals;
// using namespace std::placeholders;

// namespace ff
// {

// TetherControlNode::TetherControlNode()
// : Node("tether_ctrl_node")
// {
//     // Declare and get parameters
//     declareParameters();
    
//     // Get control frequency
//     control_frequency_ = this->get_parameter("control_frequency").as_double();
    
//     // Create QoS profile with BEST_EFFORT
//     auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
//     qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
//     // Subscribe to relative position
//     relative_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
//         "/relative_position", 10,
//         std::bind(&TetherControlNode::relativePositionCallback, this, _1));
    
//     // Subscribe to robot1 pose
//     robot1_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
//         "/robot/pose", qos,
//         std::bind(&TetherControlNode::robot1PoseCallback, this, _1));
    
//     // Subscribe to robot2 state
//     std::string state_topic = this->declare_parameter("state_channel", "/robot2/est/state");
//     robot2_state_sub_ = this->create_subscription<ff_msgs::msg::FreeFlyerStateStamped>(
//         state_topic, 10,
//         std::bind(&TetherControlNode::robot2StateCallback, this, _1));
    
//     // Publisher for wrench commands
//     wrench_pub_ = this->create_publisher<ff_msgs::msg::Wrench2D>(
//         "/robot2/wrench_body", 10);
    
//     // Initialize feedback gain matrix
//     initializeFeedbackGain();
    
//     // Create control timer
//     auto period = std::chrono::duration<double>(1.0 / control_frequency_);
//     control_timer_ = this->create_wall_timer(
//         std::chrono::duration_cast<std::chrono::milliseconds>(period),
//         std::bind(&TetherControlNode::controlTimerCallback, this));
    
//     RCLCPP_INFO(this->get_logger(), 
//                 "Tether Control Node initialized at %.1f Hz", 
//                 control_frequency_);
//     RCLCPP_INFO(this->get_logger(), "Subscribing to state: %s", state_topic.c_str());
// }

// void TetherControlNode::declareParameters()
// {
//     // Control parameters
//     this->declare_parameter("control_frequency", 10.0);
    
//     // Feedback gains for position
//     this->declare_parameter("kp_x", 2.0);
//     this->declare_parameter("kp_y", 2.0);
//     this->declare_parameter("kp_theta", 0.0);
    
//     // Feedback gains for velocity (damping)
//     this->declare_parameter("kd_x", 1.0);
//     this->declare_parameter("kd_y", 1.0);
//     this->declare_parameter("kd_theta", 0.0);
// }

// void TetherControlNode::initializeFeedbackGain()
// {
//     K_.setZero();
    
//     // Position feedback gains (columns 0, 1, 2)
//     K_(0, 0) = this->get_parameter("kp_x").as_double();
//     K_(1, 1) = this->get_parameter("kp_y").as_double();
//     K_(2, 2) = this->get_parameter("kp_theta").as_double();
    
//     // Velocity feedback gains (columns 3, 4, 5)
//     K_(0, 3) = this->get_parameter("kd_x").as_double();
//     K_(1, 4) = this->get_parameter("kd_y").as_double();
//     K_(2, 5) = this->get_parameter("kd_theta").as_double();
    
//     RCLCPP_INFO(this->get_logger(), 
//                 "Feedback gains - Kp: [%.2f, %.2f, %.2f], Kd: [%.2f, %.2f, %.2f]",
//                 K_(0, 0), K_(1, 1), K_(2, 2),
//                 K_(0, 3), K_(1, 4), K_(2, 5));
// }

// void TetherControlNode::relativePositionCallback(
//     const geometry_msgs::msg::PointStamped::SharedPtr msg)
// {
//     current_relative_pos_ = *msg;
//     relative_pos_received_ = true;
    
//     // Set target relative position on first reception
//     if (!target_set_ && robot2_state_received_) {
//         target_rel_x_ = msg->point.x;
//         target_rel_y_ = msg->point.y;
//         target_set_ = true;
        
//         RCLCPP_INFO(this->get_logger(),
//                     "Target relative position locked: [%.3f, %.3f] m",
//                     target_rel_x_, target_rel_y_);
//     }
// }

// void TetherControlNode::robot1PoseCallback(
//     const geometry_msgs::msg::PoseStamped::SharedPtr msg)
// {
//     robot1_pose_ = *msg;
//     robot1_pose_received_ = true;
// }

// void TetherControlNode::robot2StateCallback(
//     const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg)
// {
//     // Convert FreeFlyerState to StateVec
//     robot2_state_[0] = msg->state.pose.x;
//     robot2_state_[1] = msg->state.pose.y;
//     robot2_state_[2] = msg->state.pose.theta;
//     robot2_state_[3] = msg->state.twist.vx;
//     robot2_state_[4] = msg->state.twist.vy;
//     robot2_state_[5] = msg->state.twist.wz;
    
//     robot2_state_received_ = true;
    
//     if (!target_set_) {
//         RCLCPP_INFO_ONCE(this->get_logger(), "Robot2 state received, waiting for relative position");
//     }
// }

// void TetherControlNode::controlTimerCallback()
// {
//     // Check if all required data is available
//     if (!target_set_) {
//         RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
//                               "Waiting for target position to be set");
//         return;
//     }
    
//     if (!relative_pos_received_) {
//         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
//                              "No relative position data received");
//         return;
//     }
    
//     if (!robot1_pose_received_) {
//         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
//                              "No robot1 pose data received");
//         return;
//     }
    
//     if (!robot2_state_received_) {
//         RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
//                              "No robot2 state received");
//         return;
//     }
    
//     // Calculate desired state for robot2
//     StateVec desired_state;
//     desired_state.setZero();
    
//     // Desired position = robot1 position + target relative position
//     desired_state[0] = robot1_pose_.pose.position.x + target_rel_x_;  // x
//     desired_state[1] = robot1_pose_.pose.position.y + target_rel_y_;  // y
//     desired_state[2] = 0.0;  // theta (don't care)
    
//     // Desired velocity = 0 (maintain position)
//     desired_state[3] = 0.0;  // vx
//     desired_state[4] = 0.0;  // vy
//     desired_state[5] = 0.0;  // wz
    
//     // Calculate state error
//     StateVec state_error = desired_state - robot2_state_;
    
//     // Wrap angle error to [-pi, pi]
//     state_error[2] = std::remainder(state_error[2], 2.0 * M_PI);
    
//     // Compute control using feedback gain
//     ControlVec control = K_ * state_error;
    
//     // Create and publish wrench command (body frame)
//     ff_msgs::msg::Wrench2D wrench_msg;
//     wrench_msg.fx = control[0];
//     wrench_msg.fy = control[1];
//     wrench_msg.tz = control[2];
    
//     wrench_pub_->publish(wrench_msg);
    
//     // Debug output
//     double error_x = current_relative_pos_.point.x - target_rel_x_;
//     double error_y = current_relative_pos_.point.y - target_rel_y_;
//     double error_distance = std::sqrt(error_x * error_x + error_y * error_y);
    
//     RCLCPP_DEBUG(this->get_logger(),
//                  "Rel pos error: [%.3f, %.3f] (%.3f m), Wrench: [%.2f, %.2f, %.2f]",
//                  error_x, error_y, error_distance,
//                  wrench_msg.fx, wrench_msg.fy, wrench_msg.tz);
// }

// }  // namespace ff

// int main(int argc, char ** argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<ff::TetherControlNode>();
//     rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }
// MIT License
//
// Copyright (c) 2024 Stanford Autonomous Systems Lab

#include "ff_control/tether_ctrl.hpp"
#include <chrono>
#include <cmath>
#include <algorithm>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace ff
{

TetherControlNode::TetherControlNode()
: Node("tether_ctrl_node")
{
    // Declare and get parameters
    declareParameters();
    
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    kp_x_ = this->get_parameter("kp_x").as_double();
    kp_y_ = this->get_parameter("kp_y").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();
    
    // Create QoS profile with BEST_EFFORT (match mocap)
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    // Subscribe to relative position
    relative_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/relative_position", 10,
        std::bind(&TetherControlNode::relativePositionCallback, this, _1));
    
    // Subscribe to robot poses (both from mocap)
    robot1_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot/pose", qos,
        std::bind(&TetherControlNode::robot1PoseCallback, this, _1));
    
    robot2_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot2/pose", qos,
        std::bind(&TetherControlNode::robot2PoseCallback, this, _1));
    
    // Publish velocity commands
    cmd_vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/robot2/cmd_vel", 10);
    
    // Create control timer
    auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&TetherControlNode::controlTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), 
                "========================================");
    RCLCPP_INFO(this->get_logger(), 
                "Tether Control Node Started");
    RCLCPP_INFO(this->get_logger(), 
                "  Control Rate: %.1f Hz", control_frequency_);
    RCLCPP_INFO(this->get_logger(), 
                "  Gains: Kp_x=%.2f, Kp_y=%.2f", kp_x_, kp_y_);
    RCLCPP_INFO(this->get_logger(), 
                "  Max Velocity: %.2f m/s", max_velocity_);
    RCLCPP_INFO(this->get_logger(), 
                "========================================");
    RCLCPP_INFO(this->get_logger(), 
                "Waiting for data...");
}

void TetherControlNode::declareParameters()
{
    this->declare_parameter("control_frequency", 10.0);
    this->declare_parameter("kp_x", 2.0);
    this->declare_parameter("kp_y", 2.0);
    this->declare_parameter("max_velocity", 1.0);
}

void TetherControlNode::relativePositionCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    current_relative_pos_ = *msg;
    
    if (!relative_pos_received_) {
        RCLCPP_INFO(this->get_logger(), "✓ Relative position data received");
        relative_pos_received_ = true;
    }
    
    // Lock target on first reception when robot2 data is also available
    if (!target_set_ && robot2_pose_received_) {
        target_rel_x_ = msg->point.x;
        target_rel_y_ = msg->point.y;
        target_set_ = true;
        
        double distance = std::sqrt(target_rel_x_ * target_rel_x_ + 
                                   target_rel_y_ * target_rel_y_);
        
        RCLCPP_INFO(this->get_logger(), 
                    "========================================");
        RCLCPP_INFO(this->get_logger(), 
                    "🎯 TARGET LOCKED");
        RCLCPP_INFO(this->get_logger(), 
                    "  Relative Position: [%.3f, %.3f] m", 
                    target_rel_x_, target_rel_y_);
        RCLCPP_INFO(this->get_logger(), 
                    "  Distance: %.3f m", distance);
        RCLCPP_INFO(this->get_logger(), 
                    "========================================");
        RCLCPP_INFO(this->get_logger(), 
                    "🚀 Control active!");
    }
}

void TetherControlNode::robot1PoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    robot1_pose_ = *msg;
    
    if (!robot1_pose_received_) {
        RCLCPP_INFO(this->get_logger(), 
                    "✓ Robot1 pose received at [%.2f, %.2f]",
                    msg->pose.position.x, msg->pose.position.y);
        robot1_pose_received_ = true;
    }
}

void TetherControlNode::robot2PoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg)
{
    robot2_pose_ = *msg;
    
    if (!robot2_pose_received_) {
        RCLCPP_INFO(this->get_logger(), 
                    "✓ Robot2 pose received at [%.2f, %.2f]",
                    msg->pose.position.x, msg->pose.position.y);
        robot2_pose_received_ = true;
    }
}

void TetherControlNode::controlTimerCallback()
{
    // Status logging (every 5 seconds)
    static int status_count = 0;
    if (status_count++ % (int)(5 * control_frequency_) == 0) {
        if (!target_set_) {
            RCLCPP_INFO(this->get_logger(),
                        "Waiting... (rel_pos=%d, robot1=%d, robot2=%d)",
                        relative_pos_received_, 
                        robot1_pose_received_, 
                        robot2_pose_received_);
        }
    }
    
    // Check if target is set
    if (!target_set_) {
        return;
    }
    
    // Check if all data is available
    if (!relative_pos_received_ || !robot1_pose_received_ || !robot2_pose_received_) {
        RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 5000,
                             "Missing data! Cannot control.");
        return;
    }
    
    // Calculate desired position for robot2
    // desired_pos = robot1_pos + target_relative_pos
    double desired_x = robot1_pose_.pose.position.x + target_rel_x_;
    double desired_y = robot1_pose_.pose.position.y + target_rel_y_;
    
    // Calculate position error
    double error_x = desired_x - robot2_pose_.pose.position.x;
    double error_y = desired_y - robot2_pose_.pose.position.y;
    double error_magnitude = std::sqrt(error_x * error_x + error_y * error_y);
    
    // Proportional control
    double vel_x = kp_x_ * error_x;
    double vel_y = kp_y_ * error_y;
    
    // Limit velocity
    double vel_magnitude = std::sqrt(vel_x * vel_x + vel_y * vel_y);
    if (vel_magnitude > max_velocity_) {
        double scale = max_velocity_ / vel_magnitude;
        vel_x *= scale;
        vel_y *= scale;
    }
    
    // Create and publish command
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = vel_x;
    cmd_vel.linear.y = vel_y;
    cmd_vel.linear.z = 0.0;
    cmd_vel.angular.x = 0.0;
    cmd_vel.angular.y = 0.0;
    cmd_vel.angular.z = 0.0;
    
    cmd_vel_pub_->publish(cmd_vel);
    
    // Debug output (every 2 seconds)
    static int debug_count = 0;
    if (debug_count++ % (int)(2 * control_frequency_) == 0) {
        double current_rel_x = current_relative_pos_.point.x;
        double current_rel_y = current_relative_pos_.point.y;
        double rel_error_x = current_rel_x - target_rel_x_;
        double rel_error_y = current_rel_y - target_rel_y_;
        double rel_error = std::sqrt(rel_error_x * rel_error_x + rel_error_y * rel_error_y);
        
        RCLCPP_INFO(this->get_logger(),
                    "Rel Error: %.3f m [%.3f, %.3f] | Cmd: [%.2f, %.2f] m/s",
                    rel_error, rel_error_x, rel_error_y,
                    cmd_vel.linear.x, cmd_vel.linear.y);
    }
}

}  // namespace ff

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ff::TetherControlNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}