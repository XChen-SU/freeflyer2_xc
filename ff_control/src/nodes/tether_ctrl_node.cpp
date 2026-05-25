// tether_ctrl_node.cpp
#include "ff_control/tether_ctrl.hpp"
#include <chrono>
#include <cmath>

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace ff
{

TetherControlNode::TetherControlNode()
: Node("tether_ctrl_node")
{
    declareParameters();
    
    control_frequency_ = this->get_parameter("control_frequency").as_double();
    
    auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
    qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
    
    // 订阅相对位置
    relative_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/relative_position", 10,
        std::bind(&TetherControlNode::relativePositionCallback, this, _1));
    
    // 订阅 robot 状态（需要速度信息）
    robot_state_sub_ = this->create_subscription<ff_msgs::msg::FreeFlyerStateStamped>(
        "/robot/est/state", 10,
        std::bind(&TetherControlNode::robotStateCallback, this, _1));
    
    // 订阅 robot2 位姿
    robot2_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        "/robot2/pose", qos,
        std::bind(&TetherControlNode::robot2PoseCallback, this, _1));
    
    // 发布 wrench（世界坐标系）
    wrench_pub_ = this->create_publisher<ff_msgs::msg::Wrench2D>(
        "/robot/ctrl/wrench", 10);
    
    // 初始化反馈增益
    initializeFeedbackGain();
    
    // 创建控制定时器
    auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = this->create_wall_timer(
        std::chrono::duration_cast<std::chrono::milliseconds>(period),
        std::bind(&TetherControlNode::controlTimerCallback, this));
    
    RCLCPP_INFO(this->get_logger(), 
                "========================================");
    RCLCPP_INFO(this->get_logger(), 
                "Tether Control Node Started");
    RCLCPP_INFO(this->get_logger(), 
                "  Controlled Robot: /robot");
    RCLCPP_INFO(this->get_logger(), 
                "  Following: /robot2");
    RCLCPP_INFO(this->get_logger(), 
                "  Control Rate: %.1f Hz", control_frequency_);
    RCLCPP_INFO(this->get_logger(), 
                "========================================");
    RCLCPP_INFO(this->get_logger(), 
                "Waiting for data...");
}

void TetherControlNode::declareParameters()
{
    this->declare_parameter("control_frequency", 10.0);
    this->declare_parameter("kp_x", 5.0);
    this->declare_parameter("kp_y", 5.0);
    this->declare_parameter("kd_x", 2.0);
    this->declare_parameter("kd_y", 2.0);
}

void TetherControlNode::initializeFeedbackGain()
{
    K_.setZero();
    
    K_(0, 0) = this->get_parameter("kp_x").as_double();
    K_(1, 1) = this->get_parameter("kp_y").as_double();
    K_(2, 2) = 0.0;
    
    K_(0, 3) = this->get_parameter("kd_x").as_double();
    K_(1, 4) = this->get_parameter("kd_y").as_double();
    K_(2, 5) = 0.0;
    
    RCLCPP_INFO(this->get_logger(), 
                "Feedback gains - Kp: [%.2f, %.2f], Kd: [%.2f, %.2f]",
                K_(0, 0), K_(1, 1), K_(0, 3), K_(1, 4));
}

void TetherControlNode::relativePositionCallback(
    const geometry_msgs::msg::PointStamped::SharedPtr msg)
{
    current_relative_pos_ = *msg;
    
    if (!relative_pos_received_) {
        RCLCPP_INFO(this->get_logger(), "✓ Relative position received");
        relative_pos_received_ = true;
    }
    
    if (!target_set_ && robot2_pose_received_ && robot_state_received_) {
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

void TetherControlNode::robotStateCallback(
    const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg)
{
    robot_state_[0] = msg->state.pose.x;
    robot_state_[1] = msg->state.pose.y;
    robot_state_[2] = msg->state.pose.theta;
    robot_state_[3] = msg->state.twist.vx;
    robot_state_[4] = msg->state.twist.vy;
    robot_state_[5] = msg->state.twist.wz;
    
    if (!robot_state_received_) {
        RCLCPP_INFO(this->get_logger(), "✓ Robot state received");
        robot_state_received_ = true;
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
    if (!target_set_ || !relative_pos_received_ || !robot_state_received_ || !robot2_pose_received_) {
        return;
    }
    
    // 计算期望状态
    StateVec desired_state;
    desired_state.setZero();
    
    desired_state[0] = robot2_pose_.pose.position.x - target_rel_x_;
    desired_state[1] = robot2_pose_.pose.position.y - target_rel_y_;
    desired_state[2] = 0.0;
    desired_state[3] = 0.0;
    desired_state[4] = 0.0;
    desired_state[5] = 0.0;
    
    // 计算状态误差
    StateVec state_error = desired_state - robot_state_;
    state_error[2] = std::remainder(state_error[2], 2.0 * M_PI);
    
    // 计算控制力（世界坐标系）
    ControlVec control = K_ * state_error;
    
    // 发布 wrench
    ff_msgs::msg::Wrench2D wrench_msg;
    wrench_msg.fx = control[0];
    wrench_msg.fy = control[1];
    wrench_msg.tz = control[2];
    
    wrench_pub_->publish(wrench_msg);
    
    // 调试输出
    static int debug_count = 0;
    if (debug_count++ % (int)(2 * control_frequency_) == 0) {
        double rel_error_x = current_relative_pos_.point.x - target_rel_x_;
        double rel_error_y = current_relative_pos_.point.y - target_rel_y_;
        double rel_error = std::sqrt(rel_error_x * rel_error_x + rel_error_y * rel_error_y);
        
        RCLCPP_INFO(this->get_logger(),
                    "Error: %.3f m [%.3f, %.3f] | Wrench: [%.2f, %.2f] N",
                    rel_error, rel_error_x, rel_error_y,
                    wrench_msg.fx, wrench_msg.fy);
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