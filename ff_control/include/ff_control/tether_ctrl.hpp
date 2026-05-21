// // MIT License
// //
// // Copyright (c) 2024 Stanford Autonomous Systems Lab

// #ifndef FF_CONTROL__TETHER_CTRL_HPP_
// #define FF_CONTROL__TETHER_CTRL_HPP_

// #include <rclcpp/rclcpp.hpp>
// #include <geometry_msgs/msg/point_stamped.hpp>
// #include <geometry_msgs/msg/pose_stamped.hpp>
// #include <ff_msgs/msg/free_flyer_state_stamped.hpp>
// #include <ff_msgs/msg/wrench2_d.hpp>
// #include <Eigen/Dense>

// namespace ff
// {

// class TetherControlNode : public rclcpp::Node
// {
// public:
//     using StateVec = Eigen::Matrix<double, 6, 1>;
//     using ControlVec = Eigen::Matrix<double, 3, 1>;
//     using FeedbackMat = Eigen::Matrix<double, 3, 6>;

//     TetherControlNode();

// private:
//     void relativePositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
//     void robot1PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
//     void robot2StateCallback(const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg);
//     void controlTimerCallback();
    
//     // Subscriptions
//     rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr relative_pos_sub_;
//     rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot1_pose_sub_;
//     rclcpp::Subscription<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr robot2_state_sub_;
    
//     // Publishers
//     rclcpp::Publisher<ff_msgs::msg::Wrench2D>::SharedPtr wrench_pub_;
    
//     // Timer
//     rclcpp::TimerBase::SharedPtr control_timer_;
    
//     // Target relative position
//     double target_rel_x_ = 0.0;
//     double target_rel_y_ = 0.0;
//     bool target_set_ = false;
    
//     // Current data
//     geometry_msgs::msg::PointStamped current_relative_pos_;
//     geometry_msgs::msg::PoseStamped robot1_pose_;
//     StateVec robot2_state_;
    
//     bool relative_pos_received_ = false;
//     bool robot1_pose_received_ = false;
//     bool robot2_state_received_ = false;
    
//     // Control parameters
//     FeedbackMat K_;
//     double control_frequency_ = 10.0;
    
//     void declareParameters();
//     void initializeFeedbackGain();
// };

// }  // namespace ff

// #endif  // FF_CONTROL__TETHER_CTRL_HPP_
// MIT License
//
// Copyright (c) 2024 Stanford Autonomous Systems Lab

#ifndef FF_CONTROL__TETHER_CTRL_HPP_
#define FF_CONTROL__TETHER_CTRL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>

namespace ff
{

class TetherControlNode : public rclcpp::Node
{
public:
    TetherControlNode();

private:
    void relativePositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void robot1PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void robot2PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlTimerCallback();
    
    // Subscriptions
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr relative_pos_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot1_pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot2_pose_sub_;
    
    // Publishers
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    
    // Timer
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    // Target relative position (locked at startup)
    double target_rel_x_ = 0.0;
    double target_rel_y_ = 0.0;
    bool target_set_ = false;
    
    // Current data
    geometry_msgs::msg::PointStamped current_relative_pos_;
    geometry_msgs::msg::PoseStamped robot1_pose_;
    geometry_msgs::msg::PoseStamped robot2_pose_;
    
    // Data received flags
    bool relative_pos_received_ = false;
    bool robot1_pose_received_ = false;
    bool robot2_pose_received_ = false;
    
    // Control parameters
    double kp_x_ = 2.0;
    double kp_y_ = 2.0;
    double control_frequency_ = 10.0;
    double max_velocity_ = 1.0;  // m/s
    
    void declareParameters();
};

}  // namespace ff

#endif  // FF_CONTROL__TETHER_CTRL_HPP_