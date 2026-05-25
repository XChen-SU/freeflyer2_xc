// tether_ctrl.hpp
#ifndef FF_CONTROL__TETHER_CTRL_HPP_
#define FF_CONTROL__TETHER_CTRL_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <ff_msgs/msg/free_flyer_state_stamped.hpp>
#include <ff_msgs/msg/wrench2_d.hpp>
#include <Eigen/Dense>

namespace ff
{

class TetherControlNode : public rclcpp::Node
{
public:
    using StateVec = Eigen::Matrix<double, 6, 1>;
    using ControlVec = Eigen::Matrix<double, 3, 1>;
    using FeedbackMat = Eigen::Matrix<double, 3, 6>;

    TetherControlNode();

private:
    void relativePositionCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg);
    void robotStateCallback(const ff_msgs::msg::FreeFlyerStateStamped::SharedPtr msg);
    void robot2PoseCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void controlTimerCallback();
    
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr relative_pos_sub_;
    rclcpp::Subscription<ff_msgs::msg::FreeFlyerStateStamped>::SharedPtr robot_state_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot2_pose_sub_;
    
    // 发布 wrench（世界坐标系）
    rclcpp::Publisher<ff_msgs::msg::Wrench2D>::SharedPtr wrench_pub_;
    
    rclcpp::TimerBase::SharedPtr control_timer_;
    
    double target_rel_x_ = 0.0;
    double target_rel_y_ = 0.0;
    bool target_set_ = false;
    
    geometry_msgs::msg::PointStamped current_relative_pos_;
    StateVec robot_state_;
    geometry_msgs::msg::PoseStamped robot2_pose_;
    
    bool relative_pos_received_ = false;
    bool robot_state_received_ = false;
    bool robot2_pose_received_ = false;
    
    FeedbackMat K_;
    double control_frequency_ = 10.0;
    
    void declareParameters();
    void initializeFeedbackGain();
};

}  // namespace ff

#endif