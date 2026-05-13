#include <chrono>
#include <cmath>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>

#include "ff_msgs/msg/free_flyer_state.hpp"
#include "ff_control/linear_ctrl.hpp"

using namespace std::chrono_literals;

class TetherControlNode : public ff::LinearController
{
public:
  TetherControlNode()
  : rclcpp::Node("tether_ctrl_node"),
    ff::LinearController()
  {
    // 订阅相对位置
    rel_pos_sub_ = this->create_subscription<geometry_msgs::msg::Vector3Stamped>(
        "/relative_position", 10,
        std::bind(&TetherControlNode::relative_pos_callback, this, std::placeholders::_1));
    
    // 控制回路
    control_timer_ = this->create_wall_timer(
        100ms, std::bind(&TetherControlNode::ControlLoop, this));
    
    // 参数
    fixed_speed_ = declare_parameter("fixed_speed", 0.2);
    
    // 反馈增益矩阵
    K_.fill(0);
    const double gain_df = declare_parameter("gain_df", 10.0);
    const double gain_dt = declare_parameter("gain_dt", 0.4);
    K_(0, 3) = gain_df;
    K_(1, 4) = gain_df;
    K_(2, 5) = gain_dt;
    
    RCLCPP_INFO(this->get_logger(), 
        "Tether Control Node Started - Speed: %.2f m/s", fixed_speed_);
  }

private:
  rclcpp::Subscription<geometry_msgs::msg::Vector3Stamped>::SharedPtr rel_pos_sub_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  
  geometry_msgs::msg::Vector3 relative_pos_;
  bool rel_pos_received_ = false;
  
  FeedbackMat K_;
  double fixed_speed_;
  
  void relative_pos_callback(const geometry_msgs::msg::Vector3Stamped::SharedPtr msg)
  {
    relative_pos_ = msg->vector;
    rel_pos_received_ = true;
  }
  
  void ControlLoop()
  {
    if (!StateIsReady() || !rel_pos_received_) {
      return;
    }
    
    // 计算距离（用于归一化）
    double dx = relative_pos_.x;
    double dy = relative_pos_.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    if (distance < 0.01) {  // 避免除以0
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
          "Robots too close, stopping control");
      return;
    }
    
    // 归一化 + 反方向 + 固定速度
    ff_msgs::msg::FreeFlyerState state_des{};
    state_des.twist.vx = -fixed_speed_ * dx / distance;
    state_des.twist.vy = -fixed_speed_ * dy / distance;
    state_des.twist.wz = 0.0;
    
    SendControl(state_des, K_);
    
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
        "Distance: %.3f m | Cmd velocity: [%.3f, %.3f] m/s",
        distance, state_des.twist.vx, state_des.twist.vy);
  }
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TetherControlNode>());
  rclcpp::shutdown();
  return 0;
}