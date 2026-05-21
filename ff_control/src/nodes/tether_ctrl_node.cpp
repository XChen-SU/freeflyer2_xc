// MIT License
//
// Copyright (c) 2023 Stanford Autonomous Systems Lab
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.


#include <chrono>
#include <cmath>
#include <functional>
#include <memory>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>

#include "ff_msgs/msg/free_flyer_state.hpp"
#include "ff_control/linear_ctrl.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

class TetherControlNode : public ff::LinearController
{
public:
    TetherControlNode()
    : rclcpp::Node("tether_ctrl_node"),
      ff::LinearController()
    {
        rel_pos_sub_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            "/relative_position", 10, 
            std::bind(&TetherControlNode::RelativePosCallback, this, _1));
        
        timer_ = this->create_wall_timer(
            100ms, std::bind(&TetherControlNode::ControlLoop, this));

        // default params
        this->declare_parameter("fixed_speed", 0.2);
        this->declare_parameter("safe_distance", 0.1);
        this->declare_parameter("gain_df", 10.0);
        this->declare_parameter("gain_dt", 0.4);

        RCLCPP_INFO(this->get_logger(), "Tether Control Node Started");
    }

private:
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr rel_pos_sub_;
    rclcpp::TimerBase::SharedPtr timer_;

    geometry_msgs::msg::PointStamped rel_pos_msg_;
    bool rel_pos_received_ = false;

    void StateReadyCallback() override
    {
        RCLCPP_INFO(this->get_logger(), "Robot state ready, waiting for relative position...");
    }

    void ControlLoop()
    {
        // state not yet ready
        if (!StateIsReady()) {return;}

        // relative position not yet received
        if (!rel_pos_received_) {return;}

        // get parameters
        const double fixed_speed = this->get_parameter("fixed_speed").as_double();
        const double safe_distance = this->get_parameter("safe_distance").as_double();
        const double gain_df = this->get_parameter("gain_df").as_double();
        const double gain_dt = this->get_parameter("gain_dt").as_double();

        // build feedback control matrix (velocity control only)
        FeedbackMat K = FeedbackMat::Zero();
        K(0, 3) = gain_df;  // vx -> fx
        K(1, 4) = gain_df;  // vy -> fy
        K(2, 5) = gain_dt;  // wz -> tz

        // compute 2D distance
        const double dx = rel_pos_msg_.point.x;
        const double dy = rel_pos_msg_.point.y;
        const double distance = std::sqrt(dx * dx + dy * dy);

        // safety check
        if (distance < safe_distance) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 1000,
                "Too close (%.3f m), stopping", distance);
            
            ff_msgs::msg::FreeFlyerState state_des{};
            SendControl(state_des, K);
            return;
        }

        // compute desired velocity (away from other robot)
        ff_msgs::msg::FreeFlyerState state_des{};
        state_des.pose.x = 0.0;
        state_des.pose.y = 0.0;
        state_des.pose.theta = 0.0;
        state_des.twist.vx = -fixed_speed * dx / distance;
        state_des.twist.vy = -fixed_speed * dy / distance;
        state_des.twist.wz = 0.0;

        SendControl(state_des, K);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1000,
            "Distance: %.3f m | Velocity cmd: [%.3f, %.3f] m/s",
            distance, state_des.twist.vx, state_des.twist.vy);
    }

    void RelativePosCallback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        rel_pos_msg_ = *msg;
        rel_pos_received_ = true;
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TetherControlNode>());
    rclcpp::shutdown();
    return 0;
}