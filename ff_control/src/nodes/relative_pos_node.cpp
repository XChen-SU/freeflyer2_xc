#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point_stamped.hpp>
#include <cmath>

class RelativePositionPrinter : public rclcpp::Node
{
public:
    RelativePositionPrinter() : Node("relative_pos_node")
    {
        // Create QoS profile with BEST_EFFORT
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        // Subscribe robot poses
        robot1_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot/pose", qos,
            std::bind(&RelativePositionPrinter::robot1_callback, this, std::placeholders::_1));

        robot2_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot2/pose", qos,
            std::bind(&RelativePositionPrinter::robot2_callback, this, std::placeholders::_1));
        
        // Publish relative position vector (2D only)
        relative_pos_pub_ = this->create_publisher<geometry_msgs::msg::PointStamped>(
            "/relative_position", 10);

        RCLCPP_INFO(this->get_logger(), "Relative Position Printer Node Started (2D mode)");
    }

private:
    void robot1_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        robot1_pos_ = msg->pose.position;
        robot1_received_ = true;
        print_relative_position();
    }
    
    void robot2_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        robot2_pos_ = msg->pose.position;
        robot2_received_ = true;
        print_relative_position();
    }
    
    void print_relative_position()
    {
        if (robot1_received_ && robot2_received_)
        {
            // Only calculate 2D relative position (ignore z)
            double dx = robot2_pos_.x - robot1_pos_.x;
            double dy = robot2_pos_.y - robot1_pos_.y;
            double distance_2d = std::sqrt(dx*dx + dy*dy);

            // Publish relative pos msg (z is set to 0)
            geometry_msgs::msg::PointStamped rel_pos_msg;
            rel_pos_msg.header.stamp = this->now();
            rel_pos_msg.header.frame_id = "world";
            rel_pos_msg.point.x = dx;
            rel_pos_msg.point.y = dy;
            rel_pos_msg.point.z = 0.0;  // Set z to 0 for 2D mode
            
            relative_pos_pub_->publish(rel_pos_msg);

            // Print to terminal (2D only)
            RCLCPP_INFO(this->get_logger(), 
                "Relative pos [2D]: [%.3f, %.3f] | Distance: %.3f m",
                dx, dy, distance_2d);
        }
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot1_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot2_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr relative_pos_pub_;
    
    geometry_msgs::msg::Point robot1_pos_;
    geometry_msgs::msg::Point robot2_pos_;
    bool robot1_received_ = false;
    bool robot2_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<RelativePositionPrinter>());
    rclcpp::shutdown();
    return 0;
}