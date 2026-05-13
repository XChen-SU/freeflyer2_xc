#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <cmath>

class RelativePositionPrinter : public rclcpp::Node
{
public:
    RelativePositionPrinter() : Node("relative_pos_node")
    {
        // robot1_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "/robot/pose", 10,
        //     std::bind(&RelativePositionPrinter::robot1_callback, this, std::placeholders::_1));

        // robot2_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
        //     "/robot2/pose", 10,
        //     std::bind(&RelativePositionPrinter::robot2_callback, this, std::placeholders::_1));
        
        // RCLCPP_INFO(this->get_logger(), "Relative Position Printer Node Started");



        // Create QoS profile with BEST_EFFORT
        auto qos = rclcpp::QoS(rclcpp::KeepLast(10));
        qos.reliability(rclcpp::ReliabilityPolicy::BestEffort);
        
        robot1_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot/pose", qos,  // <-- Use qos here
            std::bind(&RelativePositionPrinter::robot1_callback, this, std::placeholders::_1));

        robot2_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot2/pose", qos,  // <-- Use qos here
            std::bind(&RelativePositionPrinter::robot2_callback, this, std::placeholders::_1));
        
        RCLCPP_INFO(this->get_logger(), "Relative Position Printer Node Started");
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
            double dx = robot2_pos_.x - robot1_pos_.x;
            double dy = robot2_pos_.y - robot1_pos_.y;
            double dz = robot2_pos_.z - robot1_pos_.z;

            double distance = std::sqrt(dx*dx + dy*dy + dz*dz);

            RCLCPP_INFO(this->get_logger(), 
                "Robot1 position: [%.3f, %.3f, %.3f]",
                robot1_pos_.x, robot1_pos_.y, robot1_pos_.z);
            
            RCLCPP_INFO(this->get_logger(), 
                "Robot2 position: [%.3f, %.3f, %.3f]",
                robot2_pos_.x, robot2_pos_.y, robot2_pos_.z);
            
            RCLCPP_INFO(this->get_logger(), 
                "Relative position (robot2 - robot1): [%.3f, %.3f, %.3f]",
                dx, dy, dz);
            
            RCLCPP_INFO(this->get_logger(), 
                "Distance between robots: %.3f meters\n",
                distance);
        }
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot1_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot2_sub_;
    
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