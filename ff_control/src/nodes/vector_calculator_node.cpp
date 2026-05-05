#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include <cmath>

class VectorCalculator : public rclcpp::Node
{
public:
    VectorCalculator() : Node("vector_calculator")
    {
        robot1_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot1/pose", 10,
            std::bind(&VectorCalculator::robot1_callback, this, std::placeholders::_1));
        
        robot2_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "/robot2/pose", 10,
            std::bind(&VectorCalculator::robot2_callback, this, std::placeholders::_1));
        
        vector_pub_ = this->create_publisher<geometry_msgs::msg::Vector3Stamped>(
            "/robots_position_vector", 10);
        
        RCLCPP_INFO(this->get_logger(), "Vector Calculator Node Started");
    }

private:
    void robot1_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        robot1_pos_ = msg->pose.position;
        robot1_received_ = true;
        calculate_vector();
    }
    
    void robot2_callback(const geometry_msgs::msg::PoseStamped::SharedPtr msg)
    {
        robot2_pos_ = msg->pose.position;
        robot2_received_ = true;
        calculate_vector();
    }
    
    void calculate_vector()
    {
        if (robot1_received_ && robot2_received_)
        {
            auto vector_msg = geometry_msgs::msg::Vector3Stamped();
            vector_msg.header.stamp = this->now();
            vector_msg.header.frame_id = "world";  // 根据实际修改
            
            vector_msg.vector.x = robot2_pos_.x - robot1_pos_.x;
            vector_msg.vector.y = robot2_pos_.y - robot1_pos_.y;
            vector_msg.vector.z = robot2_pos_.z - robot1_pos_.z;
            
            double distance = std::sqrt(
                vector_msg.vector.x * vector_msg.vector.x +
                vector_msg.vector.y * vector_msg.vector.y +
                vector_msg.vector.z * vector_msg.vector.z);
            
            vector_pub_->publish(vector_msg);
            
            RCLCPP_INFO(this->get_logger(), 
                "Vector (robot1->robot2): [%.3f, %.3f, %.3f], Distance: %.3f meters",
                vector_msg.vector.x, vector_msg.vector.y, vector_msg.vector.z, distance);
        }
    }
    
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot1_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr robot2_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Vector3Stamped>::SharedPtr vector_pub_;
    
    geometry_msgs::msg::Point robot1_pos_;
    geometry_msgs::msg::Point robot2_pos_;
    bool robot1_received_ = false;
    bool robot2_received_ = false;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<VectorCalculator>());
    rclcpp::shutdown();
    return 0;
}