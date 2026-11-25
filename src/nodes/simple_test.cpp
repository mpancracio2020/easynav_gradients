#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <nav_msgs/msg/path.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

class SimpleTest : public rclcpp::Node
{
public:
    SimpleTest() : Node("simple_test")
    {
        // Publicadores para datos de prueba
        odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/robot_pose", 10);
        goal_pub_ = this->create_publisher<geometry_msgs::msg::PoseStamped>("/goal_pose", 10);
        
        // Suscriptor para el path generado
        path_sub_ = this->create_subscription<nav_msgs::msg::Path>(
            "/planner/path", 10,
            [this](const nav_msgs::msg::Path::SharedPtr msg) {
                RCLCPP_INFO(this->get_logger(), 
                           "PATH RECEIVED with %zu poses", msg->poses.size());
                
                // Mostrar detalles del path
                if (!msg->poses.empty()) {
                    const auto& first = msg->poses.front().pose.position;
                    const auto& last = msg->poses.back().pose.position;
                    RCLCPP_INFO(this->get_logger(),
                               "Path from (%.2f, %.2f) to (%.2f, %.2f)",
                               first.x, first.y, last.x, last.y);
                }
            });
        
        // Temporizador para publicar datos de prueba
        timer_ = this->create_wall_timer(
            std::chrono::seconds(3),
            [this]() {
                publish_test_data();
            });
        
        RCLCPP_INFO(this->get_logger(), "SimpleTest started - Testing GradientPlanner");
        RCLCPP_INFO(this->get_logger(), "Publishing to /robot_pose and /goal_pose");
        RCLCPP_INFO(this->get_logger(), "Listening to /planner/path");
    }

private:
    void publish_test_data()
    {
        static int counter = 0;
        
        // Publicar odometría fake
        auto odom_msg = nav_msgs::msg::Odometry();
        odom_msg.header.stamp = this->now();
        odom_msg.header.frame_id = "odom";
        odom_msg.pose.pose.position.x = 1.0 + (counter % 10) * 0.5;
        odom_msg.pose.pose.position.y = 2.0 + (counter % 10) * 0.3;
        odom_msg.pose.pose.position.z = 0.0;
        odom_msg.pose.pose.orientation.w = 1.0;
        
        odom_pub_->publish(odom_msg);
        RCLCPP_INFO(this->get_logger(), "Published odometry: (%.2f, %.2f)", 
                   odom_msg.pose.pose.position.x,
                   odom_msg.pose.pose.position.y);
        
        // Publicar goal fake cada 2 iteraciones
        if (counter % 2 == 0) {
            auto goal_msg = geometry_msgs::msg::PoseStamped();
            goal_msg.header.stamp = this->now();
            goal_msg.header.frame_id = "map";
            goal_msg.pose.position.x = 5.0 + (counter % 5);
            goal_msg.pose.position.y = 5.0 + (counter % 3);
            goal_msg.pose.position.z = 0.0;
            goal_msg.pose.orientation.w = 1.0;
            
            goal_pub_->publish(goal_msg);
            RCLCPP_INFO(this->get_logger(), "Published goal: (%.2f, %.2f)", 
                       goal_msg.pose.position.x,
                       goal_msg.pose.position.y);
        }
        
        counter++;
    }

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pub_;
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_sub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SimpleTest>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}