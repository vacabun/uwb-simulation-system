#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tinyxml2.hpp"
#include <unordered_map>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <random>
#include "uwb_interfaces/msg/uwb_data.hpp"
#include "uwb_interfaces/msg/uwb_distance.hpp"

class UWBSimulation : public rclcpp::Node
{
public:
    UWBSimulation();

private:
    void topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg);
    void uwb_simulate();
    void load_anchors_pos();
    
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
    geometry_msgs::msg::Point robotPose;

    std::string target_frame_;
    std::unordered_map<int, geometry_msgs::msg::Point> anthorPoseMap;
    std::default_random_engine tandomGenerator;

    rclcpp::Publisher<uwb_interfaces::msg::UWBData>::SharedPtr msgPublisher_;
    
};