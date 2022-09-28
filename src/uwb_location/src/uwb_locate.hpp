#ifndef _UWB_LOCATE_ 
#define _UWB_LOCATE_

#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "uwb_interfaces/msg/uwb_data.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tinyxml2/tinyxml2.hpp"
#include "ransac_locator/ransac_locator.hpp"

class UWBLocate : public rclcpp::Node
{
public:
    UWBLocate();

private:
    void topic_callback(const uwb_interfaces::msg::UWBData::SharedPtr msg);

    void load_anchors_pos();
    
    rclcpp::Subscription<uwb_interfaces::msg::UWBData>::SharedPtr subscription_;

    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr msgPublisher_;
    
    std::unordered_map<int, geometry_msgs::msg::Point> anthorPoseMap;
};
#endif