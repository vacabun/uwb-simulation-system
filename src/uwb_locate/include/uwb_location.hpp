#ifndef _UWB_LOCATE_ 
#define _UWB_LOCATE_

#include <vector>
#include <unordered_map>

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "ament_index_cpp/get_package_share_directory.hpp"

#include "tinyxml2/tinyxml2.hpp"
#include "ransac_locator/ransac_locator.hpp"

#include "uwb_interfaces/msg/uwb_location_data.hpp"
#include "uwb_interfaces/msg/uwb_data.hpp"

class UWBLocation : public rclcpp::Node
{
public:
    UWBLocation();

private:
    void topic_callback(const uwb_interfaces::msg::UWBData::SharedPtr msg);

    void load_anchors_pos();
    
    rclcpp::Subscription<uwb_interfaces::msg::UWBData>::SharedPtr subscription_;

    rclcpp::Publisher<uwb_interfaces::msg::UWBLocationData>::SharedPtr msgPublisher_;
    
    // std::unordered_map<int, geometry_msgs::msg::Point> anchorPoseMap;

    std::unordered_map<int, Position3D> anchorPoseMap;

};

#endif
