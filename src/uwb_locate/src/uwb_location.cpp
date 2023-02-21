#include "uwb_location.hpp"

UWBLocation::UWBLocation() : Node("uwb_location")
{
    this->load_anchors_pos();

    subscription_ = this->create_subscription<uwb_interfaces::msg::UWBData>(
        "/uwbData", 10, std::bind(&UWBLocation::topic_callback, this, std::placeholders::_1));

    msgPublisher_ = this->create_publisher<uwb_interfaces::msg::UWBLocationData>("uwbLocationRes", 10);
}

void UWBLocation::topic_callback(const uwb_interfaces::msg::UWBData::SharedPtr msg)
{

    std::string labelName = msg->label_name;

    std::unordered_map<int, double> uwbDistance;
    for (long unsigned int i = 0; i < msg->distances.size(); i++)
    {
        uwbDistance[msg->distances[i].anchor_id] = msg->distances[i].distance;
    }

    Position3D estimatedRes;
    LOCATOR_RETURN calState;
    calState = calculate_pos_robust_ransac(anchorPoseMap, uwbDistance, estimatedRes);
    if (calState == CALCULATE_SUCCESS)
    {
        RCLCPP_INFO(this->get_logger(), "label: %s uwb location success. res: x: %f, y: %f", 
        labelName.c_str(), estimatedRes.x, estimatedRes.y);
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "label: %s uwb location failed (type: %d). res: x: %f, y: %f", 
        labelName.c_str(),calState, estimatedRes.x, estimatedRes.y);
    }

    uwb_interfaces::msg::UWBLocationData data;

    data.set__label_name(labelName);

    data.set__x(estimatedRes.x);
    data.set__y(estimatedRes.y);

    msgPublisher_->publish(data);
}

void UWBLocation::load_anchors_pos()
{
    std::string packageShareDirectory = ament_index_cpp::get_package_share_directory("uwb_locate");

    std::string anchorConfigFilePath = packageShareDirectory + "/config/anchor.xml";

    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(anchorConfigFilePath.c_str()) != 0)
    {
        RCLCPP_INFO(this->get_logger(), "load anchor config file failed.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "load anchor config file successed.");
    }

    tinyxml2::XMLElement *anchor = doc.RootElement()->FirstChildElement("anchor");

    while (anchor)
    {
        int id = atoi(anchor->FirstAttribute()->Value());

        tinyxml2::XMLElement *attr = anchor->FirstChildElement();

        // geometry_msgs::msg::Point p;
        // p.set__x(std::stod(attr->GetText()));
        // attr = attr->NextSiblingElement();
        // p.set__y(std::stod(attr->GetText()));
        // attr = attr->NextSiblingElement();
        // p.set__z(std::stod(attr->GetText()));

        Position3D p;
        p.x = std::stod(attr->GetText());
        attr = attr->NextSiblingElement();
        p.y = std::stod(attr->GetText());
        attr = attr->NextSiblingElement();
        p.z = std::stod(attr->GetText());

        anchorPoseMap[id] = p;

        RCLCPP_INFO(this->get_logger(), "load anchor id: %d position:%f %f %f", id, anchorPoseMap[id].x, anchorPoseMap[id].y, anchorPoseMap[id].z);

        anchor = anchor->NextSiblingElement();
    }
}

