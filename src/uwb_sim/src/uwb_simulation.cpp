#include "uwb_simulation.hpp"

UWBSimulation::UWBSimulation() : Node("uwb_simulation")
{
    this->load_anchors_pos();

    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/odom", 10, std::bind(&UWBSimulation::topic_callback, this, std::placeholders::_1));

    msgPublisher_ = this->create_publisher<uwb_interfaces::msg::UWBData>("uwbData", 10);
}

void UWBSimulation::topic_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    robotPose = msg->pose.pose.position;
    RCLCPP_INFO(this->get_logger(), "position: %f %f %f", robotPose.x, robotPose.y, robotPose.z);
    this->uwb_simulate();
}

void UWBSimulation::load_anchors_pos()
{
    std::string packageShareDirectory = ament_index_cpp::get_package_share_directory("uwb_sim");

    std::string anthorConfigFilePath = packageShareDirectory + "/config/anthor.xml";

    tinyxml2::XMLDocument doc;

    if (doc.LoadFile(anthorConfigFilePath.c_str()) != 0)
    {
        RCLCPP_INFO(this->get_logger(), "load anthor config file failed.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "load anthor config file successed.");
    }

    tinyxml2::XMLElement *anthor = doc.RootElement()->FirstChildElement("anthor");

    while (anthor)
    {
        int id = atoi(anthor->FirstAttribute()->Value());

        tinyxml2::XMLElement *attr = anthor->FirstChildElement();

        geometry_msgs::msg::Point p;

        p.set__x(std::stod(attr->GetText()));
        attr = attr->NextSiblingElement();
        p.set__y(std::stod(attr->GetText()));
        attr = attr->NextSiblingElement();
        p.set__z(std::stod(attr->GetText()));

        RCLCPP_INFO(this->get_logger(), "load anthor id: %d position:%f %f %f", id, p.x, p.y, p.z);
        anthorPoseMap[id] = p;
        anthor = anthor->NextSiblingElement();
    }
}

void UWBSimulation::uwb_simulate()
{
    std::unordered_map<int, double> realDistance;

    for (auto it : anthorPoseMap)
    {
        int id = it.first;
        geometry_msgs::msg::Point anthorPose = it.second;
        realDistance[id] = sqrtf(
            pow((robotPose.x - anthorPose.x), 2) +
            pow((robotPose.y - anthorPose.y), 2) +
            pow((robotPose.z - anthorPose.z), 2));
    }

    std::unordered_map<int, double> simDistance;
    std::normal_distribution<double> distribution_normal(0., 0.1);

    for (auto it : realDistance)
    {
        int id = it.first;
        simDistance[id] = realDistance[id] + distribution_normal(tandomGenerator);

        // RCLCPP_INFO(this->get_logger(), "Id: %d real distance : %f.", id, realDistance[id]);
        // RCLCPP_INFO(this->get_logger(), "Id: %d sim distance  : %f.", id, simDistance[id]);
    }

    
    uwb_interfaces::msg::UWBData msg;

    int robotId = 0;

    msg.robot_id = robotId;

    for (auto it : realDistance)
    {
        int id = it.first; //anthor id

        uwb_interfaces::msg::UWBDistance distance;
        distance.anthor_id = id;
        distance.robot_id = msg.robot_id;
        distance.distance = simDistance[distance.anthor_id];
        
        msg.distances.push_back(distance);
    }
    
    msgPublisher_->publish(msg);

}
