#include "uwb_simulation.hpp"

UWBSimulation::UWBSimulation() : Node("uwb_simulation")
{
    this->load_config();
    this->declare_parameter("label_name", "x500_1");

    labelName =
        this->get_parameter("label_name").get_parameter_value().get<std::string>();
    RCLCPP_INFO(this->get_logger(), "label_name: %s", labelName.c_str());

    std::string subscribeTopic = "/model/" + labelName + "/odometry";

    RCLCPP_INFO(this->get_logger(), "subscribe topic : %s", subscribeTopic.c_str());

    if (!subscribeNode.Subscribe(subscribeTopic, &UWBSimulation::topic_callback, this))
    {
        RCLCPP_ERROR(this->get_logger(), "Error subscribing to topic [%s].", subscribeTopic.c_str());
    }

    msgPublisher_ = this->create_publisher<uwb_interfaces::msg::UWBData>("uwbData", 10);
}

void UWBSimulation::topic_callback(const gz::msgs::Odometry &_msg)
{
    double x = _msg.pose().position().x();
    double y = _msg.pose().position().y();
    double z = _msg.pose().position().z();

    RCLCPP_INFO(this->get_logger(), "position: %f %f %f", x, y, z);
    this->uwb_simulate(x, y, z);
}

void UWBSimulation::load_config()
{
    std::string packageShareDirectory = ament_index_cpp::get_package_share_directory("uwb_sim");

    std::string anchorConfigFilePath = packageShareDirectory + "/config/anchor.xml";

    tinyxml2::XMLDocument anchorDoc;

    if (anchorDoc.LoadFile(anchorConfigFilePath.c_str()) != 0)
    {
        RCLCPP_INFO(this->get_logger(), "load anchor config file failed.");
    }
    else
    {
        RCLCPP_INFO(this->get_logger(), "load anchor config file successed.");
    }

    tinyxml2::XMLElement *anchor = anchorDoc.RootElement()->FirstChildElement("anchor");

    while (anchor)
    {
        int id = atoi(anchor->FirstAttribute()->Value());

        tinyxml2::XMLElement *attr = anchor->FirstChildElement();

        geometry_msgs::msg::Point p;

        p.set__x(std::stod(attr->GetText()));
        attr = attr->NextSiblingElement();
        p.set__y(std::stod(attr->GetText()));
        attr = attr->NextSiblingElement();
        p.set__z(std::stod(attr->GetText()));

        RCLCPP_INFO(this->get_logger(), "load anchor id: %d position:%f %f %f", id, p.x, p.y, p.z);
        anchorPoseMap[id] = p;
        anchor = anchor->NextSiblingElement();
    }
}

void UWBSimulation::uwb_simulate(double x, double y, double z)
{
    std::unordered_map<int, double> realDistance;

    for (auto it : anchorPoseMap)
    {
        int id = it.first;
        geometry_msgs::msg::Point anchorPose = it.second;
        realDistance[id] = sqrtf(
            pow((x - anchorPose.x), 2) +
            pow((y - anchorPose.y), 2) +
            pow((z - anchorPose.z), 2));
    }

    std::unordered_map<int, double> simDistance;
    std::normal_distribution<double> distribution_normal(0., 0.1);

    for (auto it : realDistance)
    {
        int id = it.first;
        simDistance[id] = realDistance[id] + distribution_normal(tandomGenerator);

        RCLCPP_INFO(this->get_logger(), "label name: %s anchor Id: %d real distance : %f sim distance : %f.",
                    labelName.c_str(), id, realDistance[id], simDistance[id]);
    }

    uwb_interfaces::msg::UWBData msg;

    msg.label_name = labelName;

    for (auto it : realDistance)
    {
        int id = it.first; // anchor id

        uwb_interfaces::msg::UWBDistance distance;
        distance.anchor_id = id;
        distance.label_name = msg.label_name;
        distance.distance = simDistance[distance.anchor_id];

        msg.distances.push_back(distance);
    }

    msgPublisher_->publish(msg);
}
