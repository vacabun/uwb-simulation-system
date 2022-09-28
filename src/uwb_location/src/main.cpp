#include "rclcpp/rclcpp.hpp"
#include "uwb_locate.hpp"


int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBLocate>());
    rclcpp::shutdown();

    return 0;

}
