#include "rclcpp/rclcpp.hpp"
#include "uwb_simulation.hpp"

int main(int argc, char **argv)
{
    (void)argc;
    (void)argv;

    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<UWBSimulation>());
    rclcpp::shutdown();

    return 0;
}
