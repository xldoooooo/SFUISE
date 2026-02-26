#include "EstimationInterface.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimationInterface>();
    RCLCPP_INFO(node->get_logger(), "\033[1;32m---->\033[0m Starting EstimationInterface.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
