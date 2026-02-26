#include <thread>

#include "SplineFusion.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SplineFusion>();
    RCLCPP_INFO(node->get_logger(), "\033[1;32m---->\033[0m Starting SplineFusion.");
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node);
    std::thread spinning_thread([&executor]() {
        executor.spin();
    });

    rclcpp::Rate loop_rate(1000);
    while (rclcpp::ok()) {
        node->run();
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    spinning_thread.join();
    return 0;
}
