#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>
#include <rclcpp/rclcpp.hpp>

namespace ParamUtils
{

template <typename T>
inline T declare(rclcpp::Node& node, const std::string& key, const T& default_value)
{
    return node.declare_parameter<T>(key, default_value);
}

inline Eigen::Vector3d declareVector3(
    rclcpp::Node& node,
    const std::string& key,
    const std::vector<double>& default_value)
{
    const auto values = node.declare_parameter<std::vector<double>>(key, default_value);
    if (values.size() != 3) {
        RCLCPP_ERROR(node.get_logger(), "Parameter '%s' must contain exactly 3 elements.", key.c_str());
        rclcpp::shutdown();
        return Eigen::Vector3d::Zero();
    }
    return Eigen::Vector3d(values[0], values[1], values[2]);
}

} // namespace ParamUtils
