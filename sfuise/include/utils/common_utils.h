/**
 * @file common_utils.h
 * @brief 通用工具函数和类型定义
 *
 * 这个文件作为向后兼容的入口点，包含所有相关类型定义。
 * 新代码应直接包含具体的类型头文件。
 */
#pragma once

// 类型定义
#include "../types/eigen_types.h"
#include "../types/sensor_data.h"
#include "../types/calib_param.h"

// ROS2 消息转换
#include <rclcpp/rclcpp.hpp>
#include "geometry_msgs/msg/pose_stamped.hpp"

namespace sfuise {

/**
 * @brief 通用工具函数集合
 */
class CommonUtils {
public:
    /**
     * @brief 从 ROS 节点读取参数
     * @deprecated 使用 ParamUtils::declare() 代替
     */
    template <typename T>
    static T readParam(rclcpp::Node::SharedPtr node, const std::string& name)
    {
        T ans;
        if (!node->get_parameter(name, ans)) {
            RCLCPP_ERROR_STREAM(node->get_logger(), "Failed to load " << name);
            rclcpp::shutdown();
        }
        return ans;
    }

    /**
     * @brief 将位姿转换为 ROS PoseStamped 消息
     * @param t 时间戳 (纳秒)
     * @param pos 位置
     * @param orient 姿态
     * @return PoseStamped 消息
     */
    static geometry_msgs::msg::PoseStamped pose2msg(
        const int64_t t,
        const Eigen::Vector3d& pos,
        const Eigen::Quaterniond& orient)
    {
        geometry_msgs::msg::PoseStamped msg;
        msg.header.stamp = rclcpp::Time(t);
        msg.pose.position.x = pos.x();
        msg.pose.position.y = pos.y();
        msg.pose.position.z = pos.z();
        msg.pose.orientation.w = orient.w();
        msg.pose.orientation.x = orient.x();
        msg.pose.orientation.y = orient.y();
        msg.pose.orientation.z = orient.z();
        return msg;
    }
};

}  // namespace sfuise

// 向后兼容
using CommonUtils = sfuise::CommonUtils;
