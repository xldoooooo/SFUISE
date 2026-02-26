#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "builtin_interfaces/msg/time.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "geometry_msgs/msg/quaternion.hpp"
#include "geometry_msgs/msg/vector3.hpp"
#include "std_msgs/msg/header.hpp"

namespace NodeMsgUtils
{

inline int64_t stampToNs(const builtin_interfaces::msg::Time& stamp)
{
    return static_cast<int64_t>(stamp.sec) * static_cast<int64_t>(1e9) + stamp.nanosec;
}

inline int64_t headerToNs(const std_msgs::msg::Header& header)
{
    return stampToNs(header.stamp);
}

inline geometry_msgs::msg::Point toPointMsg(const Eigen::Vector3d& v)
{
    geometry_msgs::msg::Point p;
    p.x = v.x();
    p.y = v.y();
    p.z = v.z();
    return p;
}

inline geometry_msgs::msg::Vector3 toVector3Msg(const Eigen::Vector3d& v)
{
    geometry_msgs::msg::Vector3 msg;
    msg.x = v.x();
    msg.y = v.y();
    msg.z = v.z();
    return msg;
}

inline geometry_msgs::msg::Quaternion toQuaternionMsg(const Eigen::Quaterniond& q)
{
    geometry_msgs::msg::Quaternion msg;
    msg.w = q.w();
    msg.x = q.x();
    msg.y = q.y();
    msg.z = q.z();
    return msg;
}

inline Eigen::Vector3d toVector3d(const geometry_msgs::msg::Point& p)
{
    return Eigen::Vector3d(p.x, p.y, p.z);
}

inline Eigen::Vector3d toVector3d(const geometry_msgs::msg::Vector3& v)
{
    return Eigen::Vector3d(v.x, v.y, v.z);
}

inline Eigen::Quaterniond toQuaterniond(const geometry_msgs::msg::Quaternion& q)
{
    return Eigen::Quaterniond(q.w, q.x, q.y, q.z);
}

} // namespace NodeMsgUtils
