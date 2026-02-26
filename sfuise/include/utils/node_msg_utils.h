#pragma once

#include <cstdint>

#include "builtin_interfaces/msg/time.hpp"
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

} // namespace NodeMsgUtils
