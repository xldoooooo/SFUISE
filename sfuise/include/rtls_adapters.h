#pragma once

#include "fraunhofer_rtls_flare/msg/anchorlist.hpp"
#include "fraunhofer_rtls_flare/msg/rtls_stick.hpp"
#include "isas_msgs/msg/anchorlist.hpp"
#include "isas_msgs/msg/anchor_position.hpp"
#include "isas_msgs/msg/rtls_range.hpp"
#include "isas_msgs/msg/rtls_stick.hpp"

namespace RTLSAdapters
{

isas_msgs::msg::RTLSStick fraunhoferToIsas(const fraunhofer_rtls_flare::msg::RTLSStick& uwb_msg);
isas_msgs::msg::Anchorlist fraunhoferToIsas(const fraunhofer_rtls_flare::msg::Anchorlist& anchor_msg);

} // namespace RTLSAdapters
