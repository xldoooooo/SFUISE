#include "EstimationInterface.h"
#include "EstimationInterface_topics.h"
#include "utils/param_keys.h"
#include "utils/param_utils.h"

#include <chrono>

EstimationInterface::EstimationInterface() : rclcpp::Node("EstimationInterface")
{
    initialized_anchor = false;
    if_nav_uwb = false;
    readParamsInterface();
    if (!rclcpp::ok()) return;
    sub_start = this->create_subscription<std_msgs::msg::Int64>(
        EstimationInterfaceTopics::kTopicStartTime, 1000,
        std::bind(&EstimationInterface::startCallBack, this, std::placeholders::_1));
    setupImuInterface();
    setupUwbInterface();
    if (!rclcpp::ok()) return;
    setupAnchorInterface();
    if (!rclcpp::ok()) return;

    anchor_pos_pub = this->create_publisher<sensor_msgs::msg::PointCloud>(EstimationInterfaceTopics::kTopicAnchorVis, 1000);
    anchor_pub = this->create_publisher<isas_msgs::msg::Anchorlist>(EstimationInterfaceTopics::kTopicAnchorListOut, 1000);

    timer_anchor = this->create_wall_timer(
        std::chrono::milliseconds(10),
        std::bind(&EstimationInterface::publishAnchor, this));
    setupGtInterface();

    int control_point_fps = ParamUtils::declare<int>(*this, ParamKeys::EstimationInterface::kControlPointFps, 100);
    dt_ns = 1e9 / control_point_fps;

    sub_calib = this->create_subscription<sfuise_msgs::msg::Calib>(
        EstimationInterfaceTopics::kTopicCalib, 100,
        std::bind(&EstimationInterface::getCalibCallback, this, std::placeholders::_1));

    sub_est = this->create_subscription<sfuise_msgs::msg::Estimate>(
        EstimationInterfaceTopics::kTopicEstimate, 100,
        std::bind(&EstimationInterface::getEstCallback, this, std::placeholders::_1));

    pub_opt_old = this->create_publisher<nav_msgs::msg::Path>(EstimationInterfaceTopics::kTopicOptOld, 1000);
    pub_opt_window = this->create_publisher<nav_msgs::msg::Path>(EstimationInterfaceTopics::kTopicOptWindow, 1000);
    opt_old_path.header.frame_id = "map";
    pub_opt_pose = this->create_publisher<visualization_msgs::msg::Marker>(EstimationInterfaceTopics::kTopicOptPose, 1000);
    opt_pose_vis.setColor(85.0 / 255.0, 164.0 / 255.0, 104.0 / 255.0);
}
