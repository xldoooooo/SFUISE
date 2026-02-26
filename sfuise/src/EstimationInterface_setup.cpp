#include "EstimationInterface.h"
#include "EstimationInterface_topics.h"
#include "rtls_adapters.h"

#include "fraunhofer_rtls_flare/msg/anchorlist.hpp"
#include "fraunhofer_rtls_flare/msg/rtls_stick.hpp"

void EstimationInterface::setupImuInterface()
{
    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        topic_imu, 400,
        std::bind(&EstimationInterface::getImuCallback, this, std::placeholders::_1));
    pub_imu = this->create_publisher<sensor_msgs::msg::Imu>(EstimationInterfaceTopics::kTopicImuOut, 400);
}

void EstimationInterface::setupUwbInterface()
{
    if (topic_uwb == EstimationInterfaceTopics::kTopicParamUwbTdoa) {
        sub_uwb = this->create_subscription<cf_msgs::msg::Tdoa>(
            topic_uwb, 400,
            std::bind(&EstimationInterface::getTdoaUTILCallback, this, std::placeholders::_1));
        pub_uwb_tdoa = this->create_publisher<cf_msgs::msg::Tdoa>(EstimationInterfaceTopics::kTopicUwbTdoaOut, 400);
    } else if (topic_uwb == EstimationInterfaceTopics::kTopicParamUwbToa) {
        setupRtlsUwbSubscriber(topic_uwb);
        pub_uwb_toa = this->create_publisher<isas_msgs::msg::RTLSStick>(EstimationInterfaceTopics::kTopicUwbToaOut, 400);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Wrong UWB format!");
        rclcpp::shutdown();
        return;
    }
}

void EstimationInterface::setupAnchorInterface()
{
    if (if_tdoa) {
        return;
    }
    if (topic_anchor_list == EstimationInterfaceTopics::kTopicParamAnchorList) {
        setupRtlsAnchorSubscriber(topic_anchor_list);
    } else {
        RCLCPP_ERROR(this->get_logger(), "Anchor list not available!");
        rclcpp::shutdown();
        return;
    }
}

void EstimationInterface::setupRtlsUwbSubscriber(const std::string& uwb_type)
{
    if (if_fraunhofer_msg) {
        sub_uwb = this->create_subscription<fraunhofer_rtls_flare::msg::RTLSStick>(
            uwb_type, 400,
            [this](const fraunhofer_rtls_flare::msg::RTLSStick::SharedPtr uwb_msg) {
                handleToaMessage(RTLSAdapters::fraunhoferToIsas(*uwb_msg));
            });
    } else {
        sub_uwb = this->create_subscription<isas_msgs::msg::RTLSStick>(
            uwb_type, 400,
            std::bind(&EstimationInterface::getToaISASCallback, this, std::placeholders::_1));
    }
}

void EstimationInterface::setupRtlsAnchorSubscriber(const std::string& anchor_type)
{
    if (if_fraunhofer_msg) {
        sub_anchor = this->create_subscription<fraunhofer_rtls_flare::msg::Anchorlist>(
            anchor_type, 400,
            [this](const fraunhofer_rtls_flare::msg::Anchorlist::SharedPtr anchor_msg) {
                handleAnchorListMessage(RTLSAdapters::fraunhoferToIsas(*anchor_msg));
            });
    } else {
        sub_anchor = this->create_subscription<isas_msgs::msg::Anchorlist>(
            anchor_type, 400,
            std::bind(&EstimationInterface::getAnchorListFromISASCallback, this, std::placeholders::_1));
    }
}

void EstimationInterface::setupGtInterface()
{
    if (topic_ground_truth == EstimationInterfaceTopics::kTopicParamGtIsas) {
        sub_gt = this->create_subscription<geometry_msgs::msg::TransformStamped>(
            topic_ground_truth, 1000,
            std::bind(&EstimationInterface::getGtFromISASCallback, this, std::placeholders::_1));
    } else if (topic_ground_truth == EstimationInterfaceTopics::kTopicParamGtUtil) {
        sub_gt = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
            topic_ground_truth, 1000,
            std::bind(&EstimationInterface::getGtFromUTILCallback, this, std::placeholders::_1));
    }
}
