#include "EstimationInterface.h"
#include "EstimationInterface_topics.h"

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
        sub_uwb = this->create_subscription<isas_msgs::msg::RTLSStick>(
            topic_uwb, 400,
            [this](const isas_msgs::msg::RTLSStick::SharedPtr uwb_msg) {
                handleToaMessage(*uwb_msg);
            });
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
        sub_anchor = this->create_subscription<isas_msgs::msg::Anchorlist>(
            topic_anchor_list, 400,
            [this](const isas_msgs::msg::Anchorlist::SharedPtr anchor_msg) {
                handleAnchorListMessage(*anchor_msg);
            });
    } else {
        RCLCPP_ERROR(this->get_logger(), "Anchor list not available!");
        rclcpp::shutdown();
        return;
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
