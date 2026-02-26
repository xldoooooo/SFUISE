/**
 * @file EstimationInterface.h
 * @brief 估计接口节点 - 传感器数据预处理和结果可视化
 */
#pragma once

// C++ 标准库
#include <cstdint>
#include <string>

// ROS2
#include <rclcpp/rclcpp.hpp>

// ROS2 消息
#include "cf_msgs/msg/tdoa.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "isas_msgs/msg/anchor_position.hpp"
#include "isas_msgs/msg/anchorlist.hpp"
#include "isas_msgs/msg/rtls_range.hpp"
#include "isas_msgs/msg/rtls_stick.hpp"
#include "nav_msgs/msg/path.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sfuise_msgs/msg/calib.hpp"
#include "sfuise_msgs/msg/estimate.hpp"
#include "sfuise_msgs/msg/spline.hpp"
#include "std_msgs/msg/int64.hpp"
#include "visualization_msgs/msg/marker.hpp"

// 项目内部
#include "SplineState.h"
#include "utils/PoseVisualization.h"

class EstimationInterface : public rclcpp::Node
{

  public:

    EstimationInterface();

  private:

    int64_t dt_ns;
    bool initialized_anchor;
    bool if_tdoa;
    bool if_fraunhofer_msg;
    bool if_nav_uwb;
    CalibParam calib_param;
    Eigen::aligned_vector<PoseData> gt;
    Eigen::aligned_map<uint16_t, Eigen::Vector3d> anchor_list;
    double imu_sample_coeff;
    double uwb_sample_coeff;
    double imu_frequency;
    double uwb_frequency;
    double average_runtime;
    bool gyro_unit;
    bool acc_ratio;
    int anchor_init_count;
    std::string topic_imu;
    std::string topic_uwb;
    std::string topic_anchor_list;
    std::string topic_ground_truth;
    std::string anchor_path;
    SplineState spline_global;
    Eigen::aligned_vector<PoseData> opt_old;
    Eigen::aligned_vector<PoseData> opt_window;

    rclcpp::TimerBase::SharedPtr timer_anchor;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_start;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::SubscriptionBase::SharedPtr sub_uwb;
    rclcpp::SubscriptionBase::SharedPtr sub_anchor;
    rclcpp::SubscriptionBase::SharedPtr sub_gt;
    rclcpp::Subscription<sfuise_msgs::msg::Calib>::SharedPtr sub_calib;
    rclcpp::Subscription<sfuise_msgs::msg::Estimate>::SharedPtr sub_est;

    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<cf_msgs::msg::Tdoa>::SharedPtr pub_uwb_tdoa;
    rclcpp::Publisher<isas_msgs::msg::RTLSStick>::SharedPtr pub_uwb_toa;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr anchor_pos_pub;
    rclcpp::Publisher<isas_msgs::msg::Anchorlist>::SharedPtr anchor_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_opt_old;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_opt_window;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_opt_pose;

    nav_msgs::msg::Path opt_old_path;
    PoseVisualization opt_pose_vis;

    void setupImuInterface();
    void setupUwbInterface();
    void setupAnchorInterface();
    void setupGtInterface();
    void setupRtlsUwbSubscriber(const std::string& uwb_type);
    void setupRtlsAnchorSubscriber(const std::string& anchor_type);

    int64_t getMsgTimeNs(const std_msgs::msg::Header& header) const;
    sensor_msgs::msg::Imu normalizeImuMessage(const sensor_msgs::msg::Imu& imu_msg) const;
    void appendGtPose(const int64_t t_ns, const Eigen::Quaterniond& q, const Eigen::Vector3d& pos);
    void processToaAndPublish(const isas_msgs::msg::RTLSStick& toa_msg, int64_t& last_uwb);
    void processTdoaAndPublish(const cf_msgs::msg::Tdoa& tdoa_msg, int64_t& last_uwb);
    void handleToaMessage(const isas_msgs::msg::RTLSStick& uwb_msg);
    void handleAnchorListMessage(const isas_msgs::msg::Anchorlist& anchor_msg);

    template <typename AnchorListMsgT>
    void accumulateAnchorList(const AnchorListMsgT& anchor_msg)
    {
        if (initialized_anchor) return;
        const int num_sum = 20;
        for (const auto& anchor : anchor_msg.anchor) {
            Eigen::Vector3d anchor_pos(anchor.position.x, anchor.position.y, anchor.position.z);
            uint16_t anchor_id = anchor.id;
            if (anchor_init_count == 0) {
                anchor_list[anchor_id] = anchor_pos;
            } else {
                Eigen::Vector3d ave_pos = anchor_list[anchor_id];
                anchor_list[anchor_id] = (ave_pos * anchor_init_count + anchor_pos) / (anchor_init_count + 1);
            }
        }
        anchor_init_count++;
        if (anchor_init_count >= num_sum) {
            initialized_anchor = true;
            publishAnchor();
        }
    }

    void readParamsInterface();
    void getEstCallback(const sfuise_msgs::msg::Estimate::SharedPtr est_msg);
    void pubOpt(SplineState& spline_local, const bool if_window_full);
    void getImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void getCalibCallback(const sfuise_msgs::msg::Calib::SharedPtr calib_msg);
    void getToaISASCallback(const isas_msgs::msg::RTLSStick::SharedPtr uwb_msg);
    void getTdoaUTILCallback(const cf_msgs::msg::Tdoa::SharedPtr msg);
    void getGtFromISASCallback(const geometry_msgs::msg::TransformStamped::SharedPtr gt_msg);
    void getGtFromUTILCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr gt_msg);
    void getAnchorListFromISASCallback(const isas_msgs::msg::Anchorlist::SharedPtr anchor_msg);
    void getAnchorListFromUTIL(const std::string& anchor_path);
    bool sampleData(const int64_t t_ns, const int64_t last_t_ns, const double coeff, const double frequency) const;
    void publishAnchor();
    void startCallBack(const std_msgs::msg::Int64::SharedPtr start_time_msg);
    PoseData getPoseInUWB(SplineState& spline, int64_t t_ns);
};
