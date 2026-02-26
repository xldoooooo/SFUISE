/**
 * @file SplineFusion.h
 * @brief 样条融合节点 - 基于 B-spline 的传感器融合
 */
#pragma once

// C++ 标准库
#include <cstdint>
#include <string>
#include <vector>

// ROS2
#include <rclcpp/rclcpp.hpp>

// ROS2 消息
#include "cf_msgs/msg/tdoa.hpp"
#include "isas_msgs/msg/anchorlist.hpp"
#include "isas_msgs/msg/rtls_stick.hpp"
#include "sensor_msgs/msg/imu.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sfuise_msgs/msg/calib.hpp"
#include "sfuise_msgs/msg/estimate.hpp"
#include "sfuise_msgs/msg/spline.hpp"
#include "std_msgs/msg/int64.hpp"

// 项目内部
#include "Linearizer.h"
#include "utils/tic_toc.h"

class SplineFusion : public rclcpp::Node
{

  public:

    SplineFusion();
    void run();
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW

  private:

    static constexpr double NS_TO_S = 1e-9;

    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::Subscription<isas_msgs::msg::Anchorlist>::SharedPtr sub_anchor;
    rclcpp::SubscriptionBase::SharedPtr sub_uwb;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_knots_active;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr pub_knots_inactive;
    rclcpp::Publisher<sfuise_msgs::msg::Calib>::SharedPtr pub_calib;
    rclcpp::Publisher<sfuise_msgs::msg::Estimate>::SharedPtr pub_est;
    rclcpp::Publisher<std_msgs::msg::Int64>::SharedPtr pub_start_time;

    Parameters param;
    CalibParam calib_param;

    Eigen::aligned_deque<TOAData> toa_buff;
    Eigen::aligned_deque<TDOAData> tdoa_buff;
    Eigen::aligned_deque<ImuData> imu_buff;
    Eigen::aligned_deque<ImuData> imu_window;
    Eigen::aligned_deque<TOAData> toa_window;
    Eigen::aligned_deque<TDOAData> tdoa_window;

    bool if_anchor_ini;
    bool if_tdoa;
    bool if_uwb_only;

    size_t window_count;
    int window_size;
    int n_window_calib;

    int64_t dt_ns;
    int64_t bag_start_time;
    int64_t last_imu_t_ns;
    int64_t next_knot_TimeNs;

    enum SolverFlag {
        INITIAL,
        FULLSIZE
    };
    SolverFlag solver_flag;
    SplineState spline_local;

    size_t bias_block_offset;
    size_t gravity_block_offset;
    size_t hess_size;
    bool pose_fixed;
    int max_iter;
    double lambda;
    double lambda_vee;
    double average_runtime;
    std::vector<double> v_toa_offset;

    void readParameters();
    void getImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg);
    void getTdoaCallback(const cf_msgs::msg::Tdoa::SharedPtr msg);
    void getToaCallback(const isas_msgs::msg::RTLSStick::SharedPtr uwb_msg);
    void getAnchorCallback(const isas_msgs::msg::Anchorlist::SharedPtr anchor_msg);

    template <typename type_data>
    void updateMeasurements(Eigen::aligned_deque<type_data>& data_window, Eigen::aligned_deque<type_data>& data_buff)
    {
        int64_t t_window_l = spline_local.minTimeNs();
        if (!data_window.empty()) {
            while (!data_window.empty() && data_window.front().time_ns < t_window_l) {
                data_window.pop_front();
            }
        }
        int64_t t_window_r = spline_local.maxTimeNs();
        for (size_t i = 0; i < data_buff.size(); i++) {
            const auto& v = data_buff.at(i);
            if (v.time_ns >= t_window_l && v.time_ns <= t_window_r) {
                data_window.push_back(v);
            } else if (v.time_ns > t_window_r) {
                break;
            }
        }
        while (!data_buff.empty() && data_buff.front().time_ns <= t_window_r) {
            data_buff.pop_front();
        }
    }

    bool initialization();
    bool appendOneControlPointFromBuffers();
    void trySetStartupParameters(bool& param_set);
    void tryInitializeControlPoints(const bool param_set, bool& initialize_control_point);
    Eigen::Vector3d tdoaMultilateration(double t_s) const;
    void findClosestNWithOrderedID(double t_s, int N, size_t* idx) const;
    bool optimization();
    bool setParameters();
    void integrateStep(int64_t prevTime, int64_t dt_, const ImuData& imu, Eigen::Matrix3d& Rs, Eigen::Vector3d& Ps, Eigen::Vector3d& Vs);
    void integration(const int64_t curTime, Eigen::Quaterniond& qs, Eigen::Vector3d& Ps);
    bool getIMUInterval(int64_t t0, int64_t t1, std::vector<ImuData>& imu_vec);
    void displayControlPoints();
    geometry_msgs::msg::Point32 getPointMsg(Eigen::Vector3d p);
    bool optimize(const int iter);
    void updateUwbRejectFlag();
    void updateCalibrationFlags();
    void updateMeasurementWindows();
    bool shouldFixPose() const;
    void updateLinearizerSize();
    void applyIncFull(Eigen::VectorXd& inc_full);
};
