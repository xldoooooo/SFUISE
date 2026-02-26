#include "SplineFusion.h"
#include "SplineFusion_topics.h"

SplineFusion::SplineFusion() : rclcpp::Node("SplineFusion")
{
    if_anchor_ini = false;
    average_runtime = 0;
    window_count = 0;
    solver_flag = INITIAL;
    readParameters();
    if (!rclcpp::ok()) return;
    sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
        SplineFusionTopics::kTopicImuIn, 1000,
        std::bind(&SplineFusion::getImuCallback, this, std::placeholders::_1));
    sub_anchor = this->create_subscription<isas_msgs::msg::Anchorlist>(
        SplineFusionTopics::kTopicAnchorIn, 1000,
        std::bind(&SplineFusion::getAnchorCallback, this, std::placeholders::_1));
    if (if_tdoa) {
        sub_uwb = this->create_subscription<cf_msgs::msg::Tdoa>(
            SplineFusionTopics::kTopicTdoaIn, 1000,
            std::bind(&SplineFusion::getTdoaCallback, this, std::placeholders::_1));
    } else {
        sub_uwb = this->create_subscription<isas_msgs::msg::RTLSStick>(
            SplineFusionTopics::kTopicToaIn, 1000,
            std::bind(&SplineFusion::getToaCallback, this, std::placeholders::_1));
    }
    pub_knots_active = this->create_publisher<sensor_msgs::msg::PointCloud>(SplineFusionTopics::kTopicKnotsActive, 1000);
    pub_knots_inactive = this->create_publisher<sensor_msgs::msg::PointCloud>(SplineFusionTopics::kTopicKnotsInactive, 1000);
    pub_calib = this->create_publisher<sfuise_msgs::msg::Calib>(SplineFusionTopics::kTopicCalibOut, 100);
    pub_est = this->create_publisher<sfuise_msgs::msg::Estimate>(SplineFusionTopics::kTopicEstimateOut, 1000);
    pub_start_time = this->create_publisher<std_msgs::msg::Int64>(SplineFusionTopics::kTopicStartTimeOut, 1000);
}

void SplineFusion::run()
{
    static int num_window = 0;
    TicToc t_window;
    if (initialization()) {
        displayControlPoints();
        optimization();
        double t_consum = t_window.toc();
        average_runtime = (t_consum + double(num_window) * average_runtime) / double(num_window + 1);
        num_window++;
        if ((int)window_count <= n_window_calib) {
            sfuise_msgs::msg::Calib calib_msg;
            calib_msg.q_nav_uwb.w = calib_param.q_nav_uwb.w();
            calib_msg.q_nav_uwb.x = calib_param.q_nav_uwb.x();
            calib_msg.q_nav_uwb.y = calib_param.q_nav_uwb.y();
            calib_msg.q_nav_uwb.z = calib_param.q_nav_uwb.z();
            calib_msg.t_nav_uwb.x = calib_param.t_nav_uwb[0];
            calib_msg.t_nav_uwb.y = calib_param.t_nav_uwb[1];
            calib_msg.t_nav_uwb.z = calib_param.t_nav_uwb[2];
            geometry_msgs::msg::Point offset_msg;
            offset_msg.x = calib_param.offset.x();
            offset_msg.y = calib_param.offset.y();
            offset_msg.z = calib_param.offset.z();
            calib_msg.t_tag_body_set = offset_msg;
            pub_calib->publish(calib_msg);
        }
        if (spline_local.numKnots() >= (size_t)window_size) {
            window_count++;
            if (solver_flag == INITIAL) {
                solver_flag = FULLSIZE;
            }
        }
        sfuise_msgs::msg::Spline spline_msg;
        spline_local.getSplineMsg(spline_msg);
        sfuise_msgs::msg::Estimate est_msg;
        est_msg.spline = spline_msg;
        est_msg.if_full_window.data = (solver_flag != INITIAL);
        est_msg.runtime.data = average_runtime;
        pub_est->publish(est_msg);
        displayControlPoints();
        if (solver_flag == FULLSIZE) spline_local.removeOneOldState();
    }
}
