#include "EstimationInterface.h"

#include "utils/param_keys.h"
#include "utils/param_utils.h"

void EstimationInterface::readParamsInterface()
{
    anchor_init_count = 0;
    if_tdoa = ParamUtils::declare<bool>(*this, ParamKeys::EstimationInterface::kIfTdoa, false);
    if_fraunhofer_msg = ParamUtils::declare<bool>(*this, ParamKeys::EstimationInterface::kIfFraunhoferMsg, false);
    topic_imu = ParamUtils::declare<std::string>(*this, ParamKeys::EstimationInterface::kTopicImu, "");
    topic_uwb = ParamUtils::declare<std::string>(*this, ParamKeys::EstimationInterface::kTopicUwb, "");
    topic_anchor_list = ParamUtils::declare<std::string>(*this, ParamKeys::EstimationInterface::kTopicAnchorList, "");
    topic_ground_truth = ParamUtils::declare<std::string>(*this, ParamKeys::EstimationInterface::kTopicGroundTruth, "");
    imu_sample_coeff = ParamUtils::declare<double>(*this, ParamKeys::EstimationInterface::kImuSampleCoeff, 1.0);
    uwb_sample_coeff = ParamUtils::declare<double>(*this, ParamKeys::EstimationInterface::kUwbSampleCoeff, 1.0);
    imu_frequency = ParamUtils::declare<double>(*this, ParamKeys::EstimationInterface::kImuFrequency, 100.0);
    uwb_frequency = ParamUtils::declare<double>(*this, ParamKeys::EstimationInterface::kUwbFrequency, 100.0);
    gyro_unit = ParamUtils::declare<bool>(*this, ParamKeys::EstimationInterface::kGyroUnit, false);
    acc_ratio = ParamUtils::declare<bool>(*this, ParamKeys::EstimationInterface::kAccRatio, false);
    if (uwb_sample_coeff == 0) {
        RCLCPP_ERROR(this->get_logger(), "Parameter 'uwb_sample_coeff' cannot be 0!");
        rclcpp::shutdown();
        return;
    }
    if (if_tdoa) {
        anchor_path = ParamUtils::declare<std::string>(*this, ParamKeys::EstimationInterface::kAnchorPath, "");
        getAnchorListFromUTIL(anchor_path);
    }
}

void EstimationInterface::getEstCallback(const sfuise_msgs::msg::Estimate::SharedPtr est_msg)
{
    sfuise_msgs::msg::Spline spline_msg = est_msg->spline;
    SplineState spline_w;
    spline_w.init(spline_msg.dt, 0, spline_msg.start_t, spline_msg.start_idx);
    for (const auto knot : spline_msg.knots) {
        Eigen::Vector3d pos(knot.position.x, knot.position.y, knot.position.z);
        Eigen::Quaterniond quat(knot.orientation.w, knot.orientation.x, knot.orientation.y, knot.orientation.z);
        Eigen::Matrix<double, 6, 1> bias;
        bias << knot.bias_acc.x, knot.bias_acc.y, knot.bias_acc.z,
            knot.bias_gyro.x, knot.bias_gyro.y, knot.bias_gyro.z;
        spline_w.addOneStateKnot(quat, pos, bias);
    }
    for (int i = 0; i < 3; i++) {
        sfuise_msgs::msg::Knot idle = spline_msg.idles[i];
        Eigen::Vector3d t_idle(idle.position.x, idle.position.y, idle.position.z);
        Eigen::Quaterniond q_idle(idle.orientation.w, idle.orientation.x, idle.orientation.y, idle.orientation.z);
        Eigen::Matrix<double, 6, 1> b_idle;
        b_idle << idle.bias_acc.x, idle.bias_acc.y, idle.bias_acc.z, idle.bias_gyro.x, idle.bias_gyro.y, idle.bias_gyro.z;
        spline_w.setIdles(i, t_idle, q_idle, b_idle);
    }
    spline_global.updateKnots(&spline_w);
    if (if_nav_uwb) pubOpt(spline_w, !est_msg->if_full_window.data);
    average_runtime = est_msg->runtime.data;
}

void EstimationInterface::pubOpt(SplineState& spline_local, const bool if_window_full)
{
    int64_t min_t = spline_local.minTimeNs();
    int64_t max_t = spline_local.maxTimeNs();
    static int cnt = 0;
    if (!if_window_full) {
        for (auto v : opt_window) {
            if (v.time_ns < min_t) {
                opt_old.push_back(v);
                opt_old_path.poses.push_back(CommonUtils::pose2msg(v.time_ns, v.pos, v.orient));
            }
        }
    } else {
        cnt = 0;
    }
    opt_window.clear();
    nav_msgs::msg::Path opt_window_path;
    opt_window_path.header.frame_id = "map";
    for (size_t i = cnt; i < gt.size(); i++) {
        int64_t t_ns = gt.at(i).time_ns;
        if (t_ns < min_t) {
            cnt = i;
            continue;
        } else if (t_ns > max_t) {
            break;
        }
        PoseData pose_tf = getPoseInUWB(spline_local, t_ns);
        opt_window.push_back(pose_tf);
        opt_window_path.poses.push_back(CommonUtils::pose2msg(t_ns, pose_tf.pos, pose_tf.orient));
    }
    pub_opt_old->publish(opt_old_path);
    pub_opt_window->publish(opt_window_path);
    opt_pose_vis.pubPose(opt_window.back().pos, opt_window.back().orient, pub_opt_pose, opt_window_path.header);
}

void EstimationInterface::getImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    static int64_t last_imu = 0;
    int64_t t_ns = getMsgTimeNs(imu_msg->header);
    if (sampleData(t_ns, last_imu, imu_sample_coeff, imu_frequency)) {
        last_imu = t_ns;
        pub_imu->publish(normalizeImuMessage(*imu_msg));
    }
}

void EstimationInterface::getCalibCallback(const sfuise_msgs::msg::Calib::SharedPtr calib_msg)
{
    if_nav_uwb = true;
    calib_param.q_nav_uwb = Eigen::Quaterniond(calib_msg->q_nav_uwb.w, calib_msg->q_nav_uwb.x, calib_msg->q_nav_uwb.y, calib_msg->q_nav_uwb.z);
    calib_param.t_nav_uwb = Eigen::Vector3d(calib_msg->t_nav_uwb.x, calib_msg->t_nav_uwb.y, calib_msg->t_nav_uwb.z);
    calib_param.gravity = Eigen::Vector3d(calib_msg->gravity.x, calib_msg->gravity.y, calib_msg->gravity.z);
    calib_param.offset = Eigen::Vector3d(calib_msg->t_tag_body_set.x, calib_msg->t_tag_body_set.y, calib_msg->t_tag_body_set.z);
}

void EstimationInterface::getToaISASCallback(const isas_msgs::msg::RTLSStick::SharedPtr uwb_msg)
{
    handleToaMessage(*uwb_msg);
}

void EstimationInterface::handleToaMessage(const isas_msgs::msg::RTLSStick& uwb_msg)
{
    static int64_t last_uwb = 0;
    processToaAndPublish(uwb_msg, last_uwb);
}

void EstimationInterface::getTdoaUTILCallback(const cf_msgs::msg::Tdoa::SharedPtr msg)
{
    static int64_t last_uwb = 0;
    processTdoaAndPublish(*msg, last_uwb);
}

void EstimationInterface::getGtFromISASCallback(const geometry_msgs::msg::TransformStamped::SharedPtr gt_msg)
{
    Eigen::Quaterniond q(gt_msg->transform.rotation.w, gt_msg->transform.rotation.x,
        gt_msg->transform.rotation.y, gt_msg->transform.rotation.z);
    Eigen::Vector3d pos(gt_msg->transform.translation.x, gt_msg->transform.translation.y, gt_msg->transform.translation.z);
    appendGtPose(getMsgTimeNs(gt_msg->header), q, pos);
}

void EstimationInterface::getGtFromUTILCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr gt_msg)
{
    Eigen::Quaterniond q(gt_msg->pose.pose.orientation.w, gt_msg->pose.pose.orientation.x,
        gt_msg->pose.pose.orientation.y, gt_msg->pose.pose.orientation.z);
    Eigen::Vector3d pos(gt_msg->pose.pose.position.x, gt_msg->pose.pose.position.y, gt_msg->pose.pose.position.z);
    appendGtPose(getMsgTimeNs(gt_msg->header), q, pos);
}

void EstimationInterface::getAnchorListFromISASCallback(const isas_msgs::msg::Anchorlist::SharedPtr anchor_msg)
{
    handleAnchorListMessage(*anchor_msg);
}

void EstimationInterface::handleAnchorListMessage(const isas_msgs::msg::Anchorlist& anchor_msg)
{
    accumulateAnchorList(anchor_msg);
}

void EstimationInterface::startCallBack(const std_msgs::msg::Int64::SharedPtr start_time_msg)
{
    int64_t bag_start_time = start_time_msg->data;
    spline_global.init(dt_ns, 0, bag_start_time);
}
