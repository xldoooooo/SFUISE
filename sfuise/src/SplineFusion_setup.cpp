#include "SplineFusion.h"

#include "utils/node_msg_utils.h"
#include "utils/param_keys.h"
#include "utils/param_utils.h"

void SplineFusion::readParameters()
{
    if_uwb_only = (ParamUtils::declare<double>(*this, ParamKeys::SplineFusion::kImuSampleCoeff, 1.0) == 0);
    param.if_opt_g = true;
    param.if_opt_transform = true;
    param.w_uwb = ParamUtils::declare<double>(*this, ParamKeys::SplineFusion::kWUwb, 1.0);
    max_iter = ParamUtils::declare<int>(*this, ParamKeys::SplineFusion::kMaxIter, 20);
    dt_ns = 1e9 / ParamUtils::declare<int>(*this, ParamKeys::SplineFusion::kControlPointFps, 100);
    if_tdoa = ParamUtils::declare<bool>(*this, ParamKeys::SplineFusion::kIfTdoa, true);
    bag_start_time = 0;
    n_window_calib = ParamUtils::declare<int>(*this, ParamKeys::SplineFusion::kNWindowCalib, 50);
    window_size = ParamUtils::declare<int>(*this, ParamKeys::SplineFusion::kWindowSize, 10);
    if (n_window_calib == 0) {
        RCLCPP_ERROR(this->get_logger(), "n_window_calib cannot be set 0.");
        rclcpp::shutdown();
        return;
    } else {
        param.q_nav_uwb_init.setIdentity();
        param.t_nav_uwb_init.setZero();
    }
    param.accel_var_inv = ParamUtils::declareVector3(*this, ParamKeys::SplineFusion::kAccelVarInv, std::vector<double>{1.0, 1.0, 1.0});
    param.bias_accel_var_inv = ParamUtils::declareVector3(*this, ParamKeys::SplineFusion::kBiasAccelVarInv, std::vector<double>{1.0, 1.0, 1.0});
    param.w_acc = ParamUtils::declare<double>(*this, ParamKeys::SplineFusion::kWAccel, 1.0);
    param.w_bias_acc = ParamUtils::declare<double>(*this, ParamKeys::SplineFusion::kWBiasAccel, 1.0);
    param.gyro_var_inv = ParamUtils::declareVector3(*this, ParamKeys::SplineFusion::kGyroVarInv, std::vector<double>{1.0, 1.0, 1.0});
    param.bias_gyro_var_inv = ParamUtils::declareVector3(*this, ParamKeys::SplineFusion::kBiasGyroVarInv, std::vector<double>{1.0, 1.0, 1.0});
    param.w_gyro = ParamUtils::declare<double>(*this, ParamKeys::SplineFusion::kWGyro, 1.0);
    param.w_bias_gyro = ParamUtils::declare<double>(*this, ParamKeys::SplineFusion::kWBiasGyro, 1.0);
    param.if_reject_uwb = ParamUtils::declare<bool>(*this, ParamKeys::SplineFusion::kIfRejectUwb, false);
    if (param.if_reject_uwb) {
        param.reject_uwb_thresh = ParamUtils::declare<double>(*this, ParamKeys::SplineFusion::kRejectUwbThresh, 1.0);
        param.reject_uwb_window_width = ParamUtils::declare<double>(*this, ParamKeys::SplineFusion::kRejectUwbWindowWidth, 1.0);
    }
    calib_param.offset = ParamUtils::declareVector3(*this, ParamKeys::SplineFusion::kOffset, std::vector<double>{0.0, 0.0, 0.0});
    if (!if_tdoa) {
        v_toa_offset = ParamUtils::declare<std::vector<double>>(*this, ParamKeys::SplineFusion::kToaOffset, std::vector<double>{});
    }
}

void SplineFusion::getImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
{
    int64_t t_ns = NodeMsgUtils::headerToNs(imu_msg->header);
    Eigen::Vector3d acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
    Eigen::Vector3d gyro(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
    ImuData imu(t_ns, gyro, acc);
    imu_buff.push_back(imu);
}

void SplineFusion::getTdoaCallback(const cf_msgs::msg::Tdoa::SharedPtr msg)
{
    TDOAData uwb(NodeMsgUtils::headerToNs(msg->header), msg->id_a, msg->id_b, msg->data);
    tdoa_buff.push_back(uwb);
}

void SplineFusion::getToaCallback(const isas_msgs::msg::RTLSStick::SharedPtr uwb_msg)
{
    int64_t t_ns = NodeMsgUtils::headerToNs(uwb_msg->header);
    for (const auto& rg : uwb_msg->ranges) {
        if (rg.ra == 0) continue;
        TOAData uwb(t_ns, rg.id, rg.range);
        toa_buff.push_back(uwb);
    }
}

void SplineFusion::getAnchorCallback(const isas_msgs::msg::Anchorlist::SharedPtr anchor_msg)
{
    if (if_anchor_ini) return;
    for (const auto& anchor : anchor_msg->anchor) {
        param.anchor_list[anchor.id] = Eigen::Vector3d(anchor.position.x, anchor.position.y, anchor.position.z);
    }
    if_anchor_ini = true;
    if (!if_tdoa) {
        int i = 0;
        for (auto it = param.anchor_list.begin(); it != param.anchor_list.end(); it++) {
            param.toa_offset[it->first] = v_toa_offset[i];
            i++;
        }
    }
}

bool SplineFusion::setParameters()
{
    if (imu_buff.empty() && !if_uwb_only) return false;
    if (imu_buff.empty() && toa_buff.empty() && tdoa_buff.empty()) {
        return false;
    } else {
        if (!imu_buff.empty()) {
            bag_start_time += imu_buff.front().time_ns;
        } else if (!toa_buff.empty()) {
            bag_start_time += toa_buff.front().time_ns;
        } else {
            bag_start_time += tdoa_buff.front().time_ns;
        }
    }
    next_knot_TimeNs = bag_start_time;
    return true;
}
