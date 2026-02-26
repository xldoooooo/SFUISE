#include "EstimationInterface.h"

#include <cmath>

#include "utils/node_msg_utils.h"

sensor_msgs::msg::Imu EstimationInterface::normalizeImuMessage(const sensor_msgs::msg::Imu& imu_msg) const
{
    sensor_msgs::msg::Imu imu_ds_msg;
    imu_ds_msg.header = imu_msg.header;

    Eigen::Vector3d acc(imu_msg.linear_acceleration.x, imu_msg.linear_acceleration.y, imu_msg.linear_acceleration.z);
    if (acc_ratio) acc *= 9.81;
    imu_ds_msg.linear_acceleration.x = acc[0];
    imu_ds_msg.linear_acceleration.y = acc[1];
    imu_ds_msg.linear_acceleration.z = acc[2];

    Eigen::Vector3d gyro(imu_msg.angular_velocity.x, imu_msg.angular_velocity.y, imu_msg.angular_velocity.z);
    if (gyro_unit) gyro *= M_PI / 180.0;
    imu_ds_msg.angular_velocity.x = gyro[0];
    imu_ds_msg.angular_velocity.y = gyro[1];
    imu_ds_msg.angular_velocity.z = gyro[2];

    return imu_ds_msg;
}

void EstimationInterface::appendGtPose(const int64_t t_ns, const Eigen::Quaterniond& q, const Eigen::Vector3d& pos)
{
    Eigen::Quaterniond q_copy = q;
    Eigen::Vector3d pos_copy = pos;
    gt.push_back(PoseData(t_ns, q_copy, pos_copy));
}

void EstimationInterface::processToaAndPublish(const isas_msgs::msg::RTLSStick& toa_msg, int64_t& last_uwb)
{
    int64_t t_ns = NodeMsgUtils::headerToNs(toa_msg.header);
    if (sampleData(t_ns, last_uwb, uwb_sample_coeff, uwb_frequency)) {
        pub_uwb_toa->publish(toa_msg);
        last_uwb = t_ns;
    }
}

void EstimationInterface::processTdoaAndPublish(const cf_msgs::msg::Tdoa& tdoa_msg, int64_t& last_uwb)
{
    int64_t t_ns = NodeMsgUtils::headerToNs(tdoa_msg.header);
    if (sampleData(t_ns, last_uwb, uwb_sample_coeff, uwb_frequency)) {
        pub_uwb_tdoa->publish(tdoa_msg);
        last_uwb = t_ns;
    }
}

bool EstimationInterface::sampleData(const int64_t t_ns, const int64_t last_t_ns, const double coeff, const double frequency) const
{
    if (coeff == 0) return false;
    if (coeff == 1) return true;
    int64_t dt = 1e9 / (coeff * frequency);
    return (t_ns - last_t_ns > dt - 1e5);
}

PoseData EstimationInterface::getPoseInUWB(SplineState& spline, int64_t t_ns)
{
    Eigen::Quaterniond orient_interp;
    Eigen::Vector3d t_interp = spline.itpPosition(t_ns);
    spline.itpQuaternion(t_ns, &orient_interp);
    Eigen::Quaterniond q = calib_param.q_nav_uwb * orient_interp;
    Eigen::Vector3d t = calib_param.q_nav_uwb * (orient_interp * calib_param.offset + t_interp) + calib_param.t_nav_uwb;
    return PoseData(t_ns, q, t);
}
