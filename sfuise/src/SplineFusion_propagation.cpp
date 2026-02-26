#include "SplineFusion.h"

#include <chrono>
#include <limits>

bool SplineFusion::appendOneControlPointFromBuffers()
{
    int64_t min_time = 1e18;
    if (!imu_buff.empty()) min_time = imu_buff.back().time_ns;
    if (!toa_buff.empty()) min_time = std::min(toa_buff.back().time_ns, min_time);
    if (!tdoa_buff.empty()) min_time = std::min(tdoa_buff.back().time_ns, min_time);
    if (min_time <= spline_local.nextMaxTimeNs()) {
        return false;
    }

    Eigen::Quaterniond q_ini = spline_local.getKnotOrt(spline_local.numKnots() - 1);
    Eigen::Quaterniond q_ini_backup = q_ini;
    Eigen::Vector3d pos_ini = spline_local.getKnotPos(spline_local.numKnots() - 1);
    Eigen::Matrix<double, 6, 1> bias_ini = spline_local.getKnotBias(spline_local.numKnots() - 1);
    if (!if_uwb_only) {
        if (spline_local.numKnots() <= 2) {
            last_imu_t_ns = bag_start_time;
        } else {
            last_imu_t_ns = imu_window.back().time_ns;
        }
        integration(next_knot_TimeNs, q_ini, pos_ini);
    } else {
        if (if_tdoa) {
            pos_ini = tdoaMultilateration(next_knot_TimeNs * NS_TO_S);
            q_ini = Eigen::Quaterniond::Identity();
        } else {
            RCLCPP_ERROR(this->get_logger(), "UWB-only tracking only supported for TDOA data!");
            rclcpp::shutdown();
            return false;
        }
    }
    if (q_ini_backup.dot(q_ini) < 0) q_ini = Quater::negate(q_ini);
    spline_local.addOneStateKnot(q_ini, pos_ini, bias_ini);
    next_knot_TimeNs += dt_ns;
    return true;
}

void SplineFusion::trySetStartupParameters(bool& param_set)
{
    if (!param_set) {
        param_set = setParameters();
        std_msgs::msg::Int64 start_time;
        start_time.data = bag_start_time;
        pub_start_time->publish(start_time);
    }
}

void SplineFusion::tryInitializeControlPoints(const bool param_set, bool& initialize_control_point)
{
    if (!(param_set && if_anchor_ini)) {
        return;
    }
    spline_local.init(dt_ns, 0, bag_start_time);
    if (!if_uwb_only) {
        Eigen::Vector3d gravity_sum(0, 0, 0);
        size_t n_imu = imu_buff.size();
        for (size_t i = 0; i < n_imu; i++) {
            gravity_sum += imu_buff.at(i).accel;
        }
        gravity_sum /= n_imu;
        Eigen::Vector3d gravity_ave = gravity_sum.normalized() * 9.81;
        calib_param.gravity = gravity_ave;
    }
    calib_param.q_nav_uwb = param.q_nav_uwb_init;
    calib_param.t_nav_uwb = param.t_nav_uwb_init;
    initialize_control_point = true;
    int num = 1;
    for (int i = 0; i < num; i++) {
        Eigen::Quaterniond q_ini = Eigen::Quaterniond::Identity();
        Eigen::Vector3d pos_ini = Eigen::Vector3d::Zero();
        Eigen::Matrix<double, 6, 1> bias_ini = Eigen::Matrix<double, 6, 1>::Zero();
        spline_local.addOneStateKnot(q_ini, pos_ini, bias_ini);
        next_knot_TimeNs += dt_ns;
    }
}

bool SplineFusion::initialization()
{
    static bool param_set = false;
    static bool initialize_control_point = false;
    if (initialize_control_point) {
        return appendOneControlPointFromBuffers();
    } else {
        trySetStartupParameters(param_set);
        tryInitializeControlPoints(param_set, initialize_control_point);
        return false;
    }
}

Eigen::Vector3d SplineFusion::tdoaMultilateration(double t_s) const
{
    static bool set_origin = true;
    int num_data = 7;
    size_t idx[num_data];
    findClosestNWithOrderedID(t_s, num_data, idx);
    Eigen::Vector3d pos;
    Eigen::MatrixXd H(num_data, 4);
    Eigen::VectorXd b(num_data);
    Eigen::Vector3d anchor0 = param.anchor_list.at(0);
    std::vector<double> range;
    range.push_back(tdoa_buff.at(idx[0]).data);
    for (int i = 1; i < num_data; i++) {
        range.push_back(range[i - 1] + tdoa_buff.at(idx[i]).data);
    }
    for (int i = 0; i < num_data; i++) {
        TDOAData uwb = tdoa_buff.at(idx[i]);
        double range0 = range[i];
        Eigen::Vector3d anchori = param.anchor_list.at(uint16_t(uwb.idB));
        H.row(i) = Eigen::Vector4d(anchori.x() - anchor0.x(), anchori.y() - anchor0.y(), anchori.z() - anchor0.z(), range0);
        b[i] = range0 * range0 - anchori.dot(anchori) + anchor0.dot(anchor0);
    }
    H *= -2;
    Eigen::VectorXd x = (H.transpose() * H).inverse() * H.transpose() * b;
    pos = x.head(3);
    if (!set_origin) {
        pos.setZero();
        set_origin = true;
    } else {
        pos = calib_param.q_nav_uwb.inverse() * (pos - calib_param.t_nav_uwb);
    }
    return pos;
}

void SplineFusion::findClosestNWithOrderedID(double t_s, int N, size_t* idx) const
{
    std::vector<std::pair<std::pair<int, int>, double>> diff;
    for (auto it = tdoa_buff.begin(); it != tdoa_buff.end(); it++) {
        double t = it->time_ns * NS_TO_S;
        diff.push_back(std::make_pair(std::make_pair(it->idA, it->idB), std::abs(t - t_s)));
    }
    int idA = 0;
    int idB = 1;
    for (int i = 0; i < N; i++) {
        size_t idx_min = 0;
        std::pair<std::pair<int, int>, double> it_min = diff[idx_min];
        for (size_t j = 1; j < diff.size(); j++) {
            std::pair<std::pair<int, int>, double> it = diff[j];
            if (it.first == std::make_pair(idA, idB)) {
                idx_min = it.second < it_min.second ? j : idx_min;
                it_min = diff[idx_min];
            }
        }
        idx[i] = idx_min;
        idA++;
        idB++;
        diff[idx_min].second = std::numeric_limits<double>::max();
    }
}

void SplineFusion::integrateStep(int64_t prevTime, int64_t dt_, const ImuData& imu, Eigen::Matrix3d& Rs, Eigen::Vector3d& Ps, Eigen::Vector3d& Vs)
{
    static bool first_imu = false;
    static Eigen::Vector3d acc_0;
    static Eigen::Vector3d gyr_0;
    Eigen::Vector3d linear_acceleration = imu.accel;
    Eigen::Vector3d angular_velocity = imu.gyro;
    if (!first_imu) {
        first_imu = true;
        acc_0 = linear_acceleration;
        gyr_0 = angular_velocity;
    }
    Eigen::Vector3d g = calib_param.gravity;
    Eigen::Vector3d ba;
    Eigen::Vector3d bg;
    Eigen::Matrix<double, 6, 1> bias = spline_local.itpBias(prevTime);
    ba = bias.head<3>();
    bg = bias.tail<3>();
    double dt = dt_ * NS_TO_S;
    Eigen::Vector3d un_acc_0;
    un_acc_0 = Rs * (acc_0 - ba) - g;
    Eigen::Vector3d un_gyr = 0.5 * (gyr_0 + angular_velocity) - bg;
    Rs *= Quater::deltaQ(un_gyr * dt).toRotationMatrix();
    Eigen::Vector3d un_acc_1 = Rs * (linear_acceleration - ba) - g;
    Eigen::Vector3d un_acc = 0.5 * (un_acc_0 + un_acc_1);
    Ps += dt * Vs + 0.5 * dt * dt * un_acc;
    Vs += dt * un_acc;
    acc_0 = linear_acceleration;
    gyr_0 = angular_velocity;
}

void SplineFusion::integration(const int64_t curTime, Eigen::Quaterniond& qs, Eigen::Vector3d& Ps)
{
    std::vector<ImuData> imu_vec;
    getIMUInterval(last_imu_t_ns, curTime, imu_vec);
    if (!imu_vec.empty()) {
        Eigen::Quaterniond qs0;
        spline_local.itpQuaternion(last_imu_t_ns, &qs0);
        Eigen::Matrix3d Rs0(qs0);
        Eigen::Vector3d Ps0 = spline_local.itpPosition(last_imu_t_ns);
        Eigen::Vector3d Vs0 = spline_local.itpPosition<1>(last_imu_t_ns);
        for (size_t i = 0; i < imu_vec.size(); i++) {
            int64_t dt;
            int64_t t_ns = imu_vec[i].time_ns;
            if (i == 0) {
                dt = t_ns - last_imu_t_ns;
            } else {
                dt = t_ns - imu_vec[i - 1].time_ns;
            }
            integrateStep(last_imu_t_ns, dt, imu_vec[i], Rs0, Ps0, Vs0);
        }
        qs = Eigen::Quaterniond(Rs0);
        Ps = Ps0;
    } else {
        qs = spline_local.extrapolateOrtKnot(1);
        Ps = spline_local.extrapolatePosKnot(1);
    }
}

bool SplineFusion::getIMUInterval(int64_t t0, int64_t t1, std::vector<ImuData>& imu_vec)
{
    if (imu_buff.empty()) {
        printf("No IMU available. \n");
        return false;
    }
    int idx = 0;
    while (imu_buff.at(idx).time_ns <= std::min(imu_buff.back().time_ns, t1)) {
        imu_vec.push_back(imu_buff.at(idx));
        idx++;
        if (idx >= imu_buff.size()) break;
    }
    return true;
}

void SplineFusion::displayControlPoints()
{
    sensor_msgs::msg::PointCloud points_inactive_msg;
    points_inactive_msg.header.frame_id = "map";
    auto now = std::chrono::system_clock::now();
    points_inactive_msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    points_inactive_msg.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch() % std::chrono::seconds(1)).count();
    points_inactive_msg.points.push_back(getPointMsg(spline_local.getIdlePos(0)));
    points_inactive_msg.points.push_back(getPointMsg(spline_local.getIdlePos(1)));
    points_inactive_msg.points.push_back(getPointMsg(spline_local.getIdlePos(2)));
    sensor_msgs::msg::PointCloud points_active_msg;
    points_active_msg.header.frame_id = "map";
    points_active_msg.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    points_active_msg.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch() % std::chrono::seconds(1)).count();
    for (size_t i = 0; i < spline_local.numKnots(); i++) {
        points_active_msg.points.push_back(getPointMsg(spline_local.getKnotPos(i)));
    }
    pub_knots_inactive->publish(points_inactive_msg);
    pub_knots_active->publish(points_active_msg);
}

geometry_msgs::msg::Point32 SplineFusion::getPointMsg(Eigen::Vector3d p)
{
    geometry_msgs::msg::Point32 p_msg;
    Eigen::Vector3d p_U = calib_param.q_nav_uwb * p + calib_param.t_nav_uwb;
    p_msg.x = p_U.x();
    p_msg.y = p_U.y();
    p_msg.z = p_U.z();
    return p_msg;
}
