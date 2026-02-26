/**
 * @file sensor_data.h
 * @brief 传感器数据结构定义
 */
#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sfuise {

/**
 * @brief 位姿数据
 */
struct PoseData {
    int64_t time_ns;
    Eigen::Quaterniond orient;
    Eigen::Vector3d pos;

    PoseData() = default;

    PoseData(int64_t s, const Eigen::Quaterniond& q, const Eigen::Vector3d& t)
        : time_ns(s), orient(q), pos(t) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief IMU 数据
 */
struct ImuData {
    const int64_t time_ns;
    const Eigen::Vector3d gyro;
    const Eigen::Vector3d accel;

    ImuData(const int64_t s, const Eigen::Vector3d& w, const Eigen::Vector3d& a)
        : time_ns(s), gyro(w), accel(a) {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

/**
 * @brief TOA (Time of Arrival) 测距数据
 */
struct TOAData {
    const int64_t time_ns;
    const int anchor_id;
    const double data;

    TOAData(const int64_t s, const int i, const double r)
        : time_ns(s), anchor_id(i), data(r) {}
};

/**
 * @brief TDOA (Time Difference of Arrival) 测距数据
 */
struct TDOAData {
    const int64_t time_ns;
    const int idA;
    const int idB;
    const double data;

    TDOAData(const int64_t s, const int idxA, const int idxB, const double r)
        : time_ns(s), idA(idxA), idB(idxB), data(r) {}
};

}  // namespace sfuise

// 向后兼容：保持全局命名空间中的类型别名
using PoseData = sfuise::PoseData;
using ImuData = sfuise::ImuData;
using TOAData = sfuise::TOAData;
using TDOAData = sfuise::TDOAData;
