/**
 * @file calib_param.h
 * @brief 标定参数类定义
 */
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

namespace sfuise {

/**
 * @brief 标定参数
 */
class CalibParam {
public:
    /// 标签偏移量 (body 系到 tag 的平移)
    Eigen::Vector3d offset;

    /// 导航系到 UWB 系的旋转
    Eigen::Quaterniond q_nav_uwb;

    /// 导航系到 UWB 系的平移
    Eigen::Vector3d t_nav_uwb;

    /// 重力向量
    Eigen::Vector3d gravity;

    CalibParam() = default;

    /**
     * @brief 从另一个 CalibParam 对象复制参数
     */
    void setCalibParam(const CalibParam& other)
    {
        offset = other.offset;
        q_nav_uwb = other.q_nav_uwb;
        t_nav_uwb = other.t_nav_uwb;
        gravity = other.gravity;
    }

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace sfuise

// 向后兼容
using CalibParam = sfuise::CalibParam;
