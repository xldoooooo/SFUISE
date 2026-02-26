/**
 * @file parameters.h
 * @brief 优化器参数定义
 */
#pragma once

#include <cstdint>

#include <Eigen/Core>
#include <Eigen/Geometry>

#include "types/eigen_types.h"

namespace sfuise {

/**
 * @brief 样条融合优化器的参数
 */
struct Parameters {
    /// @name 优化标志
    /// @{
    bool if_opt_g;                        ///< 是否优化重力
    bool if_opt_transform;                ///< 是否优化变换
    bool if_reject_uwb;                   ///< 是否开启 UWB 异常值拒绝
    bool if_reject_uwb_in_optimization;   ///< 优化时是否拒绝 UWB 异常值
    /// @}

    /// @name 权重参数
    /// @{
    double w_uwb;           ///< UWB 测量权重
    double w_acc;           ///< 加速度计权重
    double w_gyro;          ///< 陀螺仪权重
    double w_bias_acc;      ///< 加速度计偏置权重
    double w_bias_gyro;     ///< 陀螺仪偏置权重
    /// @}

    /// @name 异常值拒绝参数
    /// @{
    double reject_uwb_thresh;         ///< UWB 异常值阈值
    double reject_uwb_window_width;   ///< UWB 异常值检测窗口宽度
    /// @}

    /// 控制点帧率
    int control_point_fps;

    /// @name 方差逆参数
    /// @{
    Eigen::Vector3d accel_var_inv;        ///< 加速度计方差逆
    Eigen::Vector3d gyro_var_inv;         ///< 陀螺仪方差逆
    Eigen::Vector3d bias_accel_var_inv;   ///< 加速度计偏置方差逆
    Eigen::Vector3d bias_gyro_var_inv;    ///< 陀螺仪偏置方差逆
    Eigen::Vector3d pos_var_inv;          ///< 位置方差逆
    /// @}

    /// @name 初始变换参数
    /// @{
    Eigen::Vector3d t_nav_uwb_init;       ///< 导航系到 UWB 系的初始平移
    Eigen::Quaterniond q_nav_uwb_init;    ///< 导航系到 UWB 系的初始旋转
    /// @}

    /// @name 锚点配置
    /// @{
    Eigen::aligned_map<uint16_t, Eigen::Vector3d> anchor_list;   ///< 锚点位置列表
    Eigen::aligned_map<uint16_t, double> toa_offset;             ///< TOA 偏移
    /// @}

    Parameters() : toa_offset{{static_cast<uint16_t>(0), 0.0}} {}

    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
};

}  // namespace sfuise

// 向后兼容
using Parameters = sfuise::Parameters;
