#pragma once

namespace SplineFusionTopics
{

constexpr char kTopicImuIn[] = "/EstimationInterface/imu_ds";
constexpr char kTopicAnchorIn[] = "/EstimationInterface/anchor_list_sfuise";
constexpr char kTopicTdoaIn[] = "/EstimationInterface/tdoa_ds";
constexpr char kTopicToaIn[] = "/EstimationInterface/toa_ds";
constexpr char kTopicKnotsActive[] = "/SplineFusion/active_control_points";
constexpr char kTopicKnotsInactive[] = "/SplineFusion/inactive_control_points";
constexpr char kTopicCalibOut[] = "/SplineFusion/sys_calib";
constexpr char kTopicEstimateOut[] = "/SplineFusion/est_window";
constexpr char kTopicStartTimeOut[] = "/SplineFusion/start_time";

} // namespace SplineFusionTopics
