#pragma once

namespace EstimationInterfaceTopics
{

constexpr char kTopicStartTime[] = "/SplineFusion/start_time";
constexpr char kTopicCalib[] = "/SplineFusion/sys_calib";
constexpr char kTopicEstimate[] = "/SplineFusion/est_window";
constexpr char kTopicAnchorVis[] = "/EstimationInterface/visualization_anchor";
constexpr char kTopicAnchorListOut[] = "/EstimationInterface/anchor_list_sfuise";
constexpr char kTopicOptOld[] = "/EstimationInterface/bspline_optimization_old";
constexpr char kTopicOptWindow[] = "/EstimationInterface/bspline_optimization_window";
constexpr char kTopicOptPose[] = "/EstimationInterface/opt_pose";

constexpr char kTopicParamUwbTdoa[] = "/tdoa_data";
constexpr char kTopicParamUwbToa[] = "/rtls_flares";
constexpr char kTopicParamAnchorList[] = "/anchor_list";
constexpr char kTopicParamGtIsas[] = "/vive/transform/tracker_1_ref";
constexpr char kTopicParamGtUtil[] = "/pose_data";
constexpr char kTopicImuOut[] = "/EstimationInterface/imu_ds";
constexpr char kTopicUwbTdoaOut[] = "/EstimationInterface/tdoa_ds";
constexpr char kTopicUwbToaOut[] = "/EstimationInterface/toa_ds";

} // namespace EstimationInterfaceTopics
