#pragma once

namespace ParamKeys
{

namespace EstimationInterface
{
constexpr char kIfTdoa[] = "if_tdoa";
constexpr char kIfFraunhoferMsg[] = "if_fraunhofer_msg";
constexpr char kTopicImu[] = "topic_imu";
constexpr char kTopicUwb[] = "topic_uwb";
constexpr char kTopicAnchorList[] = "topic_anchor_list";
constexpr char kTopicGroundTruth[] = "topic_ground_truth";
constexpr char kImuSampleCoeff[] = "imu_sample_coeff";
constexpr char kUwbSampleCoeff[] = "uwb_sample_coeff";
constexpr char kImuFrequency[] = "imu_frequency";
constexpr char kUwbFrequency[] = "uwb_frequency";
constexpr char kGyroUnit[] = "gyro_unit";
constexpr char kAccRatio[] = "acc_ratio";
constexpr char kAnchorPath[] = "anchor_path";
constexpr char kControlPointFps[] = "control_point_fps";
} // namespace EstimationInterface

namespace SplineFusion
{
constexpr char kImuSampleCoeff[] = "imu_sample_coeff";
constexpr char kWUwb[] = "w_uwb";
constexpr char kMaxIter[] = "max_iter";
constexpr char kControlPointFps[] = "control_point_fps";
constexpr char kIfTdoa[] = "if_tdoa";
constexpr char kNWindowCalib[] = "n_window_calib";
constexpr char kWindowSize[] = "window_size";
constexpr char kAccelVarInv[] = "accel_var_inv";
constexpr char kBiasAccelVarInv[] = "bias_accel_var_inv";
constexpr char kWAccel[] = "w_accel";
constexpr char kWBiasAccel[] = "w_bias_accel";
constexpr char kGyroVarInv[] = "gyro_var_inv";
constexpr char kBiasGyroVarInv[] = "bias_gyro_var_inv";
constexpr char kWGyro[] = "w_gyro";
constexpr char kWBiasGyro[] = "w_bias_gyro";
constexpr char kIfRejectUwb[] = "if_reject_uwb";
constexpr char kRejectUwbThresh[] = "reject_uwb_thresh";
constexpr char kRejectUwbWindowWidth[] = "reject_uwb_window_width";
constexpr char kOffset[] = "offset";
constexpr char kToaOffset[] = "toa_offset";
} // namespace SplineFusion

} // namespace ParamKeys
