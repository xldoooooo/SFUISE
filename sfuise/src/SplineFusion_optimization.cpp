#include "SplineFusion.h"

#include <cmath>

void SplineFusion::updateUwbRejectFlag()
{
    if (param.if_reject_uwb) {
        if (solver_flag == INITIAL &&
            spline_local.numKnots() < int(param.reject_uwb_window_width * window_size))
            param.if_reject_uwb_in_optimization = false;
        else
            param.if_reject_uwb_in_optimization = true;
    } else {
        param.if_reject_uwb_in_optimization = false;
    }
}

void SplineFusion::updateCalibrationFlags()
{
    if ((int)window_count >= n_window_calib) {
        param.if_opt_g = false;
        param.if_opt_transform = false;
    }
}

void SplineFusion::updateMeasurementWindows()
{
    if (!if_uwb_only) updateMeasurements(imu_window, imu_buff);
    if (if_tdoa) {
        updateMeasurements(tdoa_window, tdoa_buff);
    } else {
        updateMeasurements(toa_window, toa_buff);
    }
}

bool SplineFusion::shouldFixPose() const
{
    return solver_flag == INITIAL;
}

bool SplineFusion::optimization()
{
    if (spline_local.numKnots() < 2) return false;
    updateUwbRejectFlag();
    updateCalibrationFlags();
    updateMeasurementWindows();
    bool converged = false;
    int opt_iter = 0;
    pose_fixed = shouldFixPose();
    lambda = 1e-6;
    lambda_vee = 2;
    updateLinearizerSize();
    while (!converged && opt_iter < max_iter) {
        converged = optimize(opt_iter);
        opt_iter++;
    }
    return converged;
}

bool SplineFusion::optimize(const int iter)
{
    Linearizer lopt(bias_block_offset, gravity_block_offset,
        hess_size, &spline_local, &calib_param, &param, pose_fixed);
    if (!imu_window.empty()) lopt(imu_window);
    if (!tdoa_window.empty()) lopt(tdoa_window);
    if (!toa_window.empty()) lopt(toa_window);
    if (iter) {
        double gradient_max_norm = lopt.accum.getB().array().abs().maxCoeff();
        if (gradient_max_norm < 1e-8) return true;
    }
    lopt.accum.setup_solver();
    Eigen::VectorXd Hdiag = lopt.accum.Hdiagonal();
    bool stop = false;
    while (!stop) {
        Eigen::VectorXd Hdiag_lambda = Hdiag * lambda;
        for (int i = 0; i < Hdiag_lambda.size(); i++) {
            Hdiag_lambda[i] = std::max(Hdiag_lambda[i], 1e-18);
        }
        Eigen::VectorXd inc_full = -lopt.accum.solve(&Hdiag_lambda);
        Eigen::aligned_deque<Eigen::Vector3d> knots_trans_backup;
        Eigen::aligned_deque<Eigen::Quaterniond> knots_rot_backup;
        Eigen::aligned_deque<Eigen::Matrix<double, 6, 1>> knots_bias_backup;
        spline_local.getAllStateKnots(knots_trans_backup, knots_rot_backup, knots_bias_backup);
        CalibParam calib_param_backup = calib_param;
        applyIncFull(inc_full);
        ComputeErrorSplineOpt eopt(&spline_local, &calib_param, &param);
        if (!toa_window.empty()) eopt(toa_window);
        if (!imu_window.empty()) eopt(imu_window);
        if (!tdoa_window.empty()) eopt(tdoa_window);
        double f_diff = lopt.error - eopt.error;
        double l_diff = 0.5 * inc_full.dot(inc_full * lambda - lopt.accum.getB());
        double step_quality = f_diff / l_diff;
        if (step_quality < 0) {
            lambda = std::min(100.0, lambda_vee * lambda);
            if (std::abs(lambda - 100.0) < 1e-3) {
                stop = true;
            }
            lambda_vee *= 2;
            spline_local.setAllKnots(knots_trans_backup, knots_rot_backup, knots_bias_backup);
            calib_param.setCalibParam(calib_param_backup);
        } else {
            if (inc_full.norm() / ((double)spline_local.numKnots()) < 1e-10 || std::abs(f_diff) / lopt.error < 1e-6) {
                stop = true;
            }
            lambda = std::max(1e-18, lambda * std::max(1.0 / 3, 1 - std::pow(2 * step_quality - 1, 3.0)));
            lambda_vee = 2;
            break;
        }
    }
    return stop;
}

void SplineFusion::updateLinearizerSize()
{
    int num_knots = spline_local.numKnots();
    bias_block_offset = Linearizer::POSE_SIZE * num_knots;
    hess_size = bias_block_offset;
    if (!if_uwb_only) {
        hess_size += Linearizer::ACCEL_BIAS_SIZE * num_knots;
        hess_size += Linearizer::GYRO_BIAS_SIZE * num_knots;
    }
    gravity_block_offset = hess_size;
    hess_size += Linearizer::G_SIZE;
    if (param.if_opt_transform) {
        hess_size += Linearizer::OFFSET_SIZE;
        hess_size += Linearizer::ROTATION_SIZE;
    }
}

void SplineFusion::applyIncFull(Eigen::VectorXd& inc_full)
{
    size_t num_knots = spline_local.numKnots();
    for (size_t i = 0; i < num_knots; i++) {
        Eigen::Matrix<double, 6, 1> inc = inc_full.segment<Linearizer::POSE_SIZE>(Linearizer::POSE_SIZE * i);
        spline_local.applyPoseInc(i, inc);
    }
    spline_local.checkQuaternionControlPoints();
    if (!if_uwb_only) {
        for (size_t i = 0; i < num_knots; i++) {
            Eigen::Matrix<double, 6, 1> inc = inc_full.segment<Linearizer::BIAS_SIZE>(bias_block_offset + Linearizer::BIAS_SIZE * i);
            spline_local.applyBiasInc(i, inc);
        }
        spline_local.updateBiasIdleFirstWindow();
        if (param.if_opt_g) {
            Eigen::VectorXd dg = inc_full.segment<Linearizer::G_SIZE>(gravity_block_offset);
            Eigen::Vector3d g0 = (calib_param.gravity + Sphere::TangentBasis(calib_param.gravity) * dg).normalized() * 9.81;
            calib_param.gravity = g0;
        }
    }
    if (param.if_opt_transform) {
        calib_param.t_nav_uwb += inc_full.segment<Linearizer::OFFSET_SIZE>(gravity_block_offset + Linearizer::TRANS_OFFSET);
        Eigen::Quaterniond q_inc;
        Quater::exp(inc_full.segment<Linearizer::ROTATION_SIZE>(gravity_block_offset + Linearizer::ROTATION_OFFSET), q_inc);
        calib_param.q_nav_uwb *= q_inc;
    }
}
