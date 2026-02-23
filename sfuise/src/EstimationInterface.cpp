#include <rclcpp/rclcpp.hpp>
#include <fstream>
#include "../include/SplineState.h"
#include "../include/utils/PoseVisualization.h"
#include "sensor_msgs/msg/imu.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "nav_msgs/msg/path.hpp"
#include "cf_msgs/msg/tdoa.hpp"
#include "isas_msgs/msg/rtls_range.hpp"
#include "isas_msgs/msg/rtls_stick.hpp"
#include "isas_msgs/msg/anchor_position.hpp"
#include "isas_msgs/msg/anchorlist.hpp"
#include "sensor_msgs/msg/point_cloud.hpp"
#include "sfuise_msgs/msg/spline.hpp"
#include "sfuise_msgs/msg/estimate.hpp"
#include "sfuise_msgs/msg/calib.hpp"
#include "std_msgs/msg/int64.hpp"
#include "visualization_msgs/msg/marker.hpp"

class EstimationInterface : public rclcpp::Node
{

  public:

    EstimationInterface() : rclcpp::Node("EstimationInterface")
    {
        initialized_anchor = false;
        if_nav_uwb = false;
        readParamsInterface();
        sub_start = this->create_subscription<std_msgs::msg::Int64>(
            "/SplineFusion/start_time", 1000, 
            std::bind(&EstimationInterface::startCallBack, this, std::placeholders::_1));
        
        std::string imu_type = this->declare_parameter<std::string>("topic_imu", "");
        sub_imu = this->create_subscription<sensor_msgs::msg::Imu>(
            imu_type, 400, 
            std::bind(&EstimationInterface::getImuCallback, this, std::placeholders::_1));
        pub_imu = this->create_publisher<sensor_msgs::msg::Imu>("imu_ds", 400);
        
        std::string uwb_type = this->declare_parameter<std::string>("topic_uwb", "");
        if (!uwb_type.compare("/tdoa_data")) {
            sub_uwb = this->create_subscription<cf_msgs::msg::Tdoa>(
                uwb_type, 400, 
                std::bind(&EstimationInterface::getTdoaUTILCallback, this, std::placeholders::_1));
            pub_uwb_tdoa = this->create_publisher<cf_msgs::msg::Tdoa>("tdoa_ds", 400);
        } else if (!uwb_type.compare("/rtls_flares")) {
            sub_uwb = this->create_subscription<isas_msgs::msg::RTLSStick>(
                uwb_type, 400, 
                std::bind(&EstimationInterface::getToaISASCallback, this, std::placeholders::_1));
            pub_uwb_toa = this->create_publisher<isas_msgs::msg::RTLSStick>("toa_ds", 400);
        } else {
            RCLCPP_ERROR(this->get_logger(), "Wrong UWB format!");
        }
        
        std::string anchor_type = this->declare_parameter<std::string>("topic_anchor_list", "");
        if (!if_tdoa) {
            if (!anchor_type.compare("/anchor_list")) {
                sub_anchor = this->create_subscription<isas_msgs::msg::Anchorlist>(
                    anchor_type, 400, 
                    std::bind(&EstimationInterface::getAnchorListFromISASCallback, this, std::placeholders::_1));
            } else {
                RCLCPP_ERROR(this->get_logger(), "Anchor list not available!");
            }
        }
        
        anchor_pos_pub = this->create_publisher<sensor_msgs::msg::PointCloud>("visualization_anchor", 1000);
        anchor_pub = this->create_publisher<isas_msgs::msg::Anchorlist>("anchor_list", 1000);
        
        timer_anchor = this->create_wall_timer(
            std::chrono::milliseconds(10),
            std::bind(&EstimationInterface::publishAnchor, this));
        
        std::string gt_type = this->declare_parameter<std::string>("topic_ground_truth", "");
        if (!gt_type.compare("/vive/transform/tracker_1_ref")) {
            sub_gt = this->create_subscription<geometry_msgs::msg::TransformStamped>(
                gt_type, 1000, 
                std::bind(&EstimationInterface::getGtFromISASCallback, this, std::placeholders::_1));
        } else if (!gt_type.compare("/pose_data")) {
            sub_gt = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                gt_type, 1000, 
                std::bind(&EstimationInterface::getGtFromUTILCallback, this, std::placeholders::_1));
        }
        
        int control_point_fps = this->declare_parameter<int>("control_point_fps", 100);
        dt_ns = 1e9 / control_point_fps;
        
        sub_calib = this->create_subscription<sfuise_msgs::msg::Calib>(
            "/SplineFusion/sys_calib", 100, 
            std::bind(&EstimationInterface::getCalibCallback, this, std::placeholders::_1));
        
        sub_est = this->create_subscription<sfuise_msgs::msg::Estimate>(
            "/SplineFusion/est_window", 100, 
            std::bind(&EstimationInterface::getEstCallback, this, std::placeholders::_1));
        
        pub_opt_old = this->create_publisher<nav_msgs::msg::Path>("bspline_optimization_old", 1000);
        pub_opt_window = this->create_publisher<nav_msgs::msg::Path>("bspline_optimization_window", 1000);
        opt_old_path.header.frame_id = "map";
        pub_opt_pose = this->create_publisher<visualization_msgs::msg::Marker>("opt_pose", 1000);
        opt_pose_vis.setColor(85.0/255.0,164.0/255.0,104.0/255.0);
    }

private:

    int64_t dt_ns;
    bool initialized_anchor;
    bool if_tdoa;
    bool if_nav_uwb;
    CalibParam calib_param;
    Eigen::aligned_vector<PoseData> gt;
    Eigen::aligned_map<uint16_t, Eigen::Vector3d> anchor_list;
    double imu_sample_coeff;
    double uwb_sample_coeff;
    double imu_frequency;
    double uwb_frequency;
    double average_runtime;
    bool gyro_unit;
    bool acc_ratio;
    SplineState spline_global;
    Eigen::aligned_vector<PoseData> opt_old;
    Eigen::aligned_vector<PoseData> opt_window;
    
    rclcpp::TimerBase::SharedPtr timer_anchor;
    rclcpp::Subscription<std_msgs::msg::Int64>::SharedPtr sub_start;
    rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu;
    rclcpp::SubscriptionBase::SharedPtr sub_uwb;  // 支持多种 UWB 消息类型
    rclcpp::Subscription<isas_msgs::msg::Anchorlist>::SharedPtr sub_anchor;
    rclcpp::SubscriptionBase::SharedPtr sub_gt;  // 支持多种 GT 消息类型
    rclcpp::Subscription<sfuise_msgs::msg::Calib>::SharedPtr sub_calib;
    rclcpp::Subscription<sfuise_msgs::msg::Estimate>::SharedPtr sub_est;
    
    rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr pub_imu;
    rclcpp::Publisher<cf_msgs::msg::Tdoa>::SharedPtr pub_uwb_tdoa;
    rclcpp::Publisher<isas_msgs::msg::RTLSStick>::SharedPtr pub_uwb_toa;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud>::SharedPtr anchor_pos_pub;
    rclcpp::Publisher<isas_msgs::msg::Anchorlist>::SharedPtr anchor_pub;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_opt_old;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr pub_opt_window;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr pub_opt_pose;
    
    nav_msgs::msg::Path opt_old_path;
    PoseVisualization opt_pose_vis;

    void readParamsInterface()
    {
        if_tdoa = this->declare_parameter<bool>("if_tdoa", false);
        imu_sample_coeff = this->declare_parameter<double>("imu_sample_coeff", 1.0);
        uwb_sample_coeff = this->declare_parameter<double>("uwb_sample_coeff", 1.0);
        imu_frequency = this->declare_parameter<double>("imu_frequency", 100.0);
        uwb_frequency = this->declare_parameter<double>("uwb_frequency", 100.0);
        gyro_unit = this->declare_parameter<bool>("gyro_unit", false);
        acc_ratio = this->declare_parameter<bool>("acc_ratio", false);
        if (uwb_sample_coeff == 0) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'uwb_sample_coeff' cannot be 0!");
            rclcpp::shutdown();
        }
        if (if_tdoa) {
            std::string anchor_path = this->declare_parameter<std::string>("anchor_path", "");
            getAnchorListFromUTIL(anchor_path);
        }
    }

    void getEstCallback(const sfuise_msgs::msg::Estimate::SharedPtr est_msg)
    {
        sfuise_msgs::msg::Spline spline_msg = est_msg->spline;
        SplineState spline_w;
        spline_w.init(spline_msg.dt, 0, spline_msg.start_t, spline_msg.start_idx);
        for(const auto knot : spline_msg.knots) {
            Eigen::Vector3d pos(knot.position.x, knot.position.y, knot.position.z);
            Eigen::Quaterniond quat(knot.orientation.w, knot.orientation.x, knot.orientation.y, knot.orientation.z);
            Eigen::Matrix<double, 6, 1> bias;
            bias << knot.bias_acc.x, knot.bias_acc.y, knot.bias_acc.z,
                    knot.bias_gyro.x, knot.bias_gyro.y, knot.bias_gyro.z;
            spline_w.addOneStateKnot(quat, pos, bias);
        }
        for (int i = 0; i < 3; i++) {
            sfuise_msgs::msg::Knot idle = spline_msg.idles[i];
            Eigen::Vector3d t_idle(idle.position.x, idle.position.y, idle.position.z);
            Eigen::Quaterniond q_idle(idle.orientation.w, idle.orientation.x, idle.orientation.y, idle.orientation.z);
            Eigen::Matrix<double, 6, 1> b_idle;
            b_idle << idle.bias_acc.x, idle.bias_acc.y, idle.bias_acc.z, idle.bias_gyro.x, idle.bias_gyro.y, idle.bias_gyro.z;
            spline_w.setIdles(i, t_idle, q_idle, b_idle);
        }
        spline_global.updateKnots(&spline_w);
        if (if_nav_uwb) pubOpt(spline_w, !est_msg->if_full_window.data);
        average_runtime = est_msg->runtime.data;
    }

    void pubOpt(SplineState& spline_local, const bool if_window_full)
    {
        int64_t min_t = spline_local.minTimeNs();
        int64_t max_t = spline_local.maxTimeNs();
        static int cnt = 0;
        if (!if_window_full) {
            for (auto v: opt_window) {
                if (v.time_ns < min_t) {
                    opt_old.push_back(v);
                    opt_old_path.poses.push_back(CommonUtils::pose2msg(v.time_ns, v.pos, v.orient));
                }
            }
        } else {
            cnt = 0;
        }
        opt_window.clear();
        nav_msgs::msg::Path opt_window_path;
        opt_window_path.header.frame_id = "map";
        for (size_t i = cnt; i < gt.size(); i++) {
            int64_t t_ns = gt.at(i).time_ns;
            if (t_ns < min_t) {
                cnt = i;
                continue;
            } else if (t_ns > max_t) {
                break;
            }
            PoseData pose_tf = getPoseInUWB(spline_local, t_ns);
            opt_window.push_back(pose_tf);
            opt_window_path.poses.push_back(CommonUtils::pose2msg(t_ns, pose_tf.pos, pose_tf.orient));
        }
        pub_opt_old->publish(opt_old_path);
        pub_opt_window->publish(opt_window_path);
        opt_pose_vis.pubPose(opt_window.back().pos,opt_window.back().orient,pub_opt_pose,opt_window_path.header);
     }

    void getImuCallback(const sensor_msgs::msg::Imu::SharedPtr imu_msg)
    {
        static int64_t last_imu = 0;
        int64_t t_ns = imu_msg->header.stamp.sec * (int64_t)1e9 + imu_msg->header.stamp.nanosec;
        if (sampleData(t_ns, last_imu, imu_sample_coeff, imu_frequency)) {
            Eigen::Vector3d acc(imu_msg->linear_acceleration.x, imu_msg->linear_acceleration.y, imu_msg->linear_acceleration.z);
            if (acc_ratio) acc *= 9.81;
            Eigen::Vector3d gyro(imu_msg->angular_velocity.x, imu_msg->angular_velocity.y, imu_msg->angular_velocity.z);
            if (gyro_unit) gyro *= M_PI / 180.0;
            last_imu = t_ns;
            sensor_msgs::msg::Imu imu_ds_msg;
            imu_ds_msg.header = imu_msg->header;
            imu_ds_msg.linear_acceleration.x = acc[0];
            imu_ds_msg.linear_acceleration.y = acc[1];
            imu_ds_msg.linear_acceleration.z = acc[2];
            imu_ds_msg.angular_velocity.x = gyro[0];
            imu_ds_msg.angular_velocity.y = gyro[1];
            imu_ds_msg.angular_velocity.z = gyro[2];
            pub_imu->publish(imu_ds_msg);
        }
    }

    void getCalibCallback(const sfuise_msgs::msg::Calib::SharedPtr calib_msg)
    {
        if_nav_uwb = true;
        calib_param.q_nav_uwb = Eigen::Quaterniond(calib_msg->q_nav_uwb.w, calib_msg->q_nav_uwb.x, calib_msg->q_nav_uwb.y, calib_msg->q_nav_uwb.z);
        calib_param.t_nav_uwb = Eigen::Vector3d(calib_msg->t_nav_uwb.x, calib_msg->t_nav_uwb.y, calib_msg->t_nav_uwb.z);
        calib_param.gravity = Eigen::Vector3d(calib_msg->gravity.x, calib_msg->gravity.y, calib_msg->gravity.z);
        calib_param.offset = Eigen::Vector3d(calib_msg->t_tag_body_set.x, calib_msg->t_tag_body_set.y, calib_msg->t_tag_body_set.z);
    }

    void getToaISASCallback(const isas_msgs::msg::RTLSStick::SharedPtr uwb_msg)
    {
        static int64_t last_uwb = 0;
        int64_t t_ns = uwb_msg->header.stamp.sec * (int64_t)1e9 + uwb_msg->header.stamp.nanosec;
        if (sampleData(t_ns, last_uwb, uwb_sample_coeff, uwb_frequency)) {
            for (const auto& rg : uwb_msg->ranges) {
                if (rg.ra == 0) continue;
            }
            pub_uwb_toa->publish(*uwb_msg);
            last_uwb = t_ns;
        }
    }

    void getTdoaUTILCallback(const cf_msgs::msg::Tdoa::SharedPtr msg)
    {
        static int64_t last_uwb = 0;
        int64_t t_ns = msg->header.stamp.sec * (int64_t)1e9 + msg->header.stamp.nanosec;
        if (sampleData(t_ns, last_uwb, uwb_sample_coeff, uwb_frequency)) {
            int idA = msg->id_a;
            int idB = msg->id_b;
            pub_uwb_tdoa->publish(*msg);
            last_uwb = t_ns;
        }
    }

    void getGtFromISASCallback(const geometry_msgs::msg::TransformStamped::SharedPtr gt_msg)
    {
        Eigen::Quaterniond q(gt_msg->transform.rotation.w, gt_msg->transform.rotation.x,
                             gt_msg->transform.rotation.y, gt_msg->transform.rotation.z);
        Eigen::Vector3d pos(gt_msg->transform.translation.x, gt_msg->transform.translation.y, gt_msg->transform.translation.z);
        int64_t t_ns = gt_msg->header.stamp.sec * (int64_t)1e9 + gt_msg->header.stamp.nanosec;
        PoseData pose(t_ns, q, pos);
        gt.push_back(pose);
    }

    void getGtFromUTILCallback(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr gt_msg)
    {
        Eigen::Quaterniond q(gt_msg->pose.pose.orientation.w, gt_msg->pose.pose.orientation.x,
                             gt_msg->pose.pose.orientation.y, gt_msg->pose.pose.orientation.z);
        Eigen::Vector3d pos(gt_msg->pose.pose.position.x, gt_msg->pose.pose.position.y, gt_msg->pose.pose.position.z);
        int64_t t_ns = gt_msg->header.stamp.sec * (int64_t)1e9 + gt_msg->header.stamp.nanosec;
        PoseData pose(t_ns, q, pos);
        gt.push_back(pose);
    }

    void getAnchorListFromISASCallback(const isas_msgs::msg::Anchorlist::SharedPtr anchor_msg)
    {
        if (initialized_anchor) return;
        int num_sum = 20;
        static int cnt = 0;
        for (const auto& anchor : anchor_msg->anchor) {
            Eigen::Vector3d anchor_pos(anchor.position.x, anchor.position.y, anchor.position.z);
            uint16_t anchor_id = anchor.id;
            if (cnt == 0) {
                anchor_list[anchor_id] = anchor_pos;
            } else {
                Eigen::Vector3d ave_pos = anchor_list[anchor_id];
                anchor_list[anchor_id] = (ave_pos * cnt + anchor_pos) / (cnt + 1);
                anchor_pos = anchor_list[anchor_id];
            }
        }
        cnt++;
        if (cnt >= num_sum) {
            initialized_anchor = true;
            publishAnchor();
        }
    }

    void getAnchorListFromUTIL(const std::string& anchor_path)
    {
        std::string line;
        std::ifstream infile;
        infile.open(anchor_path);
        if (!infile) {
            std::cerr << "Unable to open file: " << anchor_path << std::endl;
            exit(1);
        }
        while (std::getline(infile, line)) {
            std::istringstream iss(line);
            char comma, tmp, tmp2;
            int anchor_id;
            double x, y, z;
            iss >> tmp >> tmp >> anchor_id >> tmp >> tmp2 >> comma >> x >> comma >> y >> comma >> z;
            if (tmp2 == 'p') {
                anchor_list[anchor_id] = Eigen::Vector3d(x, y, z);
            }
        }
        infile.close();
        initialized_anchor = true;
    }

    bool sampleData(const int64_t t_ns, const int64_t last_t_ns, const double coeff, const double frequency) const
    {
        if (coeff == 0)  return false;
        int64_t dt = 1e9 / (coeff * frequency);
        if (coeff == 1) {
            return true;
        } else if (t_ns - last_t_ns > dt - 1e5) {
            return true;
        } else {
            return false;
        }
    }

    void publishAnchor()
    {
        if (!initialized_anchor) {
            return;
        } else {
            isas_msgs::msg::Anchorlist anchor_list_msg;
            for (auto it = anchor_list.begin(); it != anchor_list.end(); it++) {
                isas_msgs::msg::AnchorPosition anchor;
                Eigen::Vector3d pos = it->second;
                anchor.position.x = pos[0];
                anchor.position.y = pos[1];
                anchor.position.z = pos[2];
                anchor.id = it->first;
                anchor_list_msg.anchor.push_back(anchor);
            }
            anchor_pub->publish(anchor_list_msg);
            sensor_msgs::msg::PointCloud anchors;
            anchors.header.frame_id = "map";
            auto now = std::chrono::system_clock::now();
            anchors.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
            anchors.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
                now.time_since_epoch() % std::chrono::seconds(1)).count();
            for (auto it = anchor_list.begin(); it != anchor_list.end(); it++) {
                Eigen::Matrix<double, 3, 1> pos = it->second;
                geometry_msgs::msg::Point32 p;
                p.x = pos[0];
                p.y = pos[1];
                p.z = pos[2];
                anchors.points.push_back(p);
            }
            anchor_pos_pub->publish(anchors);
        }
    }

    void startCallBack(const std_msgs::msg::Int64::SharedPtr start_time_msg)
    {
        int64_t bag_start_time = start_time_msg->data;
        spline_global.init(dt_ns, 0, bag_start_time);
    }

    PoseData getPoseInUWB(SplineState& spline, int64_t t_ns)
    {
        Eigen::Quaterniond orient_interp;
        Eigen::Vector3d t_interp = spline.itpPosition(t_ns);
        spline.itpQuaternion(t_ns, &orient_interp);
        Eigen::Quaterniond q = calib_param.q_nav_uwb * orient_interp;
        Eigen::Vector3d t = calib_param.q_nav_uwb * (orient_interp * calib_param.offset + t_interp) + calib_param.t_nav_uwb;
        return PoseData(t_ns, q, t);
    }
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<EstimationInterface>();
    RCLCPP_INFO(node->get_logger(), "\033[1;32m---->\033[0m Starting EstimationInterface.");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
