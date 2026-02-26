#include "EstimationInterface.h"

#include <chrono>
#include <fstream>
#include <sstream>

void EstimationInterface::getAnchorListFromUTIL(const std::string& anchor_path)
{
    std::string line;
    std::ifstream infile;
    infile.open(anchor_path);
    if (!infile) {
        RCLCPP_ERROR(this->get_logger(), "Unable to open anchor file: %s", anchor_path.c_str());
        rclcpp::shutdown();
        return;
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

void EstimationInterface::publishAnchor()
{
    if (!initialized_anchor) {
        return;
    }
    isas_msgs::msg::Anchorlist anchor_list_msg;

    sensor_msgs::msg::PointCloud anchors;
    anchors.header.frame_id = "map";
    auto now = std::chrono::system_clock::now();
    anchors.header.stamp.sec = std::chrono::duration_cast<std::chrono::seconds>(now.time_since_epoch()).count();
    anchors.header.stamp.nanosec = std::chrono::duration_cast<std::chrono::nanoseconds>(
        now.time_since_epoch() % std::chrono::seconds(1)).count();

    for (const auto& [anchor_id, pos] : anchor_list) {
        isas_msgs::msg::AnchorPosition anchor;
        anchor.position.x = pos[0];
        anchor.position.y = pos[1];
        anchor.position.z = pos[2];
        anchor.id = anchor_id;
        anchor_list_msg.anchor.push_back(anchor);

        geometry_msgs::msg::Point32 p;
        p.x = pos[0];
        p.y = pos[1];
        p.z = pos[2];
        anchors.points.push_back(p);
    }

    anchor_pub->publish(anchor_list_msg);
    anchor_pos_pub->publish(anchors);
}
