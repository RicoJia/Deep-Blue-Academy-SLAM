//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "ch6/mapping_2d.h"
#include "common/io_utils.h"
#include "common/lidar2D.h"

#include <cmath>
#include <limits>

DEFINE_string(bag_path, "./dataset/sad/2dmapping/floor1.bag", "数据包路径");
DEFINE_bool(with_loop_closing, false, "是否使用回环检测");

/// 测试2D lidar SLAM

sensor_msgs::LaserScanPtr lidarToLaserScan(const sad::Lidar &lidar) {
    sensor_msgs::LaserScanPtr scan_msg = boost::make_shared<sensor_msgs::LaserScan>();
    scan_msg->header.frame_id = "laser_frame"; // Update frame as needed

    if (lidar.points_.empty()) {
        // No points, so fill with zeros.
        scan_msg->angle_min = 0.0;
        scan_msg->angle_max = 0.0;
        scan_msg->angle_increment = 0.0;
    }

    // Compute angles and ranges for each point.
    std::vector<float> ranges;
    ranges.reserve(lidar.points_.size());

    double min_angle = std::numeric_limits<double>::max();
    double max_angle = -std::numeric_limits<double>::max();

    for (const Vec2d &pt : lidar.points_) {
        double angle = std::atan2(pt.y(), pt.x());
        float r = std::hypot(pt.x(), pt.y());
        ranges.push_back(r);
        if (angle < min_angle) { min_angle = angle; }
        if (angle > max_angle) { max_angle = angle; }
    }

    scan_msg->angle_min = min_angle;
    scan_msg->angle_max = max_angle;
    if (lidar.points_.size() > 1) {
        scan_msg->angle_increment = (max_angle - min_angle) / static_cast<double>(lidar.points_.size() - 1);
    } else {
        scan_msg->angle_increment = 0.0;
    }

    // Set other LaserScan parameters.
    scan_msg->time_increment = 0.0; // Unknown from Lidar data
    scan_msg->scan_time = 0.0;      // Unknown from Lidar data
    scan_msg->range_min = 0.0;      // Set as appropriate for your sensor
    scan_msg->range_max = std::numeric_limits<float>::max(); // Set as appropriate

    // Fill the computed ranges.
    scan_msg->ranges = ranges;
    // Optionally, initialize intensities (here with zeros).
    scan_msg->intensities.resize(ranges.size(), 0.0);

    return scan_msg;
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    // Test Code
    // sad::TxtIO txt_io("./data/2dmapping/scan_file.txt");
    sad::Mapping2D mapping;

    std::system("rm -rf ./data/ch6/*");

    if (mapping.Init(FLAGS_with_loop_closing) == false) {
        return -1;
    }

    rosbag_io.AddScan2DHandle("/pavo_scan_bottom", [&](Scan2d::Ptr scan) { return mapping.ProcessScan(scan); }).Go();
    // txt_io.SetLidarProcessFunc([&](const sad::Lidar& lidar_txt_msg) { 
    //     auto scan = lidarToLaserScan(lidar_txt_msg);
    //     return mapping.ProcessScan(scan); }).Go();
    cv::imwrite("./data/ch6/global_map.png", mapping.ShowGlobalMap(2000));
    return 0;
}