#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <cmath>
#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/io_utils.h"

DEFINE_string(bag_path, "./data/2dmapping/floor2.bag", "数据包路径");

std::ofstream ofs;

// Callback function for saving LaserScan messages to a text file with original data.
inline void save_scan_ros1_2_txt(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
    // Convert ROS time to seconds.
    double time_sec = scan_msg->header.stamp.sec + scan_msg->header.stamp.nsec * 1e-9;

    // Number of scan points.
    size_t num_points = scan_msg->ranges.size();

    // Write header: "LIDAR", timestamp, angle_min, angle_increment, range_min, range_max, number of points.
    ofs << "LIDAR " << time_sec << " " << scan_msg->angle_min << " " << scan_msg->angle_increment << " "
        << scan_msg->range_min << " " << scan_msg->range_max << " " << num_points;

    // Write all range values.
    for (size_t i = 0; i < num_points; ++i) {
        ofs << " " << scan_msg->ranges[i];
    }
    ofs << "\n";
}

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    std::string OUTPUT_FILE = "ros1_scan_data.txt";

    ofs.open(OUTPUT_FILE);  // Open file.
    if (!ofs.is_open()) {
        LOG(ERROR) << "Failed to open ros1_scan_data.txt";
        return -1;
    }

    sad::RosbagIO rosbag_io(FLAGS_bag_path);
    Scan2d::Ptr last_scan = nullptr, current_scan = nullptr;

    // Register a callback that processes scan messages.
    rosbag_io
        .AddScan2DHandle("/pavo_scan_bottom",
                         [&](Scan2d::Ptr scan) -> bool {
                             // Directly use the scan data.
                             save_scan_ros1_2_txt(scan);
                             return true;
                         })
        .Go();

    ofs.close();  // Close the file when done.
    // TODO
    std::cout << "Scan saved to: " << OUTPUT_FILE << std::endl;
    return 0;
}
