// --bag_path 
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <cmath>
#include <fstream>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/io_utils.h"

// DEFINE_string(bag_path, "./data/2dmapping/floor2.bag", "数据包路径");
DEFINE_string(bag_path, "./data/ulhk/test2.bag", "数据包路径");

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

/**
 *  1. sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path, sad::Str2DatasetType(FLAGS_dataset_type));
        - Stores ULHK
    2. .AddAutoPointCloudHandle([&ndt_lo](sensor_msgs::PointCloud2::Ptr msg) -> bool {
            return AddHandle(GetLidarTopicName(), [f](const rosbag::MessageInstance &m) -> bool {
                auto msg = m.instantiate<sensor_msgs::PointCloud2>();
                if (msg == nullptr) {
                    return false;
                }
                return f(msg);
    3.  std::string RosbagIO::GetLidarTopicName() const {
            // TODO: 1 - get topic
            if (dataset_type_ == DatasetType::ULHK) {
                return ulhk_lidar_topic;
            }
        }
 */

inline void save_ulhk_3d_scan_ros1_2_txt(const sensor_msgs::PointCloud2::Ptr& msg){
    if (!ofs.is_open()) {
        LOG(ERROR) << "File is not open!";
        return;
    }
    // fixed precision only affects floating‐point; we'll switch to hex later
    ofs << std::fixed << std::setprecision(6);

    // 1) header token + literal
    ofs << "ulhk_3d" << ','
        // 2) seq
        << msg->header.seq << ','
        // 3) stamp as sec.nanosec
        << msg->header.stamp.sec << '.'
        << std::setw(9) << std::setfill('0')
        << msg->header.stamp.nsec << ','
        // 4) frame_id, height, width, n_fields
        << msg->header.frame_id << ','
        << msg->height       << ','
        << msg->width        << ','
        << msg->fields.size();

    // 5) each field
    for (auto &f : msg->fields) {
        ofs << ','
            << f.name              << ','
            << f.offset            << ','
            << int(f.datatype)     << ','
            << f.count;
    }

    // 6) metadata flags/strides
    ofs << ','
        << (msg->is_bigendian ? '1' : '0') << ','
        << msg->point_step               << ','
        << msg->row_step                 << ','
        << (msg->is_dense   ? '1' : '0');

    // 7) **raw data bytes** appended as hex
    ofs << std::hex;                    // switch to hex
    for (uint8_t byte : msg->data) {
        ofs << ',' 
            << std::setw(2) << std::setfill('0')
            << int(byte);
    }
    ofs << std::dec                 // back to decimal for any future output
        << std::setfill(' ')       // restore fill
        << '\n'; 
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

    sad::RosbagIO rosbag_io(FLAGS_bag_path, sad::Str2DatasetType("ULHK"));
    Scan2d::Ptr last_scan = nullptr, current_scan = nullptr;

    // Register a callback that processes scan messages.
    rosbag_io
        .AddScan2DHandle("/pavo_scan_bottom",
                         [&](Scan2d::Ptr scan) -> bool {
                             // Directly use the scan data.
                             save_scan_ros1_2_txt(scan);
                             return true;
                         })
        .AddAutoPointCloudHandle(
            [&](sensor_msgs::PointCloud2::Ptr msg) -> bool {
                save_ulhk_3d_scan_ros1_2_txt(msg);
                return true;
            }
        )            
        .Go();

    ofs.close();  // Close the file when done.
    // TODO
    std::cout << "Scan saved to: " << OUTPUT_FILE << std::endl;
    return 0;
}
