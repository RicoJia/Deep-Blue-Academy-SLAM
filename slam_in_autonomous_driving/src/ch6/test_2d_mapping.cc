//
// Created by xiang on 2022/3/15.
//
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/highgui.hpp>

#include "ch6/lidar_2d_utils.h"
#include "ch6/mapping_2d.h"
#include "common/io_utils.h"

#include <cmath>
#include <limits>

DEFINE_string(bag_path, "data/2dmapping/floor2.bag", "Bag path");
DEFINE_bool(with_loop_closing, false, "Whether to use loop closure");

// int FLAGS_num_frames_skip=500;
DEFINE_int64(num_frames_skip, 0, "Number of frames to skip");

/// 测试2D lidar SLAM

    sensor_msgs::LaserScanPtr convertScanFromTxt(std::stringstream &ss) {
        // Read the scan parameters.
        double time, angle_min, angle_increment, range_min, range_max;
        int num_points;
        ss >> time >> angle_min >> angle_increment >> range_min >> range_max >> num_points;
        
        // Create a new LaserScan message.
        sensor_msgs::LaserScanPtr scan_msg = boost::make_shared<sensor_msgs::LaserScan>();
        
        // Convert the timestamp.
        uint32_t sec = static_cast<uint32_t>(time);
        uint32_t nsec = static_cast<uint32_t>((time - sec) * 1e9);
        scan_msg->header.stamp.sec = sec;
        scan_msg->header.stamp.nsec = nsec;
        scan_msg->header.frame_id = "laser_frame";  // Adjust the frame as needed.
        
        // Populate the LaserScan parameters.
        scan_msg->angle_min = angle_min;
        scan_msg->angle_increment = angle_increment;
        scan_msg->range_min = range_min;
        scan_msg->range_max = range_max;
        scan_msg->angle_max = (num_points > 0) ? (angle_min + angle_increment * (num_points - 1)) : angle_min;
        scan_msg->time_increment = 0.0;  // Unknown from raw data.
        scan_msg->scan_time = 0.0;       // Unknown from raw data.
        
        // Resize and fill in the ranges.
        scan_msg->ranges.resize(num_points);
        for (int i = 0; i < num_points; ++i) {
            ss >> scan_msg->ranges[i];
        }
        
        // Optionally, initialize intensities (here set to zeros).
        scan_msg->intensities.resize(num_points, 0.0f);
        
        return scan_msg;
    }

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path);
    // Test Code
    // sad::TxtIO txt_io("./data/2dmapping/loop2_scan_file.txt");

    std::system("rm -rf ./data/ch6/*");
    sad::Mapping2D mapping;
    //TODO
    std::cout<<"loop closing: "<<FLAGS_with_loop_closing<<std::endl;

    if (mapping.Init(FLAGS_with_loop_closing) == false) {
        return -1;
    }
    int frame_num = -1;

    rosbag_io.AddScan2DHandle("/pavo_scan_bottom", [&](Scan2d::Ptr scan) { 
        frame_num ++;
        //TODO
        std::cout<<"============================= frame: "<<frame_num<<std::endl;
        bool visualize_this_scan = true;
        if (0 < frame_num && frame_num < FLAGS_num_frames_skip) visualize_this_scan = false;
        return mapping.ProcessScan(scan, visualize_this_scan); }).Go();
    // txt_io.SetLidarProcessFunc([&](std::stringstream& ss) { 
    //     auto scan = convertScanFromTxt(ss);
    //     return mapping.ProcessScan(scan); }).Go();
    cv::imwrite("./data/ch6/global_map.png", mapping.ShowGlobalMap(2000));
    return 0;
}