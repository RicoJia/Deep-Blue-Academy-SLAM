//
// Created by xiang on 2022/7/18.
// ./bin/test_ndt_lo --bag_path data/ulhk/test2.bag --stopping_msg_index 1000 --start_visualize_msg_index 100
//

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "ch7/direct_ndt_lo.h"
#include "ch7/ndt_3d.h"
#include "common/dataset_type.h"
#include "common/io_utils.h"
#include "common/timer/timer.h"

/// 本程序以ULHK数据集为例
/// 测试以NDT为主的Lidar Odometry
/// 若使用PCL NDT的话，会重新建立NDT树
DEFINE_string(bag_path, "./data/ulhk/test2.bag", "path to rosbag");
DEFINE_string(dataset_type, "ULHK", "NCLT/ULHK/KITTI/WXB_3D");  // 数据集类型
DEFINE_bool(use_pcl_ndt, false, "use pcl ndt to align?");
DEFINE_bool(use_ndt_nearby_6, false, "use ndt nearby 6?");
DEFINE_bool(display_map, true, "display map?");
DEFINE_int64(stopping_msg_index, 0, "0 means no limit, otherwise stop at this message index");
DEFINE_int64(start_visualize_msg_index, 0, "start visualization from this index");

int main(int argc, char** argv) {
    google::InitGoogleLogging(argv[0]);
    FLAGS_stderrthreshold = google::INFO;
    FLAGS_colorlogtostderr = true;
    google::ParseCommandLineFlags(&argc, &argv, true);

    size_t stopping_msg_index = 0;
    sad::RosbagIO rosbag_io(fLS::FLAGS_bag_path, sad::Str2DatasetType(FLAGS_dataset_type), 
        FLAGS_stopping_msg_index);

    sad::DirectNDTLO::Options options;
    options.use_pcl_ndt_ = fLB::FLAGS_use_pcl_ndt;
    options.ndt3d_options_.nearby_type_ =
        FLAGS_use_ndt_nearby_6 ? sad::Ndt3d::NearbyType::NEARBY6 : sad::Ndt3d::NearbyType::CENTER;
    options.display_realtime_cloud_ = FLAGS_display_map;
    sad::DirectNDTLO ndt_lo(options);

    size_t num_msgs = 0;

    rosbag_io
        .AddAutoPointCloudHandle([&](sensor_msgs::PointCloud2::Ptr msg) -> bool {
            sad::common::Timer::Evaluate(
                [&]() {
                    SE3 pose;
                    auto raw_cloud_ptr = sad::VoxelCloud(sad::PointCloud2ToCloudPtr(msg));
                    //TODO
                    std::cout<<"=================================num_msgs: "<<num_msgs<<", num points: "<<raw_cloud_ptr->points.size()<<std::endl;
                    bool visualize = FLAGS_start_visualize_msg_index < num_msgs;
                    ndt_lo.AddCloud(raw_cloud_ptr , pose, visualize);
                    rosbag_io.incrementMsgNum();
                    num_msgs++;
                },
                "NDT registration");
            return true;
        })
        .Go();

    if (FLAGS_display_map) {
        // 把地图存下来
        ndt_lo.SaveMap("./data/ch7/map.pcd");
    }

    sad::common::Timer::PrintAll();
    LOG(INFO) << "done.";

    return 0;
}
