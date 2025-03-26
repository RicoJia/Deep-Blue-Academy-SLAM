#pragma once 
#include "common/eigen_types.h"
#include "common/message_def.h"

namespace sad {
/// 一个Lidar读数结构
struct Lidar {
    Lidar() = default;

    /// 构造函数，直接传入时间戳和点集
    Lidar(double unix_time, const std::vector<Vec2d>& points)
        : unix_time_(unix_time), points_(points) {}

    double unix_time_ = 0;                // Unix系统时间戳
    std::vector<Vec2d> points_;           // 2D点集合，激光扫描数据
};

/// 新的回调类型，用于处理Lidar读数
using LidarProcessFuncType = std::function<void(const Lidar &)>;
};
