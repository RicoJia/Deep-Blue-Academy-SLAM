//
// Created by xiang on 2022/3/15.
//

#include "ch6/icp_2d.h"
#include "common/math_utils.h"

#include <glog/logging.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/search/impl/kdtree.hpp>

#include "ch6/g2o_types.h"
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/robust_kernel.h>
#include <g2o/core/robust_kernel_impl.h>
#include <g2o/core/sparse_optimizer.h>
#include <g2o/solvers/cholmod/linear_solver_cholmod.h>
#include <g2o/solvers/dense/linear_solver_dense.h>

namespace sad {

bool Icp2d::AlignGaussNewton(SE2& init_pose) {
    int iterations = 10;
    double cost = 0, lastCost = 0;
    SE2 current_pose = init_pose;
    const float max_dis2 = 0.01;    // 最近邻时的最远距离（平方）
    const int min_effect_pts = 20;  // 最小有效点数

    for (int iter = 0; iter < iterations; ++iter) {
        Mat3d H = Mat3d::Zero();
        Vec3d b = Vec3d::Zero();
        cost = 0;

        int effective_num = 0;  // 有效点数

        // 遍历source
        for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
            float r = source_scan_->ranges[i];
            if (r < source_scan_->range_min || r > source_scan_->range_max) {
                continue;
            }

            float angle = source_scan_->angle_min + i * source_scan_->angle_increment;
            float theta = current_pose.so2().log();
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();

            // 最近邻
            std::vector<int> nn_idx;
            std::vector<float> dis;
            kdtree_.nearestKSearch(pt, 1, nn_idx, dis);

            if (nn_idx.size() > 0 && dis[0] < max_dis2) {
                effective_num++;
                Mat32d J;
                J << 1, 0, 0, 1, -r * std::sin(angle + theta), r * std::cos(angle + theta);
                H += J * J.transpose();

                Vec2d e(pt.x - target_cloud_->points[nn_idx[0]].x, pt.y - target_cloud_->points[nn_idx[0]].y);
                b += -J * e;

                cost += e.dot(e);
            }
        }

        if (effective_num < min_effect_pts) {
            return false;
        }

        // solve for dx
        Vec3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            break;
        }

        cost /= effective_num;
        if (iter > 0 && cost >= lastCost) {
            break;
        }

        LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;

        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
        lastCost = cost;
    }

    init_pose = current_pose;
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose()
              << ", theta: " << current_pose.so2().log();

    return true;
}

bool Icp2d::AlignGaussNewtonPoint2Plane(SE2& init_pose) {
    int iterations = 10;
    double cost = 0, lastCost = 0;
    SE2 current_pose = init_pose;
    const float max_dis = 0.3;      // 最近邻时的最远距离
    const int min_effect_pts = 20;  // 最小有效点数

    for (int iter = 0; iter < iterations; ++iter) {
        Mat3d H = Mat3d::Zero();
        Vec3d b = Vec3d::Zero();
        cost = 0;

        int effective_num = 0;  // 有效点数

        // 遍历source
        for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
            float r = source_scan_->ranges[i];
            if (r < source_scan_->range_min || r > source_scan_->range_max) {
                continue;
            }

            float angle = source_scan_->angle_min + i * source_scan_->angle_increment;
            float theta = current_pose.so2().log();
            Vec2d pw = current_pose * Vec2d(r * std::cos(angle), r * std::sin(angle));
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();

            // 查找5个最近邻
            std::vector<int> nn_idx;
            std::vector<float> dis;
            kdtree_.nearestKSearch(pt, 5, nn_idx, dis);

            std::vector<Vec2d> effective_pts;  // 有效点
            for (int j = 0; j < nn_idx.size(); ++j) {
                if (dis[j] < max_dis) {
                    effective_pts.emplace_back(
                        Vec2d(target_cloud_->points[nn_idx[j]].x, target_cloud_->points[nn_idx[j]].y));
                }
            }

            if (effective_pts.size() < 3) {
                continue;
            }

            // 拟合直线，组装J、H和误差
            Vec3d line_coeffs;
            if (math::FitLine2D(effective_pts, line_coeffs)) {
                effective_num++;
                Vec3d J;
                J << line_coeffs[0], line_coeffs[1],
                    -line_coeffs[0] * r * std::sin(angle + theta) + line_coeffs[1] * r * std::cos(angle + theta);
                H += J * J.transpose();

                double e = line_coeffs[0] * pw[0] + line_coeffs[1] * pw[1] + line_coeffs[2];
                b += -J * e;

                cost += e * e;
            }
        }

        if (effective_num < min_effect_pts) {
            return false;
        }

        // solve for dx
        Vec3d dx = H.ldlt().solve(b);
        if (isnan(dx[0])) {
            break;
        }

        cost /= effective_num;
        if (iter > 0 && cost >= lastCost) {
            break;
        }

        LOG(INFO) << "iter " << iter << " cost = " << cost << ", effect num: " << effective_num;

        current_pose.translation() += dx.head<2>();
        current_pose.so2() = current_pose.so2() * SO2::exp(dx[2]);
        lastCost = cost;
    }

    init_pose = current_pose;
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose()
              << ", theta: " << current_pose.so2().log();

    return true;
}

bool Icp2d::AlignG2OP2L(SE2& init_pose) {
    int iterations = 10;        // 迭代次数
    double rk_delta = 0.8;
    float max_dis = 0.3;       // 最近邻时的最远距离（平方）
    int min_effect_pts = 20;    // 最小有效点数
    
    SE2 current_pose = init_pose;   // 当前位姿
    for (int iter = 0; iter < iterations; ++iter) {
        using BlockSolverType = g2o::BlockSolver<g2o::BlockSolverTraits<3, 1>>;
        using LinearSolverType = g2o::LinearSolverCholmod<BlockSolverType::PoseMatrixType>;
        auto* solver = new g2o::OptimizationAlgorithmLevenberg(g2o::make_unique<BlockSolverType>(g2o::make_unique<LinearSolverType>()));
        g2o::SparseOptimizer optimizer;
        optimizer.setAlgorithm(solver);

        auto* v = new VertexSE2();      // 新建SE2位姿顶点
        v->setId(0);                    // 设置顶点的id
        v->setEstimate(current_pose);   // 设置顶点的估计值为初始位姿
        optimizer.addVertex(v);         // 将顶点添加到优化器中
        int effective_num = 0;  // 有效点数
        // 遍历源始点云
        for (size_t i = 0; i < source_scan_->ranges.size(); ++i) {
            double range = source_scan_->ranges[i]; // 源始点云的距离
            // 判断每个点的距离是否越界
            if (range < source_scan_->range_min || range > source_scan_->range_max) 
                continue;

            // 当前激光点的角度
            double angle = source_scan_->angle_min + i * source_scan_->angle_increment;
            // 从上一次迭代得到的位姿 T_wb 的2x2旋转矩阵中，利用对数映射获取对应的旋转角度
            double theta = current_pose.so2().log();
            // 机器人坐标系下的极坐标转换为笛卡尔坐标，并转为世界坐标系下的坐标 p_i^W，
            Vec2d pw = current_pose * Vec2d(range * std::cos(angle), range * std::sin(angle));
            Point2d pt;
            pt.x = pw.x();
            pt.y = pw.y();

            // 查找5个最近邻
            std::vector<int> nn_idx;    // 最近邻的索引
            std::vector<float> dis;     // 最近邻的距离
            kdtree_.nearestKSearch(pt, 5, nn_idx, dis);

            std::vector<Vec2d> effective_pts;  // 有效点
            // 遍历所有五个近邻点
            for (int j = 0; j < nn_idx.size(); ++j) {
                // 判断每个近邻点的距离是否处于最远阈值距离内
                if (dis[j] < max_dis) 
                    // 若是，该近邻点符合要求，存储到向量中
                    effective_pts.emplace_back(Vec2d(target_cloud_->points[nn_idx[j]].x, target_cloud_->points[nn_idx[j]].y));
            }
            // 判断有效近邻点是否少于三个
            if (effective_pts.size() < 3) 
                // 若少于3个，则跳过当前激光点
                continue;

            // 拟合直线，组装J、H和误差
            Vec3d line_coeffs;
            // 利用当前点附近的几个有效近邻点，基于SVD奇异值分解，拟合出ax+by+c=0 中的最小直线系数 a,b,c，对应公式（6.11）
            if (math::FitLine2D(effective_pts, line_coeffs)) {
                effective_num++; // 有效点数
                auto *edge = new EdgeSE2P2L(range, angle, line_coeffs);
                edge->setVertex(0, v);                  // 设置边的第一个顶点为SE2位姿顶点
                edge->setInformation(Eigen::Matrix<double, 1, 1>::Identity());// 观测为2维点坐标，因此信息矩阵需设为2x2单位矩阵
                auto rk = new g2o::RobustKernelHuber;   // Huber鲁棒核函数
                rk->setDelta(rk_delta);                 // 设置阈值
                edge->setRobustKernel(rk);              // 为边设置鲁棒核函数
                optimizer.addEdge(edge);                // 将约束边添加到优化器中
            }
        }

        // 判断有效激光点数是否少于最小有效点数阈值
        if (effective_num < min_effect_pts) 
            return false;

        optimizer.setVerbose(true);        // 不输出优化过程
        optimizer.initializeOptimization(); // 初始化优化器
        optimizer.optimize(1);              // g2o内部仅非线性优化求解一次

        // 取出优化后的SE2位姿，更新当前位姿，用于下一次迭代
        current_pose = v->estimate();
    }
    init_pose = current_pose;
    LOG(INFO) << "estimated pose: " << current_pose.translation().transpose() << ", theta: " << current_pose.so2().log();
    // LOG(INFO) << "g2o: estimated pose: " << init_pose.translation().transpose() << ", theta: " << init_pose.so2().log();
    return true;
}

void Icp2d::BuildTargetKdTree() {
    if (target_scan_ == nullptr) {
        LOG(ERROR) << "target is not set";
        return;
    }

    target_cloud_.reset(new Cloud2d);
    for (size_t i = 0; i < target_scan_->ranges.size(); ++i) {
        if (target_scan_->ranges[i] < target_scan_->range_min || target_scan_->ranges[i] > target_scan_->range_max) {
            continue;
        }

        double real_angle = target_scan_->angle_min + i * target_scan_->angle_increment;

        Point2d p;
        p.x = target_scan_->ranges[i] * std::cos(real_angle);
        p.y = target_scan_->ranges[i] * std::sin(real_angle);
        target_cloud_->points.push_back(p);
    }

    target_cloud_->width = target_cloud_->points.size();
    target_cloud_->is_dense = false;
    kdtree_.setInputCloud(target_cloud_);
}

}  // namespace sad