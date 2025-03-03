//
// Created by xiang on 2021/7/21.
//

#ifndef SLAM_IN_AUTO_DRIVING_SYS_UTILS_H
#define SLAM_IN_AUTO_DRIVING_SYS_UTILS_H

#include <glog/logging.h>
#include <chrono>

namespace sad {

// 一些系统相关函数，统计代码时间之类的功能

/**
 * 统计代码运行时间
 * @tparam FuncT
 * @param func  被调用函数
 * @param func_name 函数名
 * @param times 调用次数
 */
template <typename FuncT>
void evaluate_and_call(FuncT&& func, const std::string& func_name = "", int times = 10) {
    double total_time = 0;
    for (int i = 0; i < times; ++i) {
        auto t1 = std::chrono::high_resolution_clock::now();
        func();
        auto t2 = std::chrono::high_resolution_clock::now();
        total_time += std::chrono::duration_cast<std::chrono::duration<double>>(t2 - t1).count() * 1000;
    }

    LOG(INFO) << "method " << func_name << " average time / times: " << total_time / times << "/" << times << " ms.";
}

}  // namespace sad

#endif  // SLAM_IN_AUTO_DRIVING_SYS_UTILS_H
