#pragma once

#include <Eigen/Dense>
#include <chrono>
#include <string>

/**
 * @brief 目标状态预测模块
 *
 * 作用：
 * 1. 接收 Solver 输出的 armor.xyz_in_world
 * 2. 使用 KF/EKF 思想进行状态滤波
 * 3. 输出当前滤波位置 filtered_pos
 * 4. 输出未来预测位置 predicted_pos
 *
 * 当前版本使用常速度模型：
 * state = [x, vx, y, vy, z, vz]^T
 *
 * measurement = [x, y, z]^T
 */
class Predictor {
public:
    using TimePoint = std::chrono::steady_clock::time_point;

    enum class TrackState {
        LOST = 0,       // 没有目标
        DETECTING = 1,  // 刚检测到目标，尚未稳定
        TRACKING = 2    // 稳定追踪
    };

    struct Config {
        // 进入 TRACKING 状态所需连续检测帧数
        int tracking_threshold = 3;

        // 连续丢失多少帧后清空目标
        int max_lost_count = 10;

        // 允许的最大相邻帧跳变距离，单位 m
        double max_jump_distance = 1.5;

        // 默认 dt，单位 s
        double default_dt = 0.02;

        // 最大 dt，超过则认为时间戳异常
        double max_dt = 0.2;

        // 过程噪声：位置
        double q_pos = 0.01;

        // 过程噪声：速度
        double q_vel = 0.1;

        // 观测噪声：Solver 位置测量噪声
        double r_pos = 0.05;
    };

public:
    Predictor();
    explicit Predictor(const Config& config);

    /**
     * @brief 有观测值时调用
     * @param measurement Solver 解算出的世界系位置 armor.xyz_in_world
     * @param timestamp 当前帧时间戳
     */
    void update(const Eigen::Vector3d& measurement, TimePoint timestamp);

    /**
     * @brief 当前帧没检测到目标时调用
     * 用于短时间丢帧时继续预测
     */
    void updateWithoutMeasurement(TimePoint timestamp);

    /**
     * @brief 手动清空状态
     */
    void reset();

    /**
     * @brief 获取滤波后的当前位置
     */
    Eigen::Vector3d filteredPos() const;

    /**
     * @brief 获取 delay_time 秒后的预测位置
     */
    Eigen::Vector3d predictedPos(double delay_time) const;

    /**
     * @brief 获取当前速度估计
     */
    Eigen::Vector3d velocity() const;

    /**
     * @brief 当前是否已经初始化
     */
    bool initialized() const;

    /**
     * @brief 当前是否处于稳定追踪状态
     */
    bool isTracking() const;

    /**
     * @brief 获取当前状态
     */
    TrackState state() const;

    /**
     * @brief 状态转字符串，方便调试打印
     */
    std::string stateString() const;

private:
    void initMatrices();
    void resetWithMeasurement(const Eigen::Vector3d& pos, TimePoint timestamp);

    void predict(double dt);
    void correct(const Eigen::Vector3d& measurement);

    double computeDt(TimePoint timestamp);

private:
    Config config_;

    // 状态量：[x, vx, y, vy, z, vz]
    Eigen::Matrix<double, 6, 1> x_;

    // 状态协方差
    Eigen::Matrix<double, 6, 6> P_;

    // 状态转移矩阵
    Eigen::Matrix<double, 6, 6> F_;

    // 过程噪声
    Eigen::Matrix<double, 6, 6> Q_;

    // 观测矩阵
    Eigen::Matrix<double, 3, 6> H_;

    // 观测噪声
    Eigen::Matrix<double, 3, 3> R_;

    // 单位矩阵
    Eigen::Matrix<double, 6, 6> I_;

    bool initialized_ = false;
    TimePoint last_time_;

    TrackState state_ = TrackState::LOST;

    int detect_count_ = 0;
    int lost_count_ = 0;
};