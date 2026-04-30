#include "predictor.hpp"

#include <cmath>

Predictor::Predictor()
{
    initMatrices();
}

Predictor::Predictor(const Config& config)
    : config_(config)
{
    initMatrices();
}

void Predictor::initMatrices()
{
    x_.setZero();

    P_.setIdentity();
    P_ *= 1.0;

    F_.setIdentity();
    Q_.setZero();
    H_.setZero();
    R_.setZero();
    I_.setIdentity();

    // measurement = [x, y, z]
    // state       = [x, vx, y, vy, z, vz]
    H_(0, 0) = 1.0;
    H_(1, 2) = 1.0;
    H_(2, 4) = 1.0;

    Q_(0, 0) = config_.q_pos;
    Q_(1, 1) = config_.q_vel;
    Q_(2, 2) = config_.q_pos;
    Q_(3, 3) = config_.q_vel;
    Q_(4, 4) = config_.q_pos;
    Q_(5, 5) = config_.q_vel;

    R_(0, 0) = config_.r_pos;
    R_(1, 1) = config_.r_pos;
    R_(2, 2) = config_.r_pos;
}

void Predictor::update(const Eigen::Vector3d& measurement, TimePoint timestamp)
{
    if (!initialized_) {
        resetWithMeasurement(measurement, timestamp);
        return;
    }

    const double dt = computeDt(timestamp);

    predict(dt);

    const double jump_distance = (measurement - filteredPos()).norm();

    // 异常点拒绝：防止 Solver 偶发跳点把滤波器带飞
    if (jump_distance > config_.max_jump_distance) {
        lost_count_++;

        if (lost_count_ > config_.max_lost_count) {
            resetWithMeasurement(measurement, timestamp);
        }

        return;
    }

    correct(measurement);

    detect_count_++;
    lost_count_ = 0;

    if (detect_count_ >= config_.tracking_threshold) {
        state_ = TrackState::TRACKING;
    } else {
        state_ = TrackState::DETECTING;
    }
}

void Predictor::updateWithoutMeasurement(TimePoint timestamp)
{
    if (!initialized_) {
        return;
    }

    const double dt = computeDt(timestamp);

    predict(dt);

    lost_count_++;

    if (lost_count_ > config_.max_lost_count) {
        reset();
        return;
    }

    if (state_ == TrackState::TRACKING) {
        state_ = TrackState::DETECTING;
    }
}

void Predictor::reset()
{
    x_.setZero();

    P_.setIdentity();
    P_ *= 1.0;

    initialized_ = false;
    state_ = TrackState::LOST;

    detect_count_ = 0;
    lost_count_ = 0;
}

void Predictor::resetWithMeasurement(const Eigen::Vector3d& pos, TimePoint timestamp)
{
    x_.setZero();

    // 初始位置来自 Solver
    x_(0) = pos.x();
    x_(2) = pos.y();
    x_(4) = pos.z();

    // 初始速度未知，设为 0
    x_(1) = 0.0;
    x_(3) = 0.0;
    x_(5) = 0.0;

    P_.setIdentity();
    P_ *= 1.0;

    initialized_ = true;
    last_time_ = timestamp;

    detect_count_ = 1;
    lost_count_ = 0;

    state_ = TrackState::DETECTING;
}

double Predictor::computeDt(TimePoint timestamp)
{
    double dt = std::chrono::duration<double>(timestamp - last_time_).count();
    last_time_ = timestamp;

    if (dt <= 0.0 || dt > config_.max_dt) {
        dt = config_.default_dt;
    }

    return dt;
}

void Predictor::predict(double dt)
{
    // 常速度运动模型：
    // x = x + vx * dt
    // y = y + vy * dt
    // z = z + vz * dt

    F_.setIdentity();

    F_(0, 1) = dt;
    F_(2, 3) = dt;
    F_(4, 5) = dt;

    x_ = F_ * x_;

    P_ = F_ * P_ * F_.transpose() + Q_;
}

void Predictor::correct(const Eigen::Vector3d& measurement)
{
    // 预测观测
    Eigen::Vector3d z_pred = H_ * x_;

    // 残差
    Eigen::Vector3d residual = measurement - z_pred;

    // 残差协方差
    Eigen::Matrix3d S = H_ * P_ * H_.transpose() + R_;

    // 卡尔曼增益
    Eigen::Matrix<double, 6, 3> K = P_ * H_.transpose() * S.inverse();

    // 状态更新
    x_ = x_ + K * residual;

    // 协方差更新
    P_ = (I_ - K * H_) * P_;
}

Eigen::Vector3d Predictor::filteredPos() const
{
    return Eigen::Vector3d(
        x_(0),
        x_(2),
        x_(4)
    );
}

Eigen::Vector3d Predictor::predictedPos(double delay_time) const
{
    if (!initialized_) {
        return Eigen::Vector3d::Zero();
    }

    return Eigen::Vector3d(
        x_(0) + x_(1) * delay_time,
        x_(2) + x_(3) * delay_time,
        x_(4) + x_(5) * delay_time
    );
}

Eigen::Vector3d Predictor::velocity() const
{
    return Eigen::Vector3d(
        x_(1),
        x_(3),
        x_(5)
    );
}

bool Predictor::initialized() const
{
    return initialized_;
}

bool Predictor::isTracking() const
{
    return state_ == TrackState::TRACKING;
}

Predictor::TrackState Predictor::state() const
{
    return state_;
}

std::string Predictor::stateString() const
{
    switch (state_) {
        case TrackState::LOST:
            return "LOST";
        case TrackState::DETECTING:
            return "DETECTING";
        case TrackState::TRACKING:
            return "TRACKING";
        default:
            return "UNKNOWN";
    }
}