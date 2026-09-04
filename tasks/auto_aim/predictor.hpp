#ifndef AUTO_AIM__PREDICTOR_HPP
#define AUTO_AIM__PREDICTOR_HPP

#include <Eigen/Dense>

#include <chrono>
#include <string>

namespace auto_aim
{

class Predictor
{
public:
  using TimePoint = std::chrono::steady_clock::time_point;

  enum class TrackState
  {
    LOST = 0,
    DETECTING = 1,
    TRACKING = 2
  };

  struct Config
  {
    int tracking_threshold = 3;
    int max_lost_count = 10;
    double max_jump_distance = 1.5;
    double default_dt = 0.02;
    double max_dt = 0.2;
    double q_pos = 0.01;
    double q_vel = 0.1;
    double r_pos = 0.05;
  };

  Predictor();
  explicit Predictor(const std::string & config_path);
  explicit Predictor(const Config & config);

  void update(const Eigen::Vector3d & measurement, TimePoint timestamp);
  void updateWithoutMeasurement(TimePoint timestamp);
  void reset();

  Eigen::Vector3d filteredPos() const;
  Eigen::Vector3d predictedPos(double delay_time) const;
  Eigen::Vector3d velocity() const;

  bool initialized() const;
  bool isTracking() const;
  TrackState state() const;
  std::string stateString() const;

private:
  void initMatrices();
  void resetWithMeasurement(const Eigen::Vector3d & pos, TimePoint timestamp);

  void predict(double dt);
  void correct(const Eigen::Vector3d & measurement);

  double computeDt(TimePoint timestamp);

private:
  Config config_;

  Eigen::Matrix<double, 6, 1> x_;
  Eigen::Matrix<double, 6, 6> P_;
  Eigen::Matrix<double, 6, 6> F_;
  Eigen::Matrix<double, 6, 6> Q_;
  Eigen::Matrix<double, 3, 6> H_;
  Eigen::Matrix<double, 3, 3> R_;
  Eigen::Matrix<double, 6, 6> I_;

  bool initialized_ = false;
  TimePoint last_time_;

  TrackState state_ = TrackState::LOST;

  int detect_count_ = 0;
  int lost_count_ = 0;
};

}  // namespace auto_aim

#endif  // AUTO_AIM__PREDICTOR_HPP
