#include "shooter.hpp"

#include <yaml-cpp/yaml.h>

#include "tools/logger.hpp"
#include "tools/math_tools.hpp"

namespace auto_aim
{
Shooter::Shooter(const std::string & config_path) : last_command_{false, false, 0, 0}
{
  auto yaml = YAML::LoadFile(config_path);
  first_tolerance_ = yaml["first_tolerance"].as<double>() * M_PI / 180.0;  
  second_tolerance_ = yaml["second_tolerance"].as<double>() * M_PI / 180.0;  //用π/180来转弧度更精准
  judge_distance_ = yaml["judge_distance"].as<double>();
  auto_fire_ = yaml["auto_fire"].as<bool>();
  fire_threshold_ = yaml["fire_threshold"].as<int>(3);  // 默认连续3帧
}

bool Shooter::shoot(
  const io::Command & command, const auto_aim::Aimer & aimer,
  const std::list<auto_aim::Target> & targets, const Eigen::Vector3d & gimbal_pos)
{
  if (!command.control || targets.empty() || !auto_fire_) return false;

  auto target_x = targets.front().ekf_x()[0];
  auto target_y = targets.front().ekf_x()[2];
  auto tolerance = std::sqrt(tools::square(target_x) + tools::square(target_y)) > judge_distance_
                     ? second_tolerance_
                     : first_tolerance_;
  // tools::logger()->debug("d(command.yaw) is {:.4f}", std::abs(last_command_.yaw - command.yaw));

  if (
    std::abs(last_command_.yaw - command.yaw) < tolerance * 2 &&  //此时认为command突变不应该射击
    std::abs(gimbal_pos[0] - last_command_.yaw) < tolerance &&  //应该减去上一次command的yaw值
    std::abs(gimbal_pos[1] - last_command_.pitch) < tolerance &&  //确保yaw同步对齐
    aimer.debug_aim_point.valid) {
    fire_count_++;                              // 满足条件就累加
    last_command_ = command;
    return fire_count_ >= fire_threshold_;     // 达到阈值才真正开火
  }
  
  // 不满足条件时清零
  fire_count_ = 0;
  last_command_ = command;
  return false;

} //shoot函数结束
}  // namespace auto_aim
