#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{

struct Trajectory
{
  bool unsolvable;
  double fly_time;
  double pitch;
  Trajectory(const double v0, const double d, const double h);
};

// 新增：带空气阻力的弹道解算
struct TrajectoryWithDrag
{
  bool unsolvable;
  double fly_time;
  double pitch;
  // v0 子弹初速度，d 水平距离，h 高度差，k 空气阻力系数
  TrajectoryWithDrag(double v0, double d, double h, double k);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP