#ifndef TOOLS__TRAJECTORY_HPP
#define TOOLS__TRAJECTORY_HPP

namespace tools
{
struct Trajectory
{
  bool unsolvable = true;
  double fly_time = 0.0;
  double pitch = 0.0;       // 抬头为正
  double confidence = 0.0;  // 0~1，描述当前弹道解可信度

  // 不考虑空气阻力
  // v0 子弹初速度大小，单位：m/s
  // d 目标水平距离，单位：m
  // h 目标竖直高度，单位：m
  Trajectory(const double v0, const double d, const double h);

  // 二次空气阻力模型：a_drag = -drag_coefficient * |v| * v
  // drag_coefficient 单位近似为 1/m，需要按实车打靶数据标定
  // drag_coefficient <= 0 时退化为原无阻力解析模型
  Trajectory(const double v0, const double d, const double h, const double drag_coefficient);
};

}  // namespace tools

#endif  // TOOLS__TRAJECTORY_HPP
