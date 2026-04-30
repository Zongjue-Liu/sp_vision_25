#include "trajectory.hpp"

#include <cmath>

namespace tools
{
namespace
{
constexpr double kGravity = 9.7833;
constexpr double kPi = 3.14159265358979323846;
constexpr double kEps = 1e-9;
constexpr double kMinPitch = -0.8;  // rad，低抛搜索下限
constexpr double kMaxPitch = 1.25;  // rad，低抛搜索上限
constexpr double kIntegrateDt = 0.003;
constexpr double kMaxFlyTime = 3.0;
constexpr int kSearchGrid = 120;
constexpr int kBinaryIter = 32;

struct ImpactResult
{
  bool reachable = false;
  double z = 0.0;
  double t = 0.0;
};

double clamp(const double value, const double low, const double high)
{
  if (value < low) return low;
  if (value > high) return high;
  return value;
}

bool solve_without_drag(
  const double v0, const double d, const double h, double & pitch, double & fly_time)
{
  if (v0 <= kEps || d < 0.0) return false;

  if (d < kEps) {
    pitch = (h >= 0.0) ? kPi / 2.0 : -kPi / 2.0;
    fly_time = std::abs(h) / v0;
    return std::isfinite(fly_time);
  }

  const double a = kGravity * d * d / (2.0 * v0 * v0);
  const double b = -d;
  const double c = a + h;
  const double delta = b * b - 4.0 * a * c;

  if (delta < 0.0) return false;

  const double sqrt_delta = std::sqrt(delta);
  const double tan_pitch_1 = (-b + sqrt_delta) / (2.0 * a);
  const double tan_pitch_2 = (-b - sqrt_delta) / (2.0 * a);
  const double pitch_1 = std::atan(tan_pitch_1);
  const double pitch_2 = std::atan(tan_pitch_2);
  const double t_1 = d / (v0 * std::cos(pitch_1));
  const double t_2 = d / (v0 * std::cos(pitch_2));

  if (t_1 < t_2) {
    pitch = pitch_1;
    fly_time = t_1;
  } else {
    pitch = pitch_2;
    fly_time = t_2;
  }

  return std::isfinite(pitch) && std::isfinite(fly_time) && fly_time >= 0.0;
}

ImpactResult simulate_with_drag(
  const double v0, const double d, const double pitch, const double drag_coefficient)
{
  ImpactResult result;
  if (v0 <= kEps || d < 0.0) return result;

  double x = 0.0;
  double z = 0.0;
  double vx = v0 * std::cos(pitch);
  double vz = v0 * std::sin(pitch);
  double t = 0.0;

  if (vx <= kEps) return result;

  while (t < kMaxFlyTime) {
    const double speed = std::hypot(vx, vz);
    const double ax = -drag_coefficient * speed * vx;
    const double az = -kGravity - drag_coefficient * speed * vz;

    const double next_x = x + vx * kIntegrateDt + 0.5 * ax * kIntegrateDt * kIntegrateDt;
    const double next_z = z + vz * kIntegrateDt + 0.5 * az * kIntegrateDt * kIntegrateDt;
    const double next_vx = vx + ax * kIntegrateDt;
    const double next_vz = vz + az * kIntegrateDt;

    if (next_x >= d) {
      const double ratio = (d - x) / (next_x - x);
      result.reachable = true;
      result.z = z + ratio * (next_z - z);
      result.t = t + ratio * kIntegrateDt;
      return result;
    }

    if (next_x <= x + kEps || next_vx <= kEps) return result;

    x = next_x;
    z = next_z;
    vx = next_vx;
    vz = next_vz;
    t += kIntegrateDt;
  }

  return result;
}

bool solve_with_drag(
  const double v0, const double d, const double h, const double drag_coefficient, double & pitch,
  double & fly_time)
{
  if (drag_coefficient <= 0.0) return solve_without_drag(v0, d, h, pitch, fly_time);
  if (v0 <= kEps || d < 0.0) return false;
  if (d < kEps) return solve_without_drag(v0, d, h, pitch, fly_time);

  bool has_prev = false;
  double prev_pitch = kMinPitch;
  double prev_error = 0.0;
  double low = 0.0;
  double high = 0.0;
  bool bracket_found = false;

  // 从低角度到高角度扫描，取第一个过零区间，即低抛解。
  for (int i = 0; i <= kSearchGrid; ++i) {
    const double ratio = static_cast<double>(i) / static_cast<double>(kSearchGrid);
    const double cur_pitch = kMinPitch + (kMaxPitch - kMinPitch) * ratio;
    const auto impact = simulate_with_drag(v0, d, cur_pitch, drag_coefficient);
    if (!impact.reachable) continue;

    const double cur_error = impact.z - h;
    if (std::abs(cur_error) < 1e-4) {
      pitch = cur_pitch;
      fly_time = impact.t;
      return true;
    }

    if (has_prev && prev_error <= 0.0 && cur_error >= 0.0) {
      low = prev_pitch;
      high = cur_pitch;
      bracket_found = true;
      break;
    }

    has_prev = true;
    prev_pitch = cur_pitch;
    prev_error = cur_error;
  }

  if (!bracket_found) return false;

  for (int i = 0; i < kBinaryIter; ++i) {
    const double mid = (low + high) * 0.5;
    const auto impact = simulate_with_drag(v0, d, mid, drag_coefficient);
    if (!impact.reachable || impact.z < h) {
      low = mid;
    } else {
      high = mid;
    }
  }

  const auto impact = simulate_with_drag(v0, d, high, drag_coefficient);
  if (!impact.reachable) return false;

  pitch = high;
  fly_time = impact.t;
  return std::isfinite(pitch) && std::isfinite(fly_time) && fly_time >= 0.0;
}

double estimate_confidence(const double v0, const double d, const double pitch, const double fly_time)
{
  if (!std::isfinite(v0) || !std::isfinite(d) || !std::isfinite(pitch) || !std::isfinite(fly_time)) {
    return 0.0;
  }

  double confidence = 1.0;

  // 远距离、长飞行时间、过大抬角、异常弹速都会降低弹道解可信度。
  if (d > 8.0) confidence *= clamp(1.0 - 0.06 * (d - 8.0), 0.45, 1.0);
  if (fly_time > 0.35) confidence *= clamp(1.0 - 1.2 * (fly_time - 0.35), 0.45, 1.0);
  if (std::abs(pitch) > 0.35) confidence *= clamp(1.0 - 1.5 * (std::abs(pitch) - 0.35), 0.45, 1.0);
  if (v0 < 12.0 || v0 > 30.0) confidence *= 0.65;

  return clamp(confidence, 0.0, 1.0);
}
}  // namespace

Trajectory::Trajectory(const double v0, const double d, const double h) : Trajectory(v0, d, h, 0.0)
{
}

Trajectory::Trajectory(
  const double v0, const double d, const double h, const double drag_coefficient)
{
  unsolvable = !solve_with_drag(v0, d, h, drag_coefficient, pitch, fly_time);
  confidence = unsolvable ? 0.0 : estimate_confidence(v0, d, pitch, fly_time);
}

}  // namespace tools
